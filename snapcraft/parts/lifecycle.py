# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Parts lifecycle preparation and execution."""

import copy
import os
import shutil
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Tuple

import craft_parts
from craft_cli import emit
from craft_parts import ProjectInfo, Step, StepInfo, callbacks
from craft_providers import Executor

from snapcraft import errors, extensions, linters, pack, providers, ua_manager, utils
from snapcraft.elf import Patcher, SonameCache, elf_utils
from snapcraft.elf import errors as elf_errors
from snapcraft.linters import LinterStatus
from snapcraft.meta import manifest, snap_yaml
from snapcraft.projects import (
    Architecture,
    ArchitectureProject,
    GrammarAwareProject,
    Project,
)
from snapcraft.utils import (
    convert_architecture_deb_to_platform,
    get_host_architecture,
    process_version,
)

from . import grammar, yaml_utils
from .parts import PartsLifecycle, launch_shell
from .project_check import run_project_checks
from .setup_assets import setup_assets
from .update_metadata import update_project_metadata

if TYPE_CHECKING:
    import argparse


@dataclass
class _SnapProject:
    project_file: Path
    assets_dir: Path = Path("snap")


_SNAP_PROJECT_FILES = [
    _SnapProject(project_file=Path("snapcraft.yaml")),
    _SnapProject(project_file=Path("snap/snapcraft.yaml")),
    _SnapProject(
        project_file=Path("build-aux/snap/snapcraft.yaml"),
        assets_dir=Path("build-aux/snap"),
    ),
    _SnapProject(project_file=Path(".snapcraft.yaml")),
]

_CORE_PART_KEYS = ["build-packages", "build-snaps"]
_CORE_PART_NAME = "snapcraft/core"


def get_snap_project() -> _SnapProject:
    """Find the snapcraft.yaml to load.

    :raises SnapcraftError: if the project yaml file cannot be found.
    """
    for snap_project in _SNAP_PROJECT_FILES:
        if snap_project.project_file.exists():
            return snap_project

    raise errors.ProjectMissing()


def apply_yaml(
    yaml_data: Dict[str, Any], build_on: str, build_for: str
) -> Dict[str, Any]:
    """Apply Snapcraft logic to yaml_data.

    Extensions are applied and advanced grammar is processed.
    The architectures data is reduced to architectures in the current build plan.

    :param yaml_data: The project YAML data.
    :param build_on: Architecture the snap project will be built on.
    :param build_for: Target architecture the snap project will be built to.

    :return: A dictionary of yaml data with snapcraft logic applied.
    """
    # validate project grammar
    GrammarAwareProject.validate_grammar(yaml_data)

    # Special Snapcraft Part
    core_part = {k: yaml_data.pop(k) for k in _CORE_PART_KEYS if k in yaml_data}
    if core_part:
        core_part["plugin"] = "nil"
        yaml_data["parts"][_CORE_PART_NAME] = core_part

    yaml_data = extensions.apply_extensions(
        yaml_data, arch=build_on, target_arch=build_for
    )

    if "parts" in yaml_data:
        yaml_data["parts"] = grammar.process_parts(
            parts_yaml_data=yaml_data["parts"], arch=build_on, target_arch=build_for
        )

    # replace all architectures with the architectures in the current build plan
    yaml_data["architectures"] = [Architecture(build_on=build_on, build_for=build_for)]

    return yaml_data


def process_yaml(project_file: Path) -> Dict[str, Any]:
    """Process yaml data from file into a dictionary.

    :param project_file: Path to project.

    :raises SnapcraftError: if the project yaml file cannot be loaded.

    :return: The processed YAML data.
    """
    try:
        with open(project_file, encoding="utf-8") as yaml_file:
            yaml_data = yaml_utils.load(yaml_file)
    except OSError as err:
        msg = err.strerror
        if err.filename:
            msg = f"{msg}: {err.filename!r}."
        raise errors.SnapcraftError(msg) from err

    return yaml_data


def extract_parse_info(yaml_data: Dict[str, Any]) -> Dict[str, List[str]]:
    """Remove parse-info data from parts.

    :param yaml_data: The project YAML data.

    :return: The extracted parse info for each part.
    """
    parse_info: Dict[str, List[str]] = {}

    if "parts" in yaml_data:
        for name, data in yaml_data["parts"].items():
            if "parse-info" in data:
                parse_info[name] = data.pop("parse-info")

    return parse_info


def run(command_name: str, parsed_args: "argparse.Namespace") -> None:
    """Run the parts lifecycle.

    :raises SnapcraftError: if the step name is invalid, or the project
        yaml file cannot be loaded.
    :raises LegacyFallback: if the project's base is not core22.
    """
    emit.debug(f"command: {command_name}, arguments: {parsed_args}")

    snap_project = get_snap_project()
    yaml_data = process_yaml(snap_project.project_file)
    start_time = datetime.now()

    if parsed_args.provider:
        raise errors.SnapcraftError("Option --provider is not supported.")

    if yaml_data.get("ua-services"):
        if not parsed_args.ua_token:
            raise errors.SnapcraftError(
                "UA services require a UA token to be specified."
            )

        if not parsed_args.enable_experimental_ua_services:
            raise errors.SnapcraftError(
                "Using UA services requires --enable-experimental-ua-services."
            )

    build_plan = get_build_plan(yaml_data, parsed_args)

    # Register our own callbacks
    callbacks.register_prologue(_set_global_environment)
    callbacks.register_pre_step(_set_step_environment)
    callbacks.register_post_step(_patch_elf, step_list=[Step.PRIME])

    build_count = utils.get_parallel_build_count()

    for build_on, build_for in build_plan:
        emit.verbose(f"Running on {build_on} for {build_for}")
        yaml_data_for_arch = apply_yaml(yaml_data, build_on, build_for)
        parse_info = extract_parse_info(yaml_data_for_arch)
        _expand_environment(
            yaml_data_for_arch,
            parallel_build_count=build_count,
            target_arch=build_for,
        )
        project = Project.unmarshal(yaml_data_for_arch)

        _run_command(
            command_name,
            project=project,
            parse_info=parse_info,
            parallel_build_count=build_count,
            assets_dir=snap_project.assets_dir,
            start_time=start_time,
            parsed_args=parsed_args,
        )


def _run_command(
    command_name: str,
    *,
    project: Project,
    parse_info: Dict[str, List[str]],
    assets_dir: Path,
    start_time: datetime,
    parallel_build_count: int,
    parsed_args: "argparse.Namespace",
) -> None:
    managed_mode = utils.is_managed_mode()
    part_names = getattr(parsed_args, "parts", None)

    if not managed_mode:
        run_project_checks(project, assets_dir=assets_dir)

        if command_name == "snap":
            emit.progress(
                "The 'snap' command is deprecated, use 'pack' instead.",
                permanent=True,
            )

    if (
        not managed_mode
        and not parsed_args.destructive_mode
        and not os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "host"
    ):
        if command_name == "clean" and not part_names:
            _clean_provider(project, parsed_args)
        else:
            _run_in_provider(project, command_name, parsed_args)
        return

    if managed_mode:
        work_dir = utils.get_managed_environment_home_path()
        project_dir = utils.get_managed_environment_project_path()
    else:
        work_dir = project_dir = Path.cwd()

    step_name = "prime" if command_name in ("pack", "snap", "try") else command_name

    track_stage_packages = getattr(parsed_args, "enable_manifest", False)

    lifecycle = PartsLifecycle(
        project.parts,
        work_dir=work_dir,
        assets_dir=assets_dir,
        base=project.get_effective_base(),
        project_base=project.base or "",
        confinement=project.confinement,
        package_repositories=project.package_repositories,
        parallel_build_count=parallel_build_count,
        part_names=part_names,
        adopt_info=project.adopt_info,
        project_name=project.name,
        parse_info=parse_info,
        project_vars={
            "version": project.version or "",
            "grade": project.grade or "",
        },
        extra_build_snaps=project.get_extra_build_snaps(),
        target_arch=project.get_build_for(),
        track_stage_packages=track_stage_packages,
    )

    if command_name == "clean":
        lifecycle.clean(part_names=part_names)
        return

    try:
        _run_lifecycle_and_pack(
            lifecycle,
            command_name=command_name,
            step_name=step_name,
            project=project,
            project_dir=project_dir,
            assets_dir=assets_dir,
            start_time=start_time,
            parsed_args=parsed_args,
        )
    except PermissionError as err:
        if parsed_args.debug:
            emit.progress(str(err), permanent=True)
            launch_shell()
        raise errors.FilePermissionError(err.filename, reason=err.strerror)
    except OSError as err:
        msg = err.strerror
        if err.filename:
            msg = f"{err.filename}: {msg}"
        if parsed_args.debug:
            emit.progress(msg, permanent=True)
            launch_shell()
        raise errors.SnapcraftError(msg) from err
    except Exception as err:
        if parsed_args.debug:
            emit.progress(str(err), permanent=True)
            launch_shell()
        raise errors.SnapcraftError(str(err)) from err


def _run_lifecycle_and_pack(
    lifecycle: PartsLifecycle,
    *,
    command_name: str,
    step_name: str,
    project: Project,
    project_dir: Path,
    assets_dir: Path,
    start_time: datetime,
    parsed_args: "argparse.Namespace",
) -> None:
    """Execute the parts lifecycle, generate metadata, and create the snap."""
    with ua_manager.ua_manager(parsed_args.ua_token, services=project.ua_services):
        lifecycle.run(
            step_name,
            shell=getattr(parsed_args, "shell", False),
            shell_after=getattr(parsed_args, "shell_after", False),
        )

    # Extract metadata and generate snap.yaml
    part_names = getattr(parsed_args, "part_names", None)

    if step_name == "prime" and not part_names:
        _generate_metadata(
            project=project,
            lifecycle=lifecycle,
            project_dir=project_dir,
            assets_dir=assets_dir,
            start_time=start_time,
            parsed_args=parsed_args,
        )

    if command_name in ("pack", "snap"):
        issues = linters.run_linters(lifecycle.prime_dir, lint=project.lint)
        status = linters.report(issues, intermediate=True)

        # In case of linter errors, stop execution and return the error code.
        if status in (LinterStatus.ERRORS, LinterStatus.FATAL):
            raise errors.LinterError("Linter errors found", exit_code=status)

        snap_filename = pack.pack_snap(
            lifecycle.prime_dir,
            output=parsed_args.output,
            compression=project.compression,
            name=project.name,
            version=process_version(project.version),
            target_arch=project.get_build_for(),
        )
        emit.message(f"Created snap package {snap_filename}")


def _generate_metadata(
    *,
    project: Project,
    lifecycle: PartsLifecycle,
    project_dir: Path,
    assets_dir: Path,
    start_time: datetime,
    parsed_args: "argparse.Namespace",
):
    project_vars = lifecycle.project_vars

    emit.progress("Extracting and updating metadata...")
    metadata_list = lifecycle.extract_metadata()
    update_project_metadata(
        project,
        project_vars=project_vars,
        metadata_list=metadata_list,
        assets_dir=assets_dir,
        prime_dir=lifecycle.prime_dir,
    )

    emit.progress("Copying snap assets...")
    setup_assets(
        project,
        assets_dir=assets_dir,
        project_dir=project_dir,
        prime_dir=lifecycle.prime_dir,
    )

    emit.progress("Generating snap metadata...")
    snap_yaml.write(
        project,
        lifecycle.prime_dir,
        arch=lifecycle.target_arch,
        arch_triplet=lifecycle.target_arch_triplet,
    )
    emit.progress("Generated snap metadata", permanent=True)

    if parsed_args.enable_manifest:
        _generate_manifest(
            project,
            lifecycle=lifecycle,
            start_time=start_time,
            parsed_args=parsed_args,
        )


def _generate_manifest(
    project: Project,
    *,
    lifecycle: PartsLifecycle,
    start_time: datetime,
    parsed_args: "argparse.Namespace",
) -> None:
    """Create and populate the manifest file."""
    emit.progress("Generating snap manifest...")
    image_information = parsed_args.manifest_image_information or "{}"

    parts = copy.deepcopy(project.parts)
    for name, part in parts.items():
        assets = lifecycle.get_part_pull_assets(part_name=name)
        if assets:
            part["stage-packages"] = assets.get("stage-packages", []) or []
        for key in ("stage", "prime", "stage-packages", "build-packages"):
            part.setdefault(key, [])

    manifest.write(
        project,
        lifecycle.prime_dir,
        arch=lifecycle.target_arch,
        parts=parts,
        start_time=start_time,
        image_information=image_information,
        primed_stage_packages=lifecycle.get_primed_stage_packages(),
    )
    emit.progress("Generated snap manifest", permanent=True)

    # Also copy the original snapcraft.yaml
    snap_project = get_snap_project()
    shutil.copy(snap_project.project_file, lifecycle.prime_dir / "snap")


def _clean_provider(project: Project, parsed_args: "argparse.Namespace") -> None:
    """Clean the provider environment.

    :param project: The project to clean.
    """
    emit.progress("Cleaning build provider")
    provider_name = "lxd" if parsed_args.use_lxd else None
    provider = providers.get_provider(provider_name)
    instance_name = providers.get_instance_name(
        project_name=project.name,
        project_path=Path().absolute(),
        build_on=project.get_build_on(),
        build_for=project.get_build_for(),
    )
    emit.debug(f"Cleaning instance {instance_name}")
    provider.clean_project_environments(instance_name=instance_name)
    emit.progress("Cleaned build provider", permanent=True)


# pylint: disable-next=too-many-branches, too-many-statements
def _run_in_provider(
    project: Project, command_name: str, parsed_args: "argparse.Namespace"
) -> None:
    """Pack image in provider instance."""
    emit.debug("Checking build provider availability")
    provider_name = "lxd" if parsed_args.use_lxd else None
    provider = providers.get_provider(provider_name)
    providers.ensure_provider_is_available(provider)

    cmd = ["snapcraft", command_name]

    if hasattr(parsed_args, "parts"):
        cmd.extend(parsed_args.parts)

    if getattr(parsed_args, "output", None):
        cmd.extend(["--output", parsed_args.output])

    mode = emit.get_mode().name.lower()
    cmd.append(f"--verbosity={mode}")

    if parsed_args.debug:
        cmd.append("--debug")
    if getattr(parsed_args, "shell", False):
        cmd.append("--shell")
    if getattr(parsed_args, "shell_after", False):
        cmd.append("--shell-after")

    if getattr(parsed_args, "enable_manifest", False):
        cmd.append("--enable-manifest")
    build_information = getattr(parsed_args, "manifest_build_information", None)
    if build_information:
        cmd.extend(["--manifest-build-information", build_information])

    cmd.append("--build-for")
    cmd.append(project.get_build_for())

    ua_token = getattr(parsed_args, "ua_token", "")
    if ua_token:
        cmd.extend(["--ua-token", ua_token])

    if getattr(parsed_args, "enable_experimental_ua_services", False):
        cmd.append("--enable-experimental-ua-services")

    project_path = Path().absolute()
    output_dir = utils.get_managed_environment_project_path()

    instance_name = providers.get_instance_name(
        project_name=project.name,
        project_path=project_path,
        build_on=project.get_build_on(),
        build_for=project.get_build_for(),
    )

    snapcraft_base = project.get_effective_base()
    build_base = providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE[snapcraft_base]

    if snapcraft_base == "devel":
        emit.progress(
            "Running snapcraft with a devel instance is for testing purposes only.",
            permanent=True,
        )
        allow_unstable = True
    else:
        allow_unstable = False

    base_configuration = providers.get_base_configuration(
        alias=build_base,
        instance_name=instance_name,
        http_proxy=parsed_args.http_proxy,
        https_proxy=parsed_args.https_proxy,
    )

    emit.progress("Launching instance...")
    with provider.launched_environment(
        project_name=project.name,
        project_path=project_path,
        base_configuration=base_configuration,
        build_base=build_base.value,
        instance_name=instance_name,
        allow_unstable=allow_unstable,
    ) as instance:
        try:
            providers.prepare_instance(
                instance=instance,
                host_project_path=project_path,
                bind_ssh=parsed_args.bind_ssh,
            )
            with emit.pause():
                if command_name == "try":
                    _expose_prime(project_path, instance)
                # run snapcraft inside the instance
                instance.execute_run(cmd, check=True, cwd=output_dir)
        except subprocess.CalledProcessError as err:
            raise errors.SnapcraftError(
                f"Failed to execute {command_name} in instance.",
                details=(
                    "Run the same command again with --debug to shell into "
                    "the environment if you wish to introspect this failure."
                ),
            ) from err
        finally:
            providers.capture_logs_from_instance(instance)


def _expose_prime(project_path: Path, instance: Executor):
    """Expose the instance's prime directory in ``project_path`` on the host."""
    host_prime = project_path / "prime"
    host_prime.mkdir(exist_ok=True)

    managed_root = utils.get_managed_environment_home_path()
    dirs = craft_parts.ProjectDirs(work_dir=managed_root)

    instance.mount(host_source=project_path / "prime", target=dirs.prime_dir)


def _set_global_environment(info: ProjectInfo) -> None:
    """Set global environment variables."""
    info.global_environment.update(
        {
            "SNAPCRAFT_ARCH_TRIPLET": info.arch_triplet,
            "SNAPCRAFT_TARGET_ARCH": info.target_arch,
            "SNAPCRAFT_PARALLEL_BUILD_COUNT": str(info.parallel_build_count),
            "SNAPCRAFT_PROJECT_VERSION": info.get_project_var("version", raw_read=True),
            "SNAPCRAFT_PROJECT_GRADE": info.get_project_var("grade", raw_read=True),
            "SNAPCRAFT_PROJECT_DIR": str(info.project_dir),
            "SNAPCRAFT_PROJECT_NAME": str(info.project_name),
            "SNAPCRAFT_STAGE": str(info.stage_dir),
            "SNAPCRAFT_PRIME": str(info.prime_dir),
        }
    )


def _set_step_environment(step_info: StepInfo) -> bool:
    """Set the step environment before executing each lifecycle step."""
    step_info.step_environment.update(
        {
            "SNAPCRAFT_PART_SRC": str(step_info.part_src_dir),
            "SNAPCRAFT_PART_SRC_WORK": str(step_info.part_src_subdir),
            "SNAPCRAFT_PART_BUILD": str(step_info.part_build_dir),
            "SNAPCRAFT_PART_BUILD_WORK": str(step_info.part_build_subdir),
            "SNAPCRAFT_PART_INSTALL": str(step_info.part_install_dir),
        }
    )
    return True


def _patch_elf(step_info: StepInfo) -> bool:
    """Patch rpath and interpreter in ELF files for classic mode."""
    if "enable-patchelf" not in step_info.build_attributes:
        emit.debug(f"patch_elf: not enabled for part {step_info.part_name!r}")
        return True

    if not step_info.state:
        emit.debug("patch_elf: no state information")
        return True

    try:
        # If libc is staged we'll find a dynamic linker in the payload. At
        # runtime the linker will be in the installed snap path.
        linker = elf_utils.get_dynamic_linker(
            root_path=step_info.prime_dir,
            snap_path=Path(f"/snap/{step_info.project_name}/current"),
        )
    except elf_errors.DynamicLinkerNotFound:
        # Otherwise look for the host linker, which should match the base
        # system linker. At runtime use the linker from the installed base
        # snap.
        linker = elf_utils.get_dynamic_linker(
            root_path=Path("/"), snap_path=Path(f"/snap/{step_info.base}/current")
        )

    migrated_files = step_info.state.files
    patcher = Patcher(dynamic_linker=linker, root_path=step_info.prime_dir)
    elf_files = elf_utils.get_elf_files_from_list(step_info.prime_dir, migrated_files)
    soname_cache = SonameCache()
    arch_triplet = elf_utils.get_arch_triplet()

    for elf_file in elf_files:
        elf_file.load_dependencies(
            root_path=step_info.prime_dir,
            base_path=Path(f"/snap/{step_info.base}/current"),
            content_dirs=[],  # classic snaps don't use content providers
            arch_triplet=arch_triplet,
            soname_cache=soname_cache,
        )

        relative_path = elf_file.path.relative_to(step_info.prime_dir)
        emit.progress(f"Patch ELF file: {str(relative_path)!r}")
        patcher.patch(elf_file=elf_file)

    return True


def _expand_environment(
    snapcraft_yaml: Dict[str, Any], *, parallel_build_count: int, target_arch: str
) -> None:
    """Expand global variables in the provided dictionary values.

    :param snapcraft_yaml: A dictionary containing the contents of the
        snapcraft.yaml project file.
    """
    if utils.is_managed_mode():
        work_dir = utils.get_managed_environment_home_path()
    else:
        work_dir = Path.cwd()

    project_vars = {
        "version": snapcraft_yaml.get("version", ""),
        "grade": snapcraft_yaml.get("grade", ""),
    }

    if target_arch == "all":
        target_arch = get_host_architecture()

    dirs = craft_parts.ProjectDirs(work_dir=work_dir)
    info = craft_parts.ProjectInfo(
        application_name="snapcraft",  # not used in environment expansion
        cache_dir=Path(),  # not used in environment expansion
        arch=convert_architecture_deb_to_platform(target_arch),
        parallel_build_count=parallel_build_count,
        project_name=snapcraft_yaml.get("name", ""),
        project_dirs=dirs,
        project_vars=project_vars,
    )
    _set_global_environment(info)

    craft_parts.expand_environment(snapcraft_yaml, info=info, skip=["name", "version"])


def get_build_plan(
    yaml_data: Dict[str, Any], parsed_args: "argparse.Namespace"
) -> List[Tuple[str, str]]:
    """Get a list of all build_on->build_for architectures from the project file.

    Additionally, check for the command line argument `--build-for <architecture>`
    When defined, the build plan will only contain builds where `build-for`
    matches `SNAPCRAFT_BUILD_FOR`.
    Note: `--build-for` defaults to the environmental variable `SNAPCRAFT_BUILD_FOR`.

    :param yaml_data: The project YAML data.
    :param parsed_args: snapcraft's argument namespace

    :return: List of tuples of every valid build-on->build-for combination.
    """
    archs = ArchitectureProject.unmarshal(yaml_data).architectures

    host_arch = get_host_architecture()
    build_plan: List[Tuple[str, str]] = []

    # `isinstance()` calls are for mypy type checking and should not change logic
    for arch in [arch for arch in archs if isinstance(arch, Architecture)]:
        for build_on in arch.build_on:
            if build_on in host_arch and isinstance(arch.build_for, list):
                build_plan.append((host_arch, arch.build_for[0]))
            else:
                emit.verbose(
                    f"Skipping build-on: {build_on} build-for: {arch.build_for}"
                    f" because build-on doesn't match host arch: {host_arch}"
                )

    # filter out builds not matching argument `--build_for` or env `SNAPCRAFT_BUILD_FOR`
    build_for_arg = parsed_args.build_for
    if build_for_arg is not None:
        build_plan = [build for build in build_plan if build[1] == build_for_arg]

    if len(build_plan) == 0:
        emit.message(
            "Could not make build plan:"
            " build-on architectures in snapcraft.yaml"
            f" does not match host architecture ({host_arch})."
        )
    else:
        log_output = "Created build plan:"
        for build in build_plan:
            log_output += f"\n  build-on: {build[0]} build-for: {build[1]}"
        emit.trace(log_output)

    return build_plan
