# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Craft-parts lifecycle wrapper."""

import pathlib
import subprocess
from typing import Any, Dict, List, Optional

import craft_parts
from craft_cli import emit
from craft_parts import ActionType, Part, ProjectDirs, Step
from craft_parts.packages import Repository, deb
from xdg import BaseDirectory  # type: ignore

from snapcraft import errors, repo
from snapcraft.meta import ExtractedMetadata, extract_metadata

_LIFECYCLE_STEPS = {
    "pull": Step.PULL,
    "overlay": Step.OVERLAY,
    "build": Step.BUILD,
    "stage": Step.STAGE,
    "prime": Step.PRIME,
}


class PartsLifecycle:
    """Create and manage the parts lifecycle.

    :param all_parts: A dictionary containing the parts defined in the project.
    :param work_dir: The working directory for parts processing.
    :param assets_dir: The directory containing project assets.
    :param adopt_info: The name of the part containing metadata do adopt.
    :param extra_build_snaps: A list of additional build snaps to install.

    :raises PartsLifecycleError: On error initializing the parts lifecycle.
    """

    def __init__(
        self,
        all_parts: Dict[str, Any],
        *,
        work_dir: pathlib.Path,
        assets_dir: pathlib.Path,
        base: str,
        package_repositories: List[Dict[str, Any]],
        parallel_build_count: int,
        part_names: Optional[List[str]],
        adopt_info: Optional[str],
        parse_info: Dict[str, List[str]],
        project_name: str,
        project_vars: Dict[str, str],
        extra_build_snaps: Optional[List[str]] = None,
    ):
        self._work_dir = work_dir
        self._assets_dir = assets_dir
        self._package_repositories = package_repositories
        self._part_names = part_names
        self._adopt_info = adopt_info
        self._parse_info = parse_info

        emit.progress("Initializing parts lifecycle")

        # set the cache dir for parts package management
        cache_dir = BaseDirectory.save_cache_path("snapcraft")

        try:
            self._lcm = craft_parts.LifecycleManager(
                {"parts": all_parts},
                application_name="snapcraft",
                work_dir=work_dir,
                cache_dir=cache_dir,
                base=base,
                ignore_local_sources=["*.snap"],
                extra_build_snaps=extra_build_snaps,
                parallel_build_count=parallel_build_count,
                project_name=project_name,
                project_vars_part_name=adopt_info,
                project_vars=project_vars,
            )
        except craft_parts.PartsError as err:
            raise errors.PartsLifecycleError(str(err)) from err

    @property
    def prime_dir(self) -> pathlib.Path:
        """Return the parts prime directory path."""
        return self._lcm.project_info.prime_dir

    @property
    def target_arch(self) -> str:
        """Return the parts project target architecture."""
        return self._lcm.project_info.target_arch

    @property
    def target_arch_triplet(self) -> str:
        """Return the parts project target architecture."""
        return self._lcm.project_info.arch_triplet

    @property
    def project_vars(self) -> Dict[str, str]:
        """Return the value of project variable ``version``."""
        return {
            "version": self._lcm.project_info.get_project_var("version"),
            "grade": self._lcm.project_info.get_project_var("grade"),
        }

    def run(
        self,
        step_name: str,
        *,
        debug: bool = False,
        shell: bool = False,
        shell_after: bool = False,
    ) -> None:
        """Run the parts lifecycle.

        :param target_step: The final step to execute.

        :raises PartsLifecycleError: On error during lifecycle.
        :raises RuntimeError: On unexpected error.
        """
        target_step = _LIFECYCLE_STEPS.get(step_name)
        if not target_step:
            raise RuntimeError(f"Invalid target step {step_name!r}")

        if shell:
            # convert shell to shell_after for the previous step
            previous_steps = target_step.previous_steps()
            target_step = previous_steps[-1] if previous_steps else None
            shell_after = True

        try:
            if target_step:
                actions = self._lcm.plan(target_step, part_names=self._part_names)
            else:
                actions = []

            self._install_package_repositories()

            emit.progress("Executing parts lifecycle...")

            with self._lcm.action_executor() as aex:
                for action in actions:
                    message = _action_message(action)
                    emit.progress(f"Executing parts lifecycle: {message}")
                    with emit.open_stream("Executing action") as stream:
                        aex.execute(action, stdout=stream, stderr=stream)
                    emit.message(f"Executed: {message}", intermediate=True)

            if shell_after:
                _launch_shell()

            emit.message("Executed parts lifecycle", intermediate=True)
        except RuntimeError as err:
            raise RuntimeError(f"Parts processing internal error: {err}") from err
        except OSError as err:
            if debug:
                _launch_shell()
            msg = err.strerror
            if err.filename:
                msg = f"{err.filename}: {msg}"
            raise errors.PartsLifecycleError(msg) from err
        except Exception as err:
            if debug:
                _launch_shell()
            raise errors.PartsLifecycleError(str(err)) from err

    def _install_package_repositories(self) -> None:
        if not self._package_repositories:
            return

        # Install pre-requisite packages for apt-key, if not installed.
        required_packages = ["gnupg", "dirmngr"]
        if any(p for p in required_packages if not Repository.is_package_installed(p)):
            Repository.install_packages(required_packages, refresh_package_cache=True)

        emit.progress("Installing package repositories...")
        refresh_required = repo.install(
            self._package_repositories, key_assets=self._assets_dir / "keys"
        )
        if refresh_required:
            emit.progress("Refreshing package repositories...")
            # TODO: craft-parts API for: force_refresh=refresh_required
            deb.Ubuntu.refresh_packages_list.cache_clear()
            self._lcm.refresh_packages_list()
        emit.message("Installed package repositories", intermediate=True)

    def clean(self, *, part_names: Optional[List[str]] = None) -> None:
        """Remove lifecycle artifacts.

        :param part_names: The names of the parts to clean. If not
            specified, all parts will be cleaned.
        """
        if part_names:
            message = "Cleaning parts: " + ", ".join(part_names)
        else:
            message = "Cleaning all parts"

        emit.message(message, intermediate=True)
        self._lcm.clean(part_names=part_names)

    def extract_metadata(self) -> List[ExtractedMetadata]:
        """Obtain metadata information."""
        if self._adopt_info is None or self._adopt_info not in self._parse_info:
            return []

        dirs = ProjectDirs(work_dir=self._work_dir)
        part = Part(self._adopt_info, {}, project_dirs=dirs)
        locations = (
            part.part_src_dir,
            part.part_build_dir,
            part.part_install_dir,
        )
        metadata_list: List[ExtractedMetadata] = []

        for metadata_file in self._parse_info[self._adopt_info]:
            emit.trace(f"extract metadata: parse info from {metadata_file}")

            for location in locations:
                if pathlib.Path(location, metadata_file.lstrip("/")).is_file():
                    metadata = extract_metadata(metadata_file, workdir=str(location))
                    if metadata:
                        metadata_list.append(metadata)
                        break

                    emit.message(
                        f"No metadata extracted from {metadata_file}", intermediate=True
                    )

        return metadata_list


def _launch_shell(*, cwd: Optional[pathlib.Path] = None) -> None:
    """Launch a user shell for debugging environment.

    :param cwd: Working directory to start user in.
    """
    emit.message("Launching shell on build environment...", intermediate=True)
    with emit.pause():
        subprocess.run(["bash"], check=False, cwd=cwd)


def _action_message(action: craft_parts.Action) -> str:
    msg = {
        Step.PULL: {
            ActionType.RUN: "pull",
            ActionType.RERUN: "repull",
            ActionType.SKIP: "skip pull",
            ActionType.UPDATE: "update sources for",
        },
        Step.OVERLAY: {
            ActionType.RUN: "overlay",
            ActionType.RERUN: "re-overlay",
            ActionType.SKIP: "skip overlay",
            ActionType.UPDATE: "update overlay for",
            ActionType.REAPPLY: "reapply",
        },
        Step.BUILD: {
            ActionType.RUN: "build",
            ActionType.RERUN: "rebuild",
            ActionType.SKIP: "skip build",
            ActionType.UPDATE: "update build for",
        },
        Step.STAGE: {
            ActionType.RUN: "stage",
            ActionType.RERUN: "restage",
            ActionType.SKIP: "skip stage",
        },
        Step.PRIME: {
            ActionType.RUN: "prime",
            ActionType.RERUN: "re-prime",
            ActionType.SKIP: "skip prime",
        },
    }

    message = f"{msg[action.step][action.action_type]} {action.part_name}"

    if action.reason:
        message += f" ({action.reason})"

    return message
