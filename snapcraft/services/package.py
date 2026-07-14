# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023-2024 Canonical Ltd.
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

"""Snapcraft Package service."""

from __future__ import annotations

import os
import pathlib
import shutil
import stat
import urllib.parse
from typing import Literal, cast

import requests
import yaml
from craft_application import PackageService
from craft_application.services.package import PackageFileEntry, package_file
from craft_application.util import strtobool
from craft_cli import emit
from typing_extensions import override

from snapcraft import const, errors, linters, models, pack
from snapcraft.errors import SnapcraftPrecreationEscapesPrimeError
from snapcraft.linters import LinterStatus
from snapcraft.meta import component_yaml, snap_yaml
from snapcraft.models import ContentPlug
from snapcraft.parts import extract_metadata as extract
from snapcraft.parts import update_metadata as update
from snapcraft.parts.desktop_file import DesktopFile
from snapcraft.parts.setup_assets import (
    ensure_hook_executable,
    find_icon_file,
    validate_command_chain,
)
from snapcraft.services import Lifecycle
from snapcraft.utils import process_version


class Package(PackageService):
    """Package service subclass for Snapcraft."""

    _REMOTE_FETCH_TIMEOUT = 120
    _HOOK_STUB = "#!/bin/true\n"

    @override
    def setup(self) -> None:
        """Application-specific service setup."""
        from snapcraft.services import (  # noqa: PLC0415 (import-outside-top-level)
            Project,
        )

        super().setup()
        self._project_service = cast(Project, self._services.get("project"))
        build_plan = self._services.get("build_plan").plan()
        # The build plan will be empty if the project can't build on the host arch.
        # This may happen in commands that don't run the lifecycle, so we have to check
        # if the build plan exists first.
        if build_plan:
            self._build_for = build_plan[0].build_for
            self._platform = build_plan[0].platform

    @property
    def _project(self) -> models.Project:
        """Get the project.

        This can be replaced with an upstream helper after
        https://github.com/canonical/craft-application/issues/878 is completed.
        """
        return cast(models.Project, self._project_service.get())

    @override
    def _extra_project_updates(self) -> bool:
        # Update the project from parse-info data.
        project_info = self._services.lifecycle.project_info
        project_before = self._project.marshal()
        extracted_metadata = extract.extract_lifecycle_metadata(
            self._project.adopt_info,
            self._project_service.get_parse_info(),
            project_info.work_dir,
            partitions=project_info.partitions,
        )
        update.update_from_extracted_metadata(
            self._project,
            metadata_list=extracted_metadata,
            assets_dir=self._get_assets_dir(),
            prime_dir=project_info.prime_dir,
        )
        project_changed = self._project.marshal() != project_before
        prime_changed = False

        # precreate content targets and layout sources when using core26+
        # and bare base snaps
        if self._project.get_effective_base() not in ("core22", "core24"):
            layout_changed = self._precreate_layout_targets()
            plug_changed = self._precreate_plug_targets()
            prime_changed = layout_changed or plug_changed

        return project_changed or prime_changed

    def _precreate_layout_targets(self) -> bool:
        """Create layout targets ahead of time for snapd to avoid ENOENT errors."""
        if self._project.layout is None:
            return False

        emit.debug("Pre-creating layout targets inside of snap")
        changed = False

        for src, layout in self._project.layout.items():
            result = self._parse_layout_target(src, layout)
            if result is None:
                continue
            path, ltype = result

            file = self._maybe_get_target_in_snap(path)
            if file is None:
                continue

            to_create = self._concat_with_prime_dir(file)

            match ltype:
                case "bind" | "tmpfs":
                    emit.debug(
                        f"Layout target directory {path!r} maps to {str(file)!r} inside of the snap"
                    )
                    emit.debug(f"Creating {str(file)!r} in the prime directory")
                    changed = changed or not to_create.exists()
                    to_create.mkdir(0o0755, parents=True, exist_ok=True)
                case "bind-file":
                    emit.debug(
                        f"Layout target file {path!r} maps to {str(file)!r} inside of the snap"
                    )
                    emit.debug(f"Creating {str(file)!r} in the prime directory")
                    to_create.parent.mkdir(0o0755, parents=True, exist_ok=True)
                    changed = changed or not to_create.exists()
                    to_create.touch(0o0644)

        return changed

    def _precreate_plug_targets(self) -> bool:
        """Create plug targets ahead of time for snapd to avoid ENOENT errors."""
        if self._project.plugs is None:
            return False

        emit.debug("Pre-creating plug targets inside of snap")
        changed = False

        plug_targets = []
        for name, plug in self._project.plugs.items():
            if isinstance(plug, ContentPlug) and plug.interface == "content":
                plug_targets.append(plug.target)
            elif name == "content":
                plug_targets.append(cast("dict[str, str]", plug)["target"])

        for target in plug_targets:
            file = self._maybe_get_target_in_snap(target)
            if file is None:
                continue

            to_create = self._concat_with_prime_dir(file)

            emit.debug(
                f"Plug target directory {target!r} maps to {str(file)!r} inside of the snap"
            )
            emit.debug(f"Creating {str(file)!r} in the prime directory")
            changed = changed or not to_create.exists()
            to_create.mkdir(0o0755, parents=True, exist_ok=True)

        return changed

    @staticmethod
    def _parse_layout_target(
        src: str, layout: dict[Literal["symlink", "bind", "bind-file", "type"], str]
    ) -> tuple[str, Literal["bind", "bind-file", "tmpfs"]] | None:
        """Extracts the layout type and its str-path.

        Below is an example of possible layout formats and what this method will
        do with each.

        layouts:
            # Returns (<bind-mount sourcedir>, "bind")
            <path>:
                bind: <bind-mount sourcedir>

            # Returns (<bind-mount sourcefile>, "bind-file")
            <path>:
                bind-file: <bind-mount sourcefile>

            # Returns (<path>, "tmpfs")
            <path>:
                type: tmpfs

            # Returns None
            <path>:
                symlink: <link-target>

        This method will also return None for malformed layouts, though these should
        always be caught by the project model validation before getting here.

        :returns: A tuple of (str-path, layout-type). The returned path is neither sanitized
            nor validated.
        """
        key, value = next(iter(layout.items()))

        match key:
            case "type":
                if value == "tmpfs":
                    return (src, "tmpfs")
                return None
            case "bind" | "bind-file":
                return (value, key)
            case _:
                return None

    @staticmethod
    def _maybe_get_target_in_snap(path: str) -> pathlib.Path | None:
        """Determine the path to create inside of a snap, if any, based on a layout or plug target.

        Paths beginning with "/" or "$SNAP" will end up in the final artifact, as will relative
        paths that do not begin with a variable (e.g. "foo", but not "$SNAP_DATA/foo").

        If the path is only "/" or "$SNAP", this is just the snap root and should always
        exist (no-op).

        :param path: String path to investigate
        :returns: A Path object that needs to be created, or None if nothing needs to be done.
        """
        # Make explicit references to the snap root ($SNAP) relative
        if path.startswith("$SNAP/"):
            path = path.removeprefix("$SNAP/")

        # Beginning with "/" is equivalent to beginning with "$SNAP". So likewise, make these relative too
        while path.startswith("/"):
            path = path.removeprefix("/")

        # At this point, if the string is empty, it was just a reference to the snap root. This should
        # always exist, so we can return early
        if not path:
            return None

        # If any $s remain, this is a special path that shouldn't be handled
        if "$" in path:
            return None

        return pathlib.Path(path)

    def _concat_with_prime_dir(self, path: pathlib.Path) -> pathlib.Path:
        """Concatenate a path onto the prime directory.

        :returns: Concatenated path
        :raises SnapcraftPrecreationEscapesPrimeError: When the resulting path is no longer relative
            to the prime directory."""

        prime_dir = self._services.lifecycle.prime_dir

        result = prime_dir / path
        if not result.resolve().is_relative_to(prime_dir):
            raise SnapcraftPrecreationEscapesPrimeError(path)

        return result

    def _pack_components(self, dest: pathlib.Path) -> dict[str, pathlib.Path]:
        component_map: dict[str, pathlib.Path] = {}

        if self._project.components is not None:
            for component_name, component in self._project.components.items():
                if compression := component.compression:
                    emit.debug(
                        f"Using {compression!r} compression for {component_name!r}."
                    )
                else:
                    compression = self._project.compression
                    emit.debug(
                        f"Using the snap's {compression!r} compression for {component_name!r}."
                    )

                filename = pack.pack_component(
                    cast(Lifecycle, self._services.lifecycle).get_prime_dir(
                        component_name
                    ),
                    compression=compression,
                    output_dir=dest,
                )
                component_map[component_name] = pathlib.Path(filename)

        return component_map

    def _get_pack_output(self) -> pathlib.Path:
        """Return the configured pack output path."""
        return self.output_dir

    def _get_artifact_output_dir(self) -> pathlib.Path:
        """Return the directory where component artifacts will be written."""
        output = self._get_pack_output()
        if output and not output.is_dir():
            return output.parent.resolve()

        return output.resolve()

    def _get_default_artifact_path(self) -> pathlib.Path:
        """Return the expected output path for the snap artifact."""
        output = self._get_pack_output()
        output_dir = self._get_artifact_output_dir()
        filename = pack._get_filename(  # noqa: SLF001
            str(output) if output else None,
            self._project.name,
            process_version(self._project.version),
            self._platform,
        )
        if filename is None:
            raise errors.SnapcraftError("Could not determine the snap artifact name.")

        return output_dir / filename

    def _get_component_artifact_path(self, component_name: str) -> pathlib.Path:
        """Return the expected output path for a component artifact."""
        if self._project.components is None:
            raise errors.SnapcraftError("Project does not contain any components.")

        component = self._project.components[component_name]
        version = process_version(component.version or self._project.version)
        filename = f"{self._project.name}+{component_name}_{version}.comp"
        return self._get_artifact_output_dir() / filename

    @override
    def _prime_dir_for(self, partition_name: str | None) -> pathlib.Path:
        """Return the prime directory for the default snap or a component artifact."""
        if partition_name in (None, "default"):
            return self._services.lifecycle.prime_dir

        return cast(Lifecycle, self._services.lifecycle).get_prime_dir(partition_name)

    @override
    def get_artifacts(self) -> dict[str | None, pathlib.Path]:
        """Get the expected output artifacts for the current pack operation."""
        artifacts: dict[str | None, pathlib.Path] = {
            None: self._get_default_artifact_path()
        }

        for component_name in self._project.get_component_names():
            artifacts[component_name] = self._get_component_artifact_path(
                component_name
            )

        return artifacts

    @override
    def _pack(self, *, name: str | None = None, path: pathlib.Path) -> None:
        """Pack a specific snap or component artifact."""
        self._ensure_hook_assets_executable(name)

        if name in (None, "default"):
            issues = linters.run_linters(
                self._services.lifecycle.prime_dir, lint=self._project.lint
            )
            status = linters.report(issues, intermediate=True)

            # In case of linter errors, stop execution and return the error code.
            if status in (LinterStatus.ERRORS, LinterStatus.FATAL):
                raise errors.LinterError("Linter errors found", exit_code=status)

            pack.pack_snap(
                self._services.lifecycle.prime_dir,
                output=str(path),
                compression=self._project.compression,
                name=self._project.name,
                version=process_version(self._project.version),
                target=self._platform,
            )
            return

        if self._project.components is None or name not in self._project.components:
            raise errors.SnapcraftError(f"Unknown component artifact {name!r}.")

        component = self._project.components[name]
        compression = component.compression or self._project.compression
        if component.compression:
            emit.debug(f"Using {component.compression!r} compression for {name!r}.")
        else:
            emit.debug(
                f"Using the snap's {self._project.compression!r} compression for {name!r}."
            )

        filename = pack.pack_component(
            cast(Lifecycle, self._services.lifecycle).get_prime_dir(name),
            compression=compression,
            output_dir=path.parent,
        )
        if filename != path.name:
            raise errors.SnapcraftError(
                f"Packed component {name!r} to unexpected file {filename!r}."
            )

    @override
    def pack(self, prime_dir: pathlib.Path, dest: pathlib.Path) -> list[pathlib.Path]:
        """Create one or more packages as appropriate.

        :param prime_dir: Path to the directory to pack.
        :param dest: Directory into which to write the package(s).
        :returns: A list of paths to created packages.
        """
        issues = linters.run_linters(prime_dir, lint=self._project.lint)
        status = linters.report(issues, intermediate=True)

        # In case of linter errors, stop execution and return the error code.
        if status in (LinterStatus.ERRORS, LinterStatus.FATAL):
            raise errors.LinterError("Linter errors found", exit_code=status)

        snap_file = pathlib.Path(
            pack.pack_snap(
                prime_dir,
                output=str(dest),
                compression=self._project.compression,
                name=self._project.name,
                version=process_version(self._project.version),
                target=self._platform,
            )
        )
        component_map = self._pack_components(dest)
        self._resource_map = component_map

        return [snap_file] + list(component_map.values())

    def _get_assets_dir(self) -> pathlib.Path:
        """Return a snapcraft assets directory.

        Asset directories can exist in:

        - <PROJECT_ROOT>/snap
        - <PROJECT_ROOT>/build-aux/snap
        """
        project_dir = self._services.lifecycle.project_info.project_dir
        for asset_reldir in ("snap", "build-aux/snap"):
            asset_dir = project_dir / asset_reldir
            if asset_dir.exists():
                return asset_dir

        # This is for backwards compatibility with setup_assets(...)
        return project_dir / "snap"

    @package_file("meta/snap.yaml", partition_re="default")
    def _get_snap_yaml(self, partition: str | None = None) -> str:
        """Generate the snap.yaml file contents for the default partition."""
        if partition not in (None, "default"):
            raise errors.SnapcraftError(
                f"Cannot generate snap metadata for partition {partition!r}."
            )

        return self.metadata.to_yaml_string()

    @package_file("snap/manifest.yaml", partition_re="default")
    def _get_manifest_yaml(self, partition: str | None = None) -> str | None:
        """Generate the manifest.yaml file contents for the default partition."""
        if partition not in (None, "default"):
            raise errors.SnapcraftError(
                f"Cannot generate snap manifest for partition {partition!r}."
            )

        if not strtobool(os.getenv("SNAPCRAFT_BUILD_INFO", "n")):
            return None

        lifecycle_service = cast(Lifecycle, self._services.lifecycle)
        return lifecycle_service.generate_manifest().to_yaml_string()

    @package_file("meta/component.yaml", partition_re=r"(?!default$).+")
    def _get_component_yaml(self, partition: str | None = None) -> str:
        """Generate component.yaml contents for a component partition."""
        if partition is None or partition == "default":
            raise errors.SnapcraftError(
                f"Cannot generate component metadata for partition {partition!r}."
            )

        return component_yaml.get_metadata(self._project, partition).to_yaml_string()

    @override
    def _gen_extra_assets(
        self, partition_name: str | None = None
    ) -> list[tuple[str | bytes | None | pathlib.Path, pathlib.Path]]:
        """Generate mediated post-prime assets for a partition.

        A ``(None, destination)`` entry is returned when the asset should not
        exist in prime; craft-application will delete any stale copy at that
        destination.
        """
        assets = [
            *self._get_hook_assets(partition_name),
            *self._get_gui_assets(partition_name),
            *self._get_desktop_assets(partition_name),
            *self._get_icon_assets(partition_name),
        ]

        if partition_name in (None, "default"):
            project_file = self._services.get("project").resolve_project_file_path()
            destination = (
                self._prime_dir_for(partition_name) / "snap" / project_file.name
            )
            source: pathlib.Path | None = (
                project_file if self._project_file_copy_enabled() else None
            )
            assets.insert(0, (source, destination))
            assets.extend(self._get_system_metadata_assets())

        return assets

    def _get_desktop_assets(
        self, partition_name: str | None = None
    ) -> list[tuple[str | None, pathlib.Path]]:
        """Generate rewritten desktop files for a partition."""
        if partition_name not in (None, "default") or not self._project.apps:
            return []

        prime_dir = self._prime_dir_for(partition_name)
        icon_path = self._get_effective_icon_path(partition_name)
        assets: list[tuple[str | None, pathlib.Path]] = []

        for app_name, app in self._project.apps.items():
            if not app.desktop:
                continue

            desktop_file = DesktopFile(
                snap_name=self._project.name,
                app_name=app_name,
                filename=app.desktop,
                prime_dir=prime_dir,
            )
            destination = prime_dir / "meta" / "gui" / f"{app_name}.desktop"
            assets.append(
                (
                    desktop_file.render(icon_path=icon_path),
                    destination,
                )
            )

        return assets

    def _get_icon_assets(
        self, partition_name: str | None = None
    ) -> list[tuple[bytes | pathlib.Path | None, pathlib.Path]]:
        """Generate icon assets for the default partition."""
        if partition_name not in (None, "default"):
            return []

        icon_asset = self._resolve_icon_asset(partition_name)
        if icon_asset is None:
            return []

        source, destination = icon_asset
        return [(source, destination)]

    def _get_system_metadata_assets(
        self,
    ) -> list[tuple[pathlib.Path | None, pathlib.Path]]:
        """Generate gadget/kernel metadata assets for the default partition."""
        prime_dir = self._prime_dir_for(None)
        project_dir = self._services.lifecycle.project_info.project_dir
        assets: list[tuple[pathlib.Path | None, pathlib.Path]] = []

        if self._project.type == const.ProjectType.GADGET:
            gadget_yaml = project_dir / "gadget.yaml"
            if not gadget_yaml.exists():
                raise errors.SnapcraftError("gadget.yaml is required for gadget snaps")
            assets.append((gadget_yaml, prime_dir / "meta" / "gadget.yaml"))

        if self._project.type == const.ProjectType.KERNEL:
            kernel_yaml = project_dir / "kernel.yaml"
            assets.append(
                (
                    kernel_yaml if kernel_yaml.exists() else None,
                    prime_dir / "meta" / "kernel.yaml",
                )
            )

        return assets

    def _get_hook_assets(
        self, partition_name: str | None = None
    ) -> list[tuple[str | pathlib.Path, pathlib.Path]]:
        """Generate hook assets for the default or component partition.

        Project-provided hooks override built hooks with the same filename.
        """
        assets: list[tuple[str | pathlib.Path, pathlib.Path]] = []
        assets.extend(
            self._get_project_assets(
                partition_name,
                source_subdir="hooks",
                destination_subdir="meta/hooks",
                include_built_subdir="snap/hooks",
            )
        )

        existing_hooks = {destination.name for _source, destination in assets}
        prime_dir = self._prime_dir_for(partition_name)
        for hook_name in self._get_declared_hooks(partition_name):
            if hook_name not in existing_hooks:
                assets.append(
                    (self._HOOK_STUB, prime_dir / "meta" / "hooks" / hook_name)
                )

        return assets

    def _get_gui_assets(
        self, partition_name: str | None = None
    ) -> list[tuple[pathlib.Path, pathlib.Path]]:
        """Generate static gui assets for the default or component partition."""
        return self._get_project_assets(
            partition_name,
            source_subdir="gui",
            destination_subdir="meta/gui",
        )

    def _get_project_assets(
        self,
        partition_name: str | None,
        *,
        source_subdir: str,
        destination_subdir: str,
        include_built_subdir: str | None = None,
    ) -> list[tuple[pathlib.Path, pathlib.Path]]:
        """Generate mediated assets copied from project or prime subdirectories."""
        prime_dir = self._prime_dir_for(partition_name)
        assets_dir = self._get_partition_assets_dir(partition_name)
        destination_dir = prime_dir / destination_subdir
        assets: list[tuple[pathlib.Path, pathlib.Path]] = []

        if include_built_subdir is not None:
            built_dir = prime_dir / include_built_subdir
            if built_dir.is_dir():
                overridden_assets = set[str]()
                project_dir = assets_dir / source_subdir
                if project_dir.is_dir():
                    overridden_assets = {asset.name for asset in project_dir.iterdir()}

                for asset in sorted(built_dir.iterdir()):
                    if asset.name in overridden_assets:
                        continue
                    assets.append((asset, destination_dir / asset.name))

        project_dir = assets_dir / source_subdir
        if project_dir.is_dir():
            for asset in sorted(project_dir.iterdir()):
                assets.append((asset, destination_dir / asset.name))

        return assets

    def _resolve_icon_asset(  # noqa: PLR0911
        self, partition_name: str | None = None
    ) -> tuple[bytes | pathlib.Path, pathlib.Path] | None:
        """Resolve the icon asset that should be added to prime."""
        prime_dir = self._prime_dir_for(partition_name)
        assets_dir = self._get_partition_assets_dir(partition_name)

        if partition_name not in (None, "default"):
            return None

        icon = self._project.icon

        if icon is None:
            icon_path = find_icon_file(assets_dir)
            if icon_path is None:
                return None
            return (icon_path, prime_dir / icon_path.relative_to(assets_dir.parent))

        parsed_url = urllib.parse.urlparse(icon)
        parsed_path = pathlib.Path(parsed_url.path)
        icon_ext = parsed_path.suffix[1:]
        target_icon_path = prime_dir / "meta" / "gui" / f"icon.{icon_ext}"

        if parsed_url.scheme in ["http", "https"]:
            emit.progress(f"Fetching icon from {icon!r}")
            try:
                response = requests.get(icon, timeout=self._REMOTE_FETCH_TIMEOUT)
                response.raise_for_status()
            except requests.RequestException as err:
                raise errors.SnapcraftError(
                    f"Failed to fetch icon from {icon!r}.", details=str(err)
                ) from err

            icon_data = response.content
            return (icon_data, target_icon_path)

        if parsed_url.scheme:
            raise RuntimeError(f"Unexpected icon path: {parsed_url!r}")

        source_path = prime_dir / (
            parsed_path.relative_to("/") if parsed_path.is_absolute() else parsed_path
        )
        if source_path.exists():
            return (source_path, target_icon_path)
        if parsed_path.exists():
            return (parsed_path, target_icon_path)

        icon_path = find_icon_file(assets_dir)
        if icon_path is None:
            return None

        return (icon_path, prime_dir / icon_path.relative_to(assets_dir.parent))

    def _get_effective_icon_path(self, partition_name: str | None = None) -> str | None:
        """Return the path desktop rewrites should reference for the icon."""
        icon_asset = self._resolve_icon_asset(partition_name)
        if icon_asset is None:
            return None

        _source, destination = icon_asset
        return str(destination.relative_to(self._prime_dir_for(partition_name)))

    def _get_partition_assets_dir(
        self, partition_name: str | None = None
    ) -> pathlib.Path:
        """Return the project assets directory for a default or component partition."""
        assets_dir = self._get_assets_dir()
        if partition_name in (None, "default"):
            return assets_dir

        return assets_dir / "component" / partition_name

    def _get_declared_hooks(
        self, partition_name: str | None = None
    ) -> dict[str, models.Hook]:
        """Return hook declarations for the given partition."""
        if partition_name in (None, "default"):
            return self._project.hooks or {}

        if self._project.components is None:
            return {}

        component = self._project.components.get(partition_name)
        if component is None:
            return {}

        return component.hooks or {}

    def _validate_declared_hook_command_chains(
        self, partition_name: str | None = None
    ) -> None:
        """Validate command-chain entries for declared hooks."""
        prime_dir = self._prime_dir_for(partition_name)

        for hook_name, hook in self._get_declared_hooks(partition_name).items():
            if hook.command_chain:
                validate_command_chain(
                    hook.command_chain,
                    name=f"hook {hook_name!r}",
                    prime_dir=prime_dir,
                )

    @staticmethod
    def _project_file_copy_enabled() -> bool:
        """Return whether the project file should be copied into snap/."""
        return bool(strtobool(os.getenv("SNAPCRAFT_BUILD_INFO", "n")))

    @override
    def _write_asset(
        self, source: str | bytes | None | pathlib.Path, destination: pathlib.Path
    ) -> None:
        """Write a generated package file or extra asset into prime."""
        if source is None:
            super()._write_asset(source, destination)
            return

        if isinstance(source, pathlib.Path):
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(source, destination)
        else:
            super()._write_asset(source, destination)

        if self._is_hook_asset(destination):
            ensure_hook_executable(destination)

    @staticmethod
    def _is_hook_asset(path: pathlib.Path) -> bool:
        """Return whether a path points to a mediated hook file."""
        return path.parent.name == "hooks" and path.parent.parent.name == "meta"

    @override
    def _package_file_changed(
        self, package_file: PackageFileEntry, partition_name: str | None
    ) -> bool:
        """Return whether a generated package file differs from prime contents."""
        if package_file.relative_path == pathlib.PurePosixPath("snap/manifest.yaml"):
            # craft-application normally compares package files byte-for-byte, but
            # manifest.yaml embeds snapcraft-started-at, which changes on every run.
            return self._manifest_changed(partition_name)

        return super()._package_file_changed(package_file, partition_name)

    def _manifest_changed(self, partition_name: str | None) -> bool:
        """Return whether the mediated manifest differs from the primed one.

        Ignores ``snapcraft-started-at`` because it changes on every Snapcraft
        invocation. Other fields that can drift with the build environment
        (``snapcraft-version``, OS release IDs, ``image-info``, build/stage
        package lists) are intentionally considered meaningful content changes
        that should trigger a repack.
        """
        content = self._get_manifest_yaml(partition_name)
        destination = self._prime_dir_for(partition_name) / "snap" / "manifest.yaml"

        if content is None:
            return destination.exists()

        if not destination.is_file():
            return True

        existing = yaml.safe_load(destination.read_text(encoding="utf-8"))
        generated = yaml.safe_load(content)

        if isinstance(existing, dict):
            existing.pop("snapcraft-started-at", None)
        if isinstance(generated, dict):
            generated.pop("snapcraft-started-at", None)

        return existing != generated

    @override
    def write_metadata(self, path: pathlib.Path) -> None:
        """Write mediated package metadata and assets into the given prime directory.

        This includes generated package files and mediated assets such as hooks,
        desktop files, icons, and other metadata content.

        :param path: The prime directory to update.
        """
        path.mkdir(parents=True, exist_ok=True)

        self._write_metadata_assets_changed()

    def _write_metadata_assets_changed(self) -> bool:
        """Create mediated assets during post-prime setup."""
        changed = False

        for partition_name in self.get_artifacts():
            self._validate_declared_hook_command_chains(partition_name)

            for pkg_file in self._package_files(partition_name):
                if self._package_file_changed(pkg_file, partition_name):
                    changed = True
                    generator = getattr(self, pkg_file.method_name)
                    self._write_asset(
                        generator(partition_name),
                        self._prime_dir_for(partition_name) / pkg_file.relative_path,
                    )

            for source, destination in self._gen_extra_assets(partition_name):
                if self._asset_changed(source, destination, partition_name):
                    changed = True
                    self._write_asset(source, destination)

            if self._ensure_hook_assets_executable(partition_name):
                changed = True

        self._project_was_updated = self._project_was_updated or changed
        return changed

    def _ensure_hook_assets_executable(self, partition_name: str | None = None) -> bool:
        """Ensure mediated hooks under meta/hooks are executable."""
        hooks_dir = self._prime_dir_for(partition_name) / "meta" / "hooks"
        if not hooks_dir.is_dir():
            return False

        changed = False
        for hook_path in hooks_dir.iterdir():
            if not hook_path.is_file():
                continue
            if not hook_path.stat().st_mode & stat.S_IEXEC:
                ensure_hook_executable(hook_path)
                changed = True

        return changed

    @property
    def metadata(self) -> snap_yaml.SnapMetadata:
        """Get the metadata model for this project."""
        return snap_yaml.get_metadata_from_project(
            self._project,
            self._services.lifecycle.prime_dir,
            arch=self._build_for,
        )


def _hardlink_or_copy(source: pathlib.Path, destination: pathlib.Path) -> bool:
    """Try to hardlink and fallback to copy if it fails.

    :param source: the source path.
    :param destination: the destination path.
    :returns: True if a hardlink was done or False for copy.
    """
    # Unlink the destination to avoid link failures
    destination.unlink(missing_ok=True)

    try:
        destination.hardlink_to(source)
    except OSError as os_error:
        # Cross device link
        if os_error.errno != 18:
            raise
        shutil.copy(source, destination)
        return False

    return True
