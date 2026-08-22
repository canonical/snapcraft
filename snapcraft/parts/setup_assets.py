# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Copy assets to their final locations."""

import itertools
import os
import shutil
import stat
import urllib.parse
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import requests
from craft_cli import emit

from snapcraft import const, errors, models

from .desktop_file import DesktopFile


@dataclass(frozen=True)
class MediatedIconAsset:
    """A resolved icon for mediated packaging."""

    source: Path | bytes | None
    destination: Path
    icon_path: str


def _uses_legacy_system_metadata(project: models.Project) -> bool:
    """Return whether gadget/kernel metadata should follow the core22 path."""
    return project.get_effective_base() == "core22"


def setup_assets(
    project: models.Project,
    *,
    assets_dir: Path,
    project_dir: Path,
    prime_dirs: dict[str | None, Path],
    meta_directory_handler: Callable[[Path, Path], None] | None = None,
    copy_hooks_and_gui: bool = True,
) -> None:
    """Copy assets to the appropriate locations in the snap filesystem.

    :param project: The snap project file.
    :param assets_dir: The directory containing snap project assets.
    :param project_dir: The project root directory.
    :param prime_dirs: A mapping of component names to prime directories. 'None' should
        map to the default prime directory.
    :param meta_directory_handler: callback that overrides default behavior for
        handling hooks and gui. The arguments passed are assets_dir and prime_dir.
    :param copy_hooks_and_gui: Whether to copy hooks and gui assets. When False, the
        application is expected to handle hooks and gui assets through the mediated
        packing flow.
    """
    prime_dir = prime_dirs[None]
    meta_dir = prime_dir / "meta"
    gui_dir = meta_dir / "gui"
    gui_dir.mkdir(parents=True, exist_ok=True)

    if copy_hooks_and_gui:
        copy_assets(assets_dir, prime_dir, meta_directory_handler)
    setup_hooks(project.hooks, prime_dir)

    if project.components:
        for component_name, component in project.components.items():
            if copy_hooks_and_gui:
                copy_assets(
                    assets_dir / "component" / component_name,
                    prime_dirs[component_name],
                    meta_directory_handler,
                )
            setup_hooks(component.hooks, prime_dirs[component_name])

    if _uses_legacy_system_metadata(project):
        if project.type == const.ProjectType.GADGET:
            gadget_yaml = project_dir / "gadget.yaml"
            if not gadget_yaml.exists():
                raise errors.SnapcraftError("gadget.yaml is required for gadget snaps")
            _copy_file(gadget_yaml, meta_dir / "gadget.yaml")
        elif project.type == const.ProjectType.KERNEL:
            kernel_yaml = project_dir / "kernel.yaml"
            if kernel_yaml.exists():
                _copy_file(kernel_yaml, meta_dir / "kernel.yaml")

    if not copy_hooks_and_gui:
        return

    icon_path = _finalize_icon(
        project.icon, assets_dir=assets_dir, gui_dir=gui_dir, prime_dir=prime_dir
    )
    relative_icon_path: str | None = None

    if icon_path is not None:
        if prime_dir in icon_path.parents:
            icon_path = icon_path.relative_to(prime_dir)
        relative_icon_path = str(icon_path)

    emit.debug(f"relative icon path: {relative_icon_path!r}")

    if project.apps:
        for app_name, app in project.apps.items():
            _validate_command_chain(
                app.command_chain, name=f"app {app_name!r}", prime_dir=prime_dir
            )

            if app.desktop:
                desktop_file = DesktopFile(
                    snap_name=project.name,
                    app_name=app_name,
                    filename=app.desktop,
                    prime_dir=prime_dir,
                )
                desktop_file.write(gui_dir=gui_dir, icon_path=relative_icon_path)


def get_mediated_icon_asset(
    project: models.Project, *, assets_dir: Path, prime_dir: Path
) -> MediatedIconAsset | None:
    """Resolve the mediated icon for the project, fetching remote icons once.

    The result is intended to be cached at prime time and reused for every
    pack-time ``needs_packing``/materialization call so remote icons are not
    re-fetched and network failures do not surface during repack checks.
    """
    return _get_mediated_icon_asset(
        project.icon, assets_dir=assets_dir, prime_dir=prime_dir
    )


def get_mediated_gui_assets(
    project: models.Project,
    *,
    assets_dir: Path,
    prime_dir: Path,
    icon: MediatedIconAsset | None,
) -> list[tuple[str | bytes | Path, Path]]:
    """Generate mediated desktop and icon assets for core24+ packaging.

    ``icon`` must be the cached result of :func:`get_mediated_icon_asset`.
    Desktop files are re-rendered on each call so that edits to their sources
    are always reflected in pack-time change detection.
    """
    assets: list[tuple[str | bytes | Path, Path]] = []
    relative_icon_path = icon.icon_path if icon is not None else None

    if icon is not None and icon.source is not None:
        assets.append((icon.source, prime_dir / icon.destination))

    if not project.apps:
        return assets

    for app_name, app in project.apps.items():
        if app.desktop:
            desktop_filename = app.desktop
            desktop_source = prime_dir / desktop_filename
            if not desktop_source.is_file():
                desktop_source = assets_dir / desktop_filename

            desktop_file = DesktopFile(
                snap_name=project.name,
                app_name=app_name,
                filename=str(desktop_source.relative_to(prime_dir))
                if desktop_source.is_relative_to(prime_dir)
                else os.fspath(desktop_source),
                prime_dir=prime_dir,
            )
            assets.append(
                (
                    desktop_file.render(icon_path=relative_icon_path),
                    prime_dir / "meta" / "gui" / f"{app_name}.desktop",
                )
            )

    return assets


def validate_command_chains(project: models.Project, *, prime_dir: Path) -> None:
    """Validate app command chains before packing.

    Called once at metadata-write time so errors surface before the pack-time
    ``needs_packing`` check, which must remain free of side effects.
    """
    if not project.apps:
        return

    for app_name, app in project.apps.items():
        _validate_command_chain(
            app.command_chain, name=f"app {app_name!r}", prime_dir=prime_dir
        )


def copy_assets(
    assets_dir: Path,
    prime_dir: Path,
    meta_directory_handler: Callable[[Path, Path], None] | None = None,
) -> None:
    """Copy assets into the prime dir.

    :param assets_dir: The directory containing snap project assets.
    :param prime_dir: The directory containing the content to be snapped.
    :param meta_directory_handler: callback that overrides default behavior for
        handling hooks and gui. The arguments passed are assets_dir and prime_dir.
    """
    if meta_directory_handler:
        meta_directory_handler(assets_dir, prime_dir)
    else:
        _write_snap_directory(
            assets_dir=assets_dir, prime_dir=prime_dir, meta_dir=prime_dir / "meta"
        )
        # create wrappers for hooks in the snap/hooks directory
        create_hook_wrappers(prime_dir)


def setup_hooks(hooks: dict[str, models.Hook] | None, prime_dir: Path) -> None:
    """Set up hooks assets.

    :param hooks: A dictionary of hooks to set up.
    :param prime_dir: The prime directory where the hooks should be set up.
    """
    hooks_dir = prime_dir / "meta" / "hooks"

    if hooks:
        for hook_name, hook in hooks.items():
            if hook.command_chain:
                _validate_command_chain(
                    hook.command_chain, name=f"hook {hook_name!r}", prime_dir=prime_dir
                )
            _ensure_hook(hooks_dir / hook_name)

    # Ensure all hooks are executable
    if hooks_dir.is_dir():
        for hook in hooks_dir.iterdir():
            _ensure_hook_executable(hook)


def _finalize_icon(
    icon: str | None, *, assets_dir: Path, gui_dir: Path, prime_dir: Path
) -> Path | None:
    """Ensure sure icon is properly configured and installed.

    Fetch from a remote URL, if required, and place in the meta/gui
    directory.
    """
    emit.debug(f"finalize icon: {icon!r}")

    # Nothing to do if no icon is configured, search for existing icon.
    if icon is None:
        return _find_icon_file(assets_dir)

    # Extracted appstream icon paths will either:
    # (1) point to a file relative to prime
    # (2) point to a remote http(s) url
    #
    # The 'icon' specified in the snapcraft.yaml has the same
    # constraint as (2) and would have already been validated
    # as existing by the schema.  So we can treat it the same
    # at this point, regardless of the source of the icon.
    parsed_url = urllib.parse.urlparse(icon)
    parsed_path = Path(parsed_url.path)
    icon_ext = parsed_path.suffix[1:]
    target_icon_path = Path(gui_dir, f"icon.{icon_ext}")

    target_icon_path.parent.mkdir(parents=True, exist_ok=True)
    if parsed_url.scheme in ["http", "https"]:
        # Remote - fetch URL and write to target.
        emit.progress(f"Fetching icon from {icon!r}")
        icon_data = requests.get(icon, timeout=120).content
        target_icon_path.write_bytes(icon_data)
    elif parsed_url.scheme == "":
        source_path = Path(
            prime_dir,
            parsed_path.relative_to("/") if parsed_path.is_absolute() else parsed_path,
        )
        if source_path.exists():
            # Local with path relative to prime.
            _copy_file(source_path, target_icon_path)
        elif parsed_path.exists():
            # Local with path relative to project.
            _copy_file(parsed_path, target_icon_path)
        else:
            # No icon found, fall back to searching for existing icon.
            return _find_icon_file(assets_dir)
    else:
        raise RuntimeError(f"Unexpected icon path: {parsed_url!r}")

    return target_icon_path


def _get_mediated_icon_asset(
    icon: str | None, *, assets_dir: Path, prime_dir: Path
) -> MediatedIconAsset | None:
    destination: Path | None = None

    if icon is None:
        icon_path = _find_icon_file(assets_dir)
        if icon_path is None:
            return None

        destination = Path("meta", "gui", icon_path.name)
        return MediatedIconAsset(
            source=None, destination=destination, icon_path=destination.as_posix()
        )

    parsed_url = urllib.parse.urlparse(icon)
    parsed_path = Path(parsed_url.path)
    icon_ext = parsed_path.suffix[1:]
    destination = Path("meta", "gui", f"icon.{icon_ext}")

    if parsed_url.scheme in ["http", "https"]:
        emit.progress(f"Fetching icon from {icon!r}")
        response = requests.get(icon, timeout=120)
        response.raise_for_status()
        return MediatedIconAsset(
            source=response.content,
            destination=destination,
            icon_path=destination.as_posix(),
        )

    if parsed_url.scheme == "":
        source_path = Path(
            prime_dir,
            parsed_path.relative_to("/") if parsed_path.is_absolute() else parsed_path,
        )
        if source_path.exists():
            return MediatedIconAsset(
                source=source_path,
                destination=destination,
                icon_path=destination.as_posix(),
            )

        if parsed_path.exists():
            return MediatedIconAsset(
                source=parsed_path,
                destination=destination,
                icon_path=destination.as_posix(),
            )

        return _get_mediated_icon_asset(
            None, assets_dir=assets_dir, prime_dir=prime_dir
        )

    raise RuntimeError(f"Unexpected icon path: {parsed_url!r}")


def _find_icon_file(assets_dir: Path) -> Path | None:
    for icon_path in (assets_dir / "gui/icon.png", assets_dir / "gui/icon.svg"):
        if icon_path.is_file():
            return icon_path
    return None


def _validate_command_chain(
    command_chain: list[str], *, name: str, prime_dir: Path
) -> None:
    """Verify if each item in the command chain is executable."""
    for item in command_chain:
        executable_path = prime_dir / item

        # command-chain entries must always be relative to the root of
        # the snap, i.e. PATH is not used.
        if not _is_executable(executable_path):
            raise errors.SnapcraftError(
                f"Failed to generate snap metadata: The command-chain item {item!r} "
                f"defined in {name} does not exist or is not executable.",
                resolution=f"Ensure that {item!r} is relative to the prime directory.",
            )


def _is_executable(path: Path) -> bool:
    """Verify if the given path corresponds to an executable file."""
    if not path.is_file():
        return False

    mode = path.stat().st_mode
    return bool(mode & stat.S_IXUSR or mode & stat.S_IXGRP or mode & stat.S_IXOTH)


def _write_snap_directory(*, assets_dir: Path, prime_dir: Path, meta_dir: Path) -> None:
    """Record manifest and copy assets found under the assets directory.

    These assets have priority over any code generated assets and include:
    - hooks
    - gui
    """
    prime_snap_dir = prime_dir / "snap"

    snap_dir_iter = itertools.product([prime_snap_dir], ["hooks", "gui"])
    meta_dir_iter = itertools.product([meta_dir], ["hooks", "gui"])

    for origin in itertools.chain(snap_dir_iter, meta_dir_iter):
        src_dir = assets_dir / origin[1]
        dst_dir = origin[0] / origin[1]

        if src_dir.is_dir():
            dst_dir.mkdir(parents=True, exist_ok=True)
            for asset in os.listdir(src_dir):
                source = src_dir / asset
                destination = dst_dir / asset

                destination.unlink(missing_ok=True)

                _copy_file(source, destination, follow_symlinks=True)


def _ensure_hook(hook_path: Path) -> None:
    """Create a stub for hook_path if it does not exist.

    A stub for hook_name is generated if a command-chain entry is defined
    to ensure the command-chain for a defined hook runs.

    A command-chain with no hook can occur when using extensions.
    """
    # The hook can exist if it was copied over from snap/hooks from the
    # project root or from prime_dir/snap/hooks (provided by a part).
    if hook_path.exists():
        return

    hook_path.parent.mkdir(parents=True, exist_ok=True)
    hook_path.write_text("#!/bin/true\n")


def _ensure_hook_executable(hook_path: Path) -> None:
    """Ensure hook is executable.

    :param hook_path: file path of the hook
    """
    if not hook_path.stat().st_mode & stat.S_IEXEC:
        hook_path.chmod(0o755)


def create_hook_wrappers(prime_dir: Path, *, overwrite: bool = True) -> None:
    """Create wrappers for hooks.

    Hooks in the snap/hooks/ directory are typically built by parts.
    When the snap is packed, these hooks stay in the snap/hooks/ directory.
    A hook wrapper (minimal shell script) in meta/hooks/ will
    execute the hook in $SNAP/snap/hooks/.

    :param prime_dir: The directory containing the content to be snapped.
    :param overwrite: Whether wrappers should replace existing meta/hooks entries.
    """
    hooks_in_snap_dir = _get_built_hooks(prime_dir)
    if not hooks_in_snap_dir:
        return

    hooks_meta_dir = _ensure_meta_hooks_dir(prime_dir)

    # create a wrapper for each hook
    for hook in hooks_in_snap_dir:
        _ensure_hook_executable(hook)
        _write_hook_wrapper(hook.name, hooks_meta_dir / hook.name, overwrite=overwrite)


def provision_hooks(prime_dir: Path, *, overwrite: bool = True) -> None:
    """Provision built hooks directly into meta/hooks.

    Hooks in snap/hooks/ are copied into meta/hooks/ so the primed tree contains
    directly runnable hook payloads. Existing meta/hooks entries can be preserved
    to let project hooks override generated ones.

    :param prime_dir: The directory containing the content to be snapped.
    :param overwrite: Whether to replace an existing hook in meta/hooks.
    """
    hooks_in_snap_dir = _get_built_hooks(prime_dir)
    if not hooks_in_snap_dir:
        return

    hooks_meta_dir = _ensure_meta_hooks_dir(prime_dir)

    for hook in hooks_in_snap_dir:
        _ensure_hook_executable(hook)

        destination = hooks_meta_dir / hook.name
        if destination.exists() and not overwrite:
            continue

        destination.unlink(missing_ok=True)
        _copy_file(hook, destination, follow_symlinks=True)
        _ensure_hook_executable(destination)


def _get_built_hooks(prime_dir: Path) -> list[Path]:
    """Return hooks built into ``snap/hooks`` for the given prime directory."""
    hooks_snap_dir = Path(prime_dir, "snap", "hooks")
    if not hooks_snap_dir.is_dir():
        return []

    return [hook for hook in hooks_snap_dir.iterdir() if hook.is_file()]


def _ensure_meta_hooks_dir(prime_dir: Path) -> Path:
    """Ensure ``meta/hooks`` exists and return its path."""
    hooks_meta_dir = Path(prime_dir, "meta", "hooks")
    hooks_meta_dir.mkdir(parents=True, exist_ok=True)
    return hooks_meta_dir


def _write_hook_wrapper(
    hook_name: str, wrapper_path: Path, *, overwrite: bool = True
) -> None:
    """Write hook wrapper file.

    The wrapper is a minimal shell script that calls a hook in $SNAP/snap/hooks/

    :param hook_name: name of the hook
    :param wrapper_path: file path of hook wrapper
    :param overwrite: Whether to replace an existing hook at wrapper_path.
    """
    if wrapper_path.exists():
        if not overwrite:
            return
        wrapper_path.unlink()

    with open(wrapper_path, "w+", encoding="utf-8") as wrapper_file:
        print("#!/bin/sh", file=wrapper_file)
        print(
            f'exec "{Path("$SNAP", "snap", "hooks", hook_name)}" "$@"',
            file=wrapper_file,
        )

    wrapper_path.chmod(0o755)


def _copy_file(source: Path, destination: Path, **kwargs) -> None:
    """Copy file if source and destination are not the same file."""
    if destination.exists() and source.samefile(destination):
        emit.debug(
            f"skip copying {str(source)!r}: source and destination are the same file"
        )
    else:
        shutil.copy(source, destination, **kwargs)
