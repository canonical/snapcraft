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

"""Copy assets to their final locations."""

import itertools
import os
import shutil
import stat
import urllib.parse
from pathlib import Path
from typing import Callable, List, Optional

import requests
from craft_cli import emit

from snapcraft import errors, models

from .desktop_file import DesktopFile


def setup_assets(
    project: models.Project,
    *,
    assets_dir: Path,
    project_dir: Path,
    prime_dir: Path,
    meta_directory_handler: Optional[Callable[[Path, Path], None]] = None,
) -> None:
    """Copy assets to the appropriate locations in the snap filesystem.

    :param project: The snap project file.
    :param assets_dir: The directory containing snap project assets.
    :param project_dir: The project root directory.
    :param prime_dir: The directory containing the content to be snapped.
    :param meta_directory_handler: callback that overrides default behavior for
        handling hooks and gui. The arguments passed are assets_dir and prime_dir.
    """
    meta_dir = prime_dir / "meta"
    gui_dir = meta_dir / "gui"
    gui_dir.mkdir(parents=True, exist_ok=True)
    hooks_dir = meta_dir / "hooks"

    if meta_directory_handler:
        meta_directory_handler(assets_dir, prime_dir)
    else:
        _write_snap_directory(
            assets_dir=assets_dir, prime_dir=prime_dir, meta_dir=meta_dir
        )
        # create wrappers for hooks in the snap/hooks directory
        _create_hook_wrappers(prime_dir)

    # hooks with command chains will execute the command chain instead of the hook wrapper
    if project.hooks:
        for hook_name, hook in project.hooks.items():
            if hook.command_chain:
                _validate_command_chain(
                    hook.command_chain, name=f"hook {hook_name!r}", prime_dir=prime_dir
                )
            _ensure_hook(meta_dir / "hooks" / hook_name)

    # Ensure all hooks are executable
    if hooks_dir.is_dir():
        for hook in hooks_dir.iterdir():  # type: ignore
            # mypy says we are passing a Hook, but this is indeed a Path
            _ensure_hook_executable(hook)  # type: ignore

    if project.type == "gadget":
        gadget_yaml = project_dir / "gadget.yaml"
        if not gadget_yaml.exists():
            raise errors.SnapcraftError("gadget.yaml is required for gadget snaps")
        _copy_file(gadget_yaml, meta_dir / "gadget.yaml")

    if project.type == "kernel":
        kernel_yaml = project_dir / "kernel.yaml"
        if kernel_yaml.exists():
            _copy_file(kernel_yaml, meta_dir / "kernel.yaml")

    if not project.apps:
        return

    icon_path = _finalize_icon(
        project.icon, assets_dir=assets_dir, gui_dir=gui_dir, prime_dir=prime_dir
    )
    relative_icon_path: Optional[str] = None

    if icon_path is not None:
        if prime_dir in icon_path.parents:
            icon_path = icon_path.relative_to(prime_dir)
        relative_icon_path = str(icon_path)

    emit.debug(f"relative icon path: {relative_icon_path!r}")

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


def _finalize_icon(
    icon: Optional[str], *, assets_dir: Path, gui_dir: Path, prime_dir: Path
) -> Optional[Path]:
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


def _find_icon_file(assets_dir: Path) -> Optional[Path]:
    for icon_path in (assets_dir / "gui/icon.png", assets_dir / "gui/icon.svg"):
        if icon_path.is_file():
            return icon_path
    return None


def _validate_command_chain(
    command_chain: List[str], *, name: str, prime_dir: Path
) -> None:
    """Verify if each item in the command chain is executble."""
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


def _create_hook_wrappers(prime_dir: Path) -> None:
    """Create wrappers for hooks.

    Hooks in the snap/hooks/ directory are typically built by parts.
    When the snap is packed, these hooks stay in the snap/hooks/ directory.
    A hook wrapper (minimal shell script) in meta/hooks/ will
    execute the hook in $SNAP/snap/hooks/.

    :param prime_dir: The directory containing the content to be snapped.
    """
    # collect hooks in snap/hooks directory
    hooks_snap_dir = Path(prime_dir, "snap", "hooks")
    hooks_in_snap_dir = hooks_snap_dir.iterdir() if hooks_snap_dir.is_dir() else []

    # return if there are no hooks to process
    if not hooks_in_snap_dir:
        return

    # create directory to hold hook wrappers
    hooks_meta_dir = Path(prime_dir, "meta", "hooks")
    hooks_meta_dir.mkdir(parents=True, exist_ok=True)

    # create a wrapper for each hook
    for hook in hooks_in_snap_dir:
        _ensure_hook_executable(hook)
        _write_hook_wrapper(hook.name, hooks_meta_dir / hook.name)


def _write_hook_wrapper(hook_name: str, wrapper_path: Path) -> None:
    """Write hook wrapper file.

    The wrapper is a minimal shell script that calls a hook in $SNAP/snap/hooks/

    :param hook_name: name of the hook
    :param wrapper_path: file path of hook wrapper
    """
    wrapper_path.unlink(missing_ok=True)

    with open(wrapper_path, "w+", encoding="utf-8") as wrapper_file:
        print("#!/bin/sh", file=wrapper_file)
        print(
            f'exec "{Path("$SNAP", "snap", "hooks", hook_name)}" "$@"',
            file=wrapper_file,
        )


def _copy_file(source: Path, destination: Path, **kwargs) -> None:
    """Copy file if source and destination are not the same file."""
    if destination.exists() and source.samefile(destination):
        emit.debug(
            f"skip copying {str(source)!r}: source and destination are the same file"
        )
    else:
        shutil.copy(source, destination, **kwargs)
