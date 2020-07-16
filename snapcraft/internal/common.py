# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

# Data/methods shared between plugins and snapcraft
import glob
import logging
import math
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import tempfile
import urllib
from contextlib import suppress
from pathlib import Path
from typing import Callable, List, Union

from snapcraft.internal import errors

SNAPCRAFT_FILES = ["parts", "stage", "prime"]
_DEFAULT_PLUGINDIR = os.path.join(sys.prefix, "share", "snapcraft", "plugins")
_plugindir = _DEFAULT_PLUGINDIR
_DEFAULT_SCHEMADIR = os.path.join(sys.prefix, "share", "snapcraft", "schema")
_schemadir = _DEFAULT_SCHEMADIR
_DEFAULT_EXTENSIONSDIR = os.path.join(sys.prefix, "share", "snapcraft", "extensions")
_extensionsdir = _DEFAULT_EXTENSIONSDIR
_DEFAULT_KEYRINGSDIR = os.path.join(sys.prefix, "share", "snapcraft", "keyrings")
_keyringsdir = _DEFAULT_KEYRINGSDIR
_DEFAULT_LEGACY_SNAPCRAFT_DIR = os.path.join(sys.prefix, "legacy_snapcraft")
_legacy_snapcraft_dir = _DEFAULT_LEGACY_SNAPCRAFT_DIR

_DOCKERENV_FILE = "/.dockerenv"
_PODMAN_FILE = "/run/.containerenv"

MAX_CHARACTERS_WRAP = 120

env = []  # type: List[str]

logger = logging.getLogger(__name__)


def assemble_env():
    return "\n".join(["export " + e for e in env])


run_number: int = 0


def _run(cmd: List[str], runner: Callable, **kwargs):
    global run_number
    run_number += 1

    assert isinstance(cmd, list), "run command must be a list"

    lines: List[str] = list()

    # Set shell.
    lines.append("#!/bin/sh")

    # Account for `env` parameter by populating exports.
    # Ordering matters: assembled_env overrides `env` parameter.
    cmd_env = kwargs.pop("env", None)
    if cmd_env:
        lines.append("#############################")
        lines.append("# Exported via `env` parameter:")
        for key in sorted(cmd_env.keys()):
            value = cmd_env.get(key)
            lines.append(f"export {key}={value!r}")

    # Account for assembled_env.
    lines.append("#############################")
    lines.append("# Exported via assembled env:")
    lines.extend(["export " + e for e in env])

    # Account for `cwd` by changing directory.
    cmd_workdir = kwargs.pop("cwd", None)
    if cmd_workdir:
        lines.append("#############################")
        lines.append("# Configured via `cwd` parameter:")
    else:
        cmd_workdir = os.getcwd()
        lines.append("#############################")
        lines.append("# Implicit working directory:")
    lines.append(f"cd {cmd_workdir!r}")

    # Finally, execute desired command.
    lines.append("#############################")
    lines.append("# Execute command:")
    cmd_string = " ".join([shlex.quote(c) for c in cmd])
    lines.append(f"exec {cmd_string}")

    # Save script executed by snapcraft.
    pid = os.getpid()
    temp_dir = Path(tempfile.gettempdir(), f"snapcraft-{pid}")
    temp_dir.mkdir(mode=0o755, parents=True, exist_ok=True)

    script_path = temp_dir / f"run-{run_number}.sh"
    script = "\n".join(lines) + "\n"

    # Write script.
    script_path.write_text(script)
    script_path.chmod(0o755)

    runner_command = ["/bin/sh", str(script_path)]
    runner_command_string = " ".join([shlex.quote(c) for c in runner_command])
    try:
        logger.debug(f"Executing assembled script: {runner_command_string!r}")
        return runner(runner_command, **kwargs)
    except subprocess.CalledProcessError as call_error:
        raise errors.SnapcraftCommandError(
            command=cmd_string, call_error=call_error
        ) from call_error


def run(cmd: List[str], **kwargs) -> None:
    _run(cmd, subprocess.check_call, **kwargs)


def run_output(cmd: List[str], **kwargs) -> str:
    output = _run(cmd, subprocess.check_output, **kwargs)
    try:
        return output.decode(sys.getfilesystemencoding()).strip()
    except UnicodeEncodeError:
        logger.warning("Could not decode output for {!r} correctly".format(cmd))
        return output.decode("latin-1", "surrogateescape").strip()


def get_installed_snap_path(snap_name: str):
    """Returns the path to the currently installed snap."""
    return os.path.join(os.path.sep, "snap", snap_name, "current")


def format_snap_name(snap, *, allow_empty_version: bool = False) -> str:
    """Return a filename representing the snap depending on snap attributes.

    :param dict snap: a dictionary of keys containing name, version and arch.
    :param bool allow_empty_version: if set a filename without a version is
                                     allowed.
    """
    if allow_empty_version and snap.get("version") is None:
        extension = "{name}_{arch}.snap"
    else:
        extension = "{name}_{version}_{arch}.snap"

    if "arch" not in snap:
        snap["arch"] = snap.get("architectures", None)
    if not snap["arch"]:
        snap["arch"] = "all"
    elif len(snap["arch"]) == 1:
        snap["arch"] = snap["arch"][0]
    else:
        snap["arch"] = "multi"

    return extension.format(**snap)


def is_snap() -> bool:
    snap_name = os.environ.get("SNAP_NAME", "")
    is_snap = snap_name == "snapcraft"
    logger.debug(
        "snapcraft is running as a snap {!r}, "
        "SNAP_NAME set to {!r}".format(is_snap, snap_name)
    )
    return is_snap


def is_process_container() -> bool:
    logger.debug("snapcraft is running in a docker or podman (OCI) container")
    return any([os.path.exists(p) for p in (_DOCKERENV_FILE, _PODMAN_FILE)])


def set_plugindir(plugindir):
    global _plugindir
    _plugindir = plugindir


def get_plugindir():
    return _plugindir


def set_schemadir(schemadir):
    global _schemadir
    _schemadir = schemadir


def get_schemadir():
    return _schemadir


def set_extensionsdir(extensionsdir):
    global _extensionsdir
    _extensionsdir = extensionsdir


def get_extensionsdir():
    return _extensionsdir


def set_keyringsdir(keyringsdir):
    global _keyringsdir
    _keyringsdir = keyringsdir


def get_keyringsdir():
    return _keyringsdir


def get_arch_triplet():
    raise errors.PluginOutdatedError("use 'project.arch_triplet'")


def get_arch():
    raise errors.PluginOutdatedError("use 'project.deb_arch'")


def get_parallel_build_count():
    raise errors.PluginOutdatedError("use 'parallel_build_count'")


def set_legacy_snapcraft_dir(snapcraftdir):
    global _legacy_snapcraft_dir
    _legacy_snapcraft_dir = snapcraftdir


def get_legacy_snapcraft_dir():
    return _legacy_snapcraft_dir


def get_python2_path(root):
    """Return a valid PYTHONPATH or raise an exception."""
    python_paths = glob.glob(
        os.path.join(root, "usr", "lib", "python2*", "dist-packages")
    )
    try:
        return python_paths[0]
    except IndexError:
        raise errors.SnapcraftEnvironmentError(
            "PYTHONPATH cannot be set for {!r}".format(root)
        )


def get_url_scheme(url):
    return urllib.parse.urlparse(url).scheme


def isurl(url):
    return get_url_scheme(url) != ""


def reset_env():
    global env
    env = []


def get_terminal_width(max_width=MAX_CHARACTERS_WRAP):
    if os.isatty(1):
        width = shutil.get_terminal_size().columns
    else:
        width = MAX_CHARACTERS_WRAP
    if max_width:
        width = min(max_width, width)
    return width


def format_output_in_columns(
    elements_list, max_width=MAX_CHARACTERS_WRAP, num_col_spaces=2
):
    """Return a formatted list of strings ready to be printed line by line

    elements_list is the list of elements ready to be printed on the output
    max_width is the number of characters the output shouldn't exceed
    num_col_spaces is the number of spaces set between 2 columns"""

    # First, try to get the starting point in term of number of lines
    total_num_chars = sum([len(elem) for elem in elements_list])
    num_lines = math.ceil(
        (total_num_chars + (len(elements_list) - 1) * num_col_spaces) / max_width
    )
    sep = " " * num_col_spaces

    candidate_output = []
    while not candidate_output:
        # dispatch elements in resulting list until num_lines
        for i, element in enumerate(elements_list):
            # for new columns, get the maximum width of this column
            if i % num_lines == 0:
                col_width = 0
                for j in range(i, i + num_lines):
                    # ignore non existing elements at the end
                    with suppress(IndexError):
                        col_width = max(len(elements_list[j]), col_width)

            if i < num_lines:
                candidate_output.append([])
            candidate_output[i % num_lines].append(element.ljust(col_width))

        # check that any line (like the first one) is still smaller than
        # max_width
        if len(sep.join(candidate_output[0])) > max_width:
            # reset and try with one more line
            num_lines += 1
            candidate_output = []

    result_output = []
    for i, line in enumerate(candidate_output):
        result_output.append(sep.join(candidate_output[i]))

    return result_output


def get_bin_paths(*, root: Union[str, pathlib.Path], existing_only=True) -> List[str]:
    paths = (os.path.join("usr", "sbin"), os.path.join("usr", "bin"), "sbin", "bin")
    rooted_paths = (os.path.join(root, p) for p in paths)

    if existing_only:
        return [p for p in rooted_paths if os.path.exists(p)]
    else:
        return list(rooted_paths)


def get_include_paths(root, arch_triplet):
    paths = [
        os.path.join(root, "include"),
        os.path.join(root, "usr", "include"),
        os.path.join(root, "include", arch_triplet),
        os.path.join(root, "usr", "include", arch_triplet),
    ]

    return [p for p in paths if os.path.exists(p)]


def get_library_paths(root, arch_triplet, existing_only=True):
    """Returns common library paths for a snap.

    If existing_only is set the paths returned must exist for
    the root that was set.
    """
    paths = [
        os.path.join(root, "lib"),
        os.path.join(root, "usr", "lib"),
        os.path.join(root, "lib", arch_triplet),
        os.path.join(root, "usr", "lib", arch_triplet),
    ]

    if existing_only:
        paths = [p for p in paths if os.path.exists(p)]

    return paths


def get_pkg_config_paths(root, arch_triplet):
    paths = [
        os.path.join(root, "lib", "pkgconfig"),
        os.path.join(root, "lib", arch_triplet, "pkgconfig"),
        os.path.join(root, "usr", "lib", "pkgconfig"),
        os.path.join(root, "usr", "lib", arch_triplet, "pkgconfig"),
        os.path.join(root, "usr", "share", "pkgconfig"),
        os.path.join(root, "usr", "local", "lib", "pkgconfig"),
        os.path.join(root, "usr", "local", "lib", arch_triplet, "pkgconfig"),
        os.path.join(root, "usr", "local", "share", "pkgconfig"),
    ]

    return [p for p in paths if os.path.exists(p)]
