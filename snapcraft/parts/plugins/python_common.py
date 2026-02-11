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

"""Common functionality for Python-based plugins.

This plugin extends Craft-parts' vanilla Python plugin to properly
set the Python interpreter according to the Snapcraft base and
confinement parameters.
"""

import logging
from pathlib import Path

from craft_parts import PartInfo, StepInfo, errors, plugins

from .poetry_plugin import PoetryPlugin
from .python_plugin import PythonPlugin
from .uv_plugin import UvPlugin

logger = logging.getLogger(__name__)


_CONFINED_PYTHON_PATH = {
    "core22": "/usr/bin/python3.10",
    "core24": "/usr/bin/python3.12",
}


def get_system_interpreter(part_info: PartInfo) -> str | None:
    """Obtain the path to the system-provided python interpreter.

    :param part_info: The info of the part that is being built.
    """
    base = part_info.project_base
    confinement = part_info.confinement

    if confinement == "classic" or base == "bare":
        # classic snaps, and snaps without bases, must always provision Python
        interpreter = None
    else:
        # otherwise, we should always know which Python is present on the
        # base. If this fails on a new base, update _CONFINED_PYTHON_PATH
        interpreter = _CONFINED_PYTHON_PATH.get(base)
        if interpreter is None:
            brief = f"Don't know which interpreter to use for base {base}."
            resolution = "Please contact the Snapcraft team."
            raise errors.PartsError(brief=brief, resolution=resolution)

    logger.debug(
        "Using python interpreter '%s' for base '%s', confinement '%s'",
        interpreter,
        base,
        confinement,
    )
    return interpreter


def post_prime(step_info: StepInfo) -> None:
    """Perform Python-specific actions right before packing."""
    base = step_info.project_base

    if base == "core22":
        # Only fix pyvenv.cfg on core24+ snaps
        return

    root_path: Path = step_info.prime_dir

    pyvenv = root_path / "pyvenv.cfg"
    if not pyvenv.is_file():
        return

    snap_path = Path(f"/snap/{step_info.project_name}/current")
    new_home = f"home = {snap_path}"

    candidates = (
        step_info.part_install_dir,
        step_info.stage_dir,
    )

    old_contents = contents = pyvenv.read_text()
    for candidate in candidates:
        old_home = f"home = {candidate}"
        contents = contents.replace(old_home, new_home)

    if old_contents != contents:
        logger.debug("Updating pyvenv.cfg to:\n%s", contents)
        pyvenv.write_text(contents)


def get_python_plugins() -> dict[str, plugins.plugins.PluginType]:
    """Get a list of currently supported Python-based plugins."""
    return {
        "poetry": PoetryPlugin,
        "python": PythonPlugin,
        "uv": UvPlugin,
    }
