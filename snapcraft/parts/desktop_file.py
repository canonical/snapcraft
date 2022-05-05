# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2022 Canonical Ltd.
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

"""Desktop file parser."""

import configparser
import os
import shlex
from pathlib import Path
from typing import Optional

from craft_cli import emit

from snapcraft import errors


class DesktopFile:
    """Parse and process a .desktop file.

    :param snap_name: The snap package name.
    :param app_name: The name of the app using the desktop file.
    :param filename: The desktop file name.
    :param prime_dir: The prime directory path.

    :raises DesktopFileError: If the desktop file does not exist.
    """

    def __init__(
        self, *, snap_name: str, app_name: str, filename: str, prime_dir: Path
    ) -> None:
        self._snap_name = snap_name
        self._app_name = app_name
        self._filename = filename
        self._prime_dir = prime_dir

        file_path = prime_dir / filename
        if not file_path.is_file():
            raise errors.DesktopFileError(
                filename, f"file does not exist (defined in app {app_name!r})"
            )

        self._parser = configparser.ConfigParser(interpolation=None)
        # mypy type checking ignored, see https://github.com/python/mypy/issues/506
        self._parser.optionxform = str  # type: ignore
        self._parser.read(file_path, encoding="utf-8")

    def _parse_and_reformat_section_exec(self, section):
        exec_value = self._parser[section]["Exec"]
        exec_split = shlex.split(exec_value, posix=False)

        # Ensure command is invoked correctly.
        if self._app_name == self._snap_name:
            exec_split[0] = self._app_name
        else:
            exec_split[0] = f"{self._snap_name}.{self._app_name}"

        self._parser[section]["Exec"] = " ".join(exec_split)

    def _parse_and_reformat_section(self, *, section, icon_path: Optional[str] = None):
        if "Exec" not in self._parser[section]:
            raise errors.DesktopFileError(self._filename, "missing 'Exec' key")

        self._parse_and_reformat_section_exec(section)

        if "Icon" in self._parser[section]:
            icon = self._parser[section]["Icon"]

            if icon_path is not None:
                icon = icon_path

            # Strip any leading slash.
            icon = icon[1:] if icon.startswith("/") else icon

            # Strip any leading ${SNAP}.
            icon = icon[8:] if icon.startswith("${SNAP}") else icon

            # With everything stripped, check to see if the icon is there.
            # if it is, add "${SNAP}" back and set the icon
            if (self._prime_dir / icon).is_file():
                self._parser[section]["Icon"] = os.path.join("${SNAP}", icon)
            else:
                emit.message(
                    f"Icon {icon!r} specified in desktop file {self._filename!r} "
                    f"not found in prime directory."
                )

    def _parse_and_reformat(self, *, icon_path: Optional[str] = None) -> None:
        if "Desktop Entry" not in self._parser.sections():
            raise errors.DesktopFileError(
                self._filename, "missing 'Desktop Entry' section"
            )

        for section in self._parser.sections():
            self._parse_and_reformat_section(section=section, icon_path=icon_path)

    def write(self, *, gui_dir: Path, icon_path: Optional[str] = None) -> None:
        """Write the desktop file.

        :param gui_dir: The desktop file destination directory.
        :param icon_path: The icon corresponding to this desktop file.
        """
        self._parse_and_reformat(icon_path=icon_path)

        gui_dir.mkdir(parents=True, exist_ok=True)

        # Rename the desktop file to match the app name. This will help
        # unity8 associate them (https://launchpad.net/bugs/1659330).
        target = gui_dir / f"{self._app_name}.desktop"

        if target.exists():
            # Unlikely. A desktop file in meta/gui/ already existed for
            # this app. Let's pretend it wasn't there and overwrite it.
            target.unlink()
        with target.open("w", encoding="utf-8") as target_file:
            self._parser.write(target_file, space_around_delimiters=False)
