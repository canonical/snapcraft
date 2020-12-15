# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2019 Canonical Ltd
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

import configparser
import logging
import os
import shlex
from typing import Optional

from . import errors

logger = logging.getLogger(__name__)


class DesktopFile:
    def __init__(
        self, *, snap_name: str, app_name: str, filename: str, prime_dir: str
    ) -> None:
        self._snap_name = snap_name
        self._app_name = app_name
        self._filename = filename
        self._prime_dir = prime_dir
        self._path = os.path.join(prime_dir, filename)
        if not os.path.exists(self._path):
            raise errors.InvalidDesktopFileError(
                filename, "does not exist (defined in the app {!r})".format(app_name)
            )

    def _parse_and_reformat_section_exec(self, section):
        exec_value = self._parser[section]["Exec"]
        exec_split = shlex.split(exec_value, posix=False)

        # Ensure command is invoked correctly.
        if self._app_name == self._snap_name:
            exec_split[0] = self._app_name
        else:
            exec_split[0] = "{}.{}".format(self._snap_name, self._app_name)

        self._parser[section]["Exec"] = " ".join(exec_split)

    def _parse_and_reformat_section(self, *, section, icon_path: Optional[str] = None):
        if "Exec" not in self._parser[section]:
            raise errors.InvalidDesktopFileError(self._filename, "missing 'Exec' key")

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
            if not os.path.exists(os.path.join(self._prime_dir, icon)):
                logger.warning(
                    "Icon {} specified in desktop file {} not found "
                    "in prime directory".format(icon, self._filename)
                )
            # if it is, add "${SNAP}" back and set the icon
            else:
                self._parser[section]["Icon"] = os.path.join("${SNAP}", icon)

    def _parse_and_reformat(self, *, icon_path: Optional[str] = None) -> None:
        self._parser = configparser.ConfigParser(interpolation=None)
        # mypy type checking ignored, see https://github.com/python/mypy/issues/506
        self._parser.optionxform = str  # type: ignore
        self._parser.read(self._path, encoding="utf-8")

        if "Desktop Entry" not in self._parser.sections():
            raise errors.InvalidDesktopFileError(
                self._filename, "missing 'Desktop Entry' section"
            )

        for section in self._parser.sections():
            self._parse_and_reformat_section(section=section, icon_path=icon_path)

    def write(self, *, gui_dir: str, icon_path: Optional[str] = None) -> None:
        self._parse_and_reformat(icon_path=icon_path)

        os.makedirs(gui_dir, exist_ok=True)

        # Rename the desktop file to match the app name. This will help
        # unity8 associate them (https://launchpad.net/bugs/1659330).
        target_filename = "{}.desktop".format(self._app_name)
        target = os.path.join(gui_dir, target_filename)
        if os.path.exists(target):
            # Unlikely. A desktop file in setup/gui/ already existed for
            # this app. Let's pretend it wasn't there and overwrite it.
            os.remove(target)
        with open(target, "w", encoding="utf-8") as f:
            self._parser.write(f, space_around_delimiters=False)
