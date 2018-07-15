# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from snapcraft.internal import errors


logger = logging.getLogger(__name__)


class DesktopFile:
    def __init__(
        self, *, name: str, filename: str, snap_name: str, prime_dir: str
    ) -> None:
        self._name = name
        self._filename = filename
        self._snap_name = snap_name
        self._prime_dir = prime_dir
        self._path = os.path.join(prime_dir, filename)
        if not os.path.exists(self._path):
            raise errors.InvalidDesktopFileError(
                filename, "does not exist (defined in the app {!r})".format(name)
            )

    def parse_and_reformat(self):
        self._parser = configparser.ConfigParser(interpolation=None)
        self._parser.optionxform = str
        self._parser.read(self._path, encoding="utf-8")
        section = "Desktop Entry"
        if section not in self._parser.sections():
            raise errors.InvalidDesktopFileError(
                self._filename, "missing 'Desktop Entry' section"
            )
        if "Exec" not in self._parser[section]:
            raise errors.InvalidDesktopFileError(self._filename, "missing 'Exec' key")
        # XXX: do we want to allow more parameters for Exec?
        if self._name == self._snap_name:
            exec_value = "{} %U".format(self._name)
        else:
            exec_value = "{}.{} %U".format(self._snap_name, self._name)
        self._parser[section]["Exec"] = exec_value
        if "Icon" in self._parser[section]:
            icon = self._parser[section]["Icon"]
            if icon.startswith("/"):
                icon = icon.lstrip("/")
                if os.path.exists(os.path.join(self._prime_dir, icon)):
                    self._parser[section]["Icon"] = "${{SNAP}}/{}".format(icon)
                else:
                    logger.warning(
                        "Icon {} specified in desktop file {} not found "
                        "in prime directory".format(icon, self._filename)
                    )

    def write(self, *, gui_dir: str) -> None:
        # Rename the desktop file to match the app name. This will help
        # unity8 associate them (https://launchpad.net/bugs/1659330).
        target_filename = "{}.desktop".format(self._name)
        target = os.path.join(gui_dir, target_filename)
        if os.path.exists(target):
            # Unlikely. A desktop file in setup/gui/ already existed for
            # this app. Let's pretend it wasn't there and overwrite it.
            os.remove(target)
        with open(target, "w", encoding="utf-8") as f:
            self._parser.write(f, space_around_delimiters=False)
