# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from copy import deepcopy

import snapcraft.yaml_utils.errors
from snapcraft import yaml_utils

from . import _schema


class ProjectInfo:
    """Information gained from the snap's snapcraft.yaml file."""

    def __init__(self, *, snapcraft_yaml_file_path) -> None:
        self.snapcraft_yaml_file_path = snapcraft_yaml_file_path
        self.__raw_snapcraft = yaml_utils.load_yaml_file(snapcraft_yaml_file_path)

        try:
            self.name = self.__raw_snapcraft["name"]
        except KeyError as key_error:
            raise snapcraft.yaml_utils.errors.YamlValidationError(
                "'name' is a required property in {!r}".format(snapcraft_yaml_file_path)
            ) from key_error
        self.version = self.__raw_snapcraft.get("version")
        self.summary = self.__raw_snapcraft.get("summary")
        self.description = self.__raw_snapcraft.get("description")
        self.confinement = self.__raw_snapcraft.get("confinement")
        self.architectures = self.__raw_snapcraft.get("architectures")
        self.grade = self.__raw_snapcraft.get("grade")
        self.base = self.__raw_snapcraft.get("base")
        self.build_base = self.__raw_snapcraft.get("build-base")
        self.type = self.__raw_snapcraft.get("type")

    def validate_raw_snapcraft(self):
        """Validate the snapcraft.yaml for this project."""
        _schema.Validator(self.__raw_snapcraft).validate()

    def get_raw_snapcraft(self):
        # TODO this should be a MappingProxyType, but ordered writing
        #      depends on reading in the current code base.
        return deepcopy(self.__raw_snapcraft)
