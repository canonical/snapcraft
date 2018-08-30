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

import codecs
from collections import OrderedDict
from copy import deepcopy

import yaml
import yaml.reader

from . import errors


class ProjectInfo:
    """Information gained from the snap's snapcraft.yaml file."""

    def __init__(self, *, snapcraft_yaml_file_path) -> None:
        self.snapcraft_yaml_file_path = snapcraft_yaml_file_path
        self.__raw_snapcraft = _load_yaml(yaml_file_path=snapcraft_yaml_file_path)

        try:
            self.name = self.__raw_snapcraft["name"]
        except KeyError as key_error:
            raise errors.YamlValidationError(
                "'name' is a required property in {!r}".format(snapcraft_yaml_file_path)
            ) from key_error
        self.version = self.__raw_snapcraft.get("version")
        self.summary = self.__raw_snapcraft.get("summary")
        self.description = self.__raw_snapcraft.get("description")
        self.confinement = self.__raw_snapcraft.get("confinement")
        self.architectures = self.__raw_snapcraft.get("architectures")
        self.grade = self.__raw_snapcraft.get("grade")
        self.base = self.__raw_snapcraft.get("base")

    def get_raw_snapcraft(self):
        # TODO this should be a MappingProxyType, but ordered writing
        #      depends on reading in the current code base.
        return deepcopy(self.__raw_snapcraft)


def _load_yaml(*, yaml_file_path: str) -> OrderedDict:
    with open(yaml_file_path, "rb") as fp:
        bs = fp.read(2)

    if bs == codecs.BOM_UTF16_LE or bs == codecs.BOM_UTF16_BE:
        encoding = "utf-16"
    else:
        encoding = "utf-8"

    try:
        with open(yaml_file_path, encoding=encoding) as fp:  # type: ignore
            yaml_contents = yaml.safe_load(fp)  # type: ignore
    except yaml.scanner.ScannerError as e:
        raise errors.YamlValidationError(
            "{} on line {} of {}".format(
                e.problem, e.problem_mark.line + 1, yaml_file_path
            )
        ) from e
    except yaml.reader.ReaderError as e:
        raise errors.YamlValidationError(
            "Invalid character {!r} at position {} of {}: {}".format(
                chr(e.character), e.position + 1, yaml_file_path, e.reason
            )
        ) from e

    return yaml_contents
