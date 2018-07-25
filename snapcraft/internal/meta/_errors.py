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

from snapcraft import formatting_utils
from snapcraft.internal import errors
from typing import List


class CommandError(errors.SnapcraftError):
    def __init__(self, message: str) -> None:
        self.fmt = message


class SnapMetaGenerationError(errors.SnapcraftError):
    pass


class MissingSnapcraftYamlKeysError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "Missing required key(s) in snapcraft.yaml: {keys}. "
        "Either specify the missing key(s), or use 'adopt-info' to get them "
        "from a part."
    )

    def __init__(self, keys: list) -> None:
        super().__init__(keys=formatting_utils.humanize_list(keys, "and"))


class AdoptedPartMissingError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'adopt-info' refers to a part named {part!r}, but it is not defined "
        "in the 'snapcraft.yaml' file."
    )

    def __init__(self, part: str) -> None:
        super().__init__(part=part)


class AdoptedPartNotParsingInfo(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'adopt-info' refers to part {part!r}, but that part is lacking the "
        "'parse-info' property."
    )

    def __init__(self, part: str) -> None:
        super().__init__(part=part)


class AmbiguousPassthroughKeyError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "The following keys are specified in their regular location "
        "as well as in passthrough: {keys}. "
        "Remove duplicate keys."
    )

    def __init__(self, keys: List[str]) -> None:
        super().__init__(keys=formatting_utils.humanize_list(keys, "and"))
