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

from snapcraft.internal import errors


class CommandError(errors.SnapcraftError):

    def __init__(self, message: str) -> None:
        self.fmt = message


class SnapMetaGenerationError(errors.SnapcraftError):
    pass


class SnapcraftYamlMissingRequiredValue(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "Missing {key!r} from the 'snapcraft.yaml' file."
    )

    def __init__(self, key):
        super().__init__(key=key)


class WrongAdoptInfo(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'adopt-info' refers to a part named {part!r}, but it is not defined "
        "in the 'snapcraft.yaml' file.")

    def __init__(self, part):
        super().__init__(part=part)


class UnexistingSourceMetaPath(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'parse-info' in the 'snapcraft.yaml' file refers to a metadata file "
        "in {path!r}, which does not exist.")

    def __init__(self, path):
        super().__init__(path=path)


class AppstreamFileParseError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'parse-info' in the 'snapcraft.yaml' file refers to an appstream "
        "metadata file in {path!r}, which is not a valid XML file.")

    def __init__(self, path):
        super().__init__(path=path)


class MissingSnapcraftYamlKeys(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "Missing required keys in the 'snapcraft.yaml' file: {keys!r}.")

    def __init__(self, keys):
        super().__init__(keys=keys)


class SourceMetadataParserError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'parse-info' in the 'snapcraft.yaml' file refers to a metadata file "
        "in {path!r}, which cannot be parsed.")

    def __init__(self, path):
        super().__init__(path=path)
>>>>>>> metadata: extract metadata from appstream
