# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

from snapcraft.internal.errors import MetadataExtractionError


class UnhandledFileError(MetadataExtractionError):

    fmt = (
        "Failed to extract metadata from {file_path!r}: "
        "This file is not handled by {extractor_name!r}."
    )

    def __init__(self, file_path: str, extractor_name: str) -> None:
        super().__init__(file_path=file_path, extractor_name=extractor_name)
