# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""External metadata helpers."""

from typing import Optional

from . import appstream
from .extracted_metadata import ExtractedMetadata


def extract_metadata(file_relpath: str, *, workdir: str) -> Optional[ExtractedMetadata]:
    """Retrieve external metadata from part files.

    :param file_relpath: Relative path to the file containing metadata.
    :param workdir: The part working directory where the metadata file is located.

    :return: The extracted metadata, if any.
    """
    return appstream.extract(file_relpath, workdir=workdir)
