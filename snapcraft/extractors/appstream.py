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

from ._metadata import ExtractedMetadata
from snapcraft.extractors import _errors

from xml.etree.ElementTree import (
    ElementTree,
    ParseError
)


def extract(path: str) -> ExtractedMetadata:
    if (not path.endswith('.metainfo.xml') and
            not path.endswith('.appdata.xml')):
        raise _errors.UnhandledFileError(path, 'appstream')

    try:
        tree = ElementTree().parse(path)
    except ParseError as e:
        raise _errors.AppstreamFileParseError(path) from e

    summary = None
    node = tree.find('summary')
    if node is not None:
        summary = node.text

    description = None
    node = tree.find('description')
    if node is not None:
        description = node.text

    return ExtractedMetadata(summary=summary, description=description)
