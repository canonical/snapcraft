# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2018 Canonical Ltd
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

import os

from ._metadata import ExtractedMetadata
from snapcraft.extractors import _errors

from xml.etree.ElementTree import ElementTree, ParseError


def extract(path: str) -> ExtractedMetadata:
    if not path.endswith(".metainfo.xml") and not path.endswith(".appdata.xml"):
        raise _errors.UnhandledFileError(path, "appstream")

    try:
        tree = ElementTree().parse(path)
    except ParseError as e:
        raise _errors.AppstreamFileParseError(path) from e

    common_id = _get_value_from_xml_element(tree, "id")
    summary = _get_value_from_xml_element(tree, "summary")
    description = _get_value_from_xml_element(tree, "description")

    icon = None
    node = tree.find("icon")
    if node is not None and "type" in node.attrib and node.attrib["type"] == "local":
        # TODO Currently we only support local icons.
        # See bug https://bugs.launchpad.net/snapcraft/+bug/1742348
        # for supporting remote icons.
        # --elopio -20180109
        icon = node.text.strip()

    desktop_file_paths = []
    nodes = tree.findall("launchable")
    for node in nodes:
        if "type" in node.attrib and node.attrib["type"] == "desktop-id":
            desktop_file_id = node.text.strip()
            desktop_file_path = _desktop_file_id_to_path(desktop_file_id)
            if desktop_file_path:
                desktop_file_paths.append(desktop_file_path)

    return ExtractedMetadata(
        common_id=common_id,
        summary=summary,
        description=description,
        icon=icon,
        desktop_file_paths=desktop_file_paths,
    )


def _get_value_from_xml_element(tree, key) -> str:
    node = tree.find(key)
    if node is not None:
        return node.text.strip()
    else:
        return None


def _desktop_file_id_to_path(desktop_file_id: str) -> str:
    # For details about desktop file ids and their corresponding paths, see
    # https://standards.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id  # noqa
    for xdg_data_dir in ("usr/local/share", "usr/share"):
        desktop_file_path = os.path.join(
            xdg_data_dir, "applications", desktop_file_id.replace("-", "/")
        )
        if os.path.exists(desktop_file_path):
            return desktop_file_path
    else:
        return None
