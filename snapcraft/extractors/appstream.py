# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2019 Canonical Ltd
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

import contextlib
import operator
import os
from io import StringIO
from typing import List, Optional

import lxml.etree

from xdg.DesktopEntry import DesktopEntry
from ._metadata import ExtractedMetadata
from snapcraft.extractors import _errors


_xslt = None
_XSLT = """\
<?xml version="1.0"?>
<xsl:stylesheet version="1.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>
<xsl:output omit-xml-declaration="yes" indent="yes"/>
<xsl:strip-space elements="*"/>

<xsl:template match="@* | node()">
    <xsl:copy>
        <xsl:apply-templates select="@* | node()[not(@xml:lang)] | node()[@xml:lang = en]" />
    </xsl:copy>
</xsl:template>

<xsl:template match="p" xml:space="preserve">
<xsl:value-of select="normalize-space(text())" />
    <xsl:for-each select="ul/li[not(@xml:lang)] | ul/li[@xml:lang = en]">
<xsl:text>&#xA;</xsl:text>
<xsl:text>- </xsl:text>
<xsl:value-of select="text()" />
    </xsl:for-each>
<xsl:text>&#xA;</xsl:text>
<xsl:text>&#xA;</xsl:text>
</xsl:template>

<xsl:template match="//ul">
    <xsl:for-each select="li[not(@xml:lang)] | li[@xml:lang = en]">
<xsl:text>- </xsl:text>
<xsl:value-of select="text()" />
<xsl:text>&#xA;</xsl:text>
    </xsl:for-each>
</xsl:template>

<xsl:template match="ol">
    <xsl:for-each select="li[not(@xml:lang)] | li[@xml:lang = en]">
<xsl:number count="li[not(@xml:lang)] | li[@xml:lang = en]" format="1. "/>
<xsl:value-of select="." />
<xsl:text>&#xA;</xsl:text>
    </xsl:for-each>
</xsl:template>

</xsl:stylesheet>
"""


def extract(relpath: str, *, workdir: str) -> ExtractedMetadata:
    if not relpath.endswith(".metainfo.xml") and not relpath.endswith(".appdata.xml"):
        raise _errors.UnhandledFileError(relpath, "appstream")

    dom = _get_transformed_dom(os.path.join(workdir, relpath))

    common_id = _get_value_from_xml_element(dom, "id")
    summary = _get_value_from_xml_element(dom, "summary")
    description = _get_value_from_xml_element(dom, "description")

    desktop_file_paths = []
    desktop_file_ids = _get_desktop_file_ids_from_nodes(dom.findall("launchable"))
    # if there are no launchables, use the appstream id to take into
    # account the legacy appstream definitions
    if common_id and common_id.endswith(".desktop") and not desktop_file_ids:
        desktop_file_ids.append(common_id)

    for desktop_file_id in desktop_file_ids:
        desktop_file_path = _desktop_file_id_to_path(desktop_file_id, workdir=workdir)
        if desktop_file_path:
            desktop_file_paths.append(desktop_file_path)

    icon = _extract_icon(dom, workdir, desktop_file_paths)

    return ExtractedMetadata(
        common_id=common_id,
        summary=summary,
        description=description,
        icon=icon,
        desktop_file_paths=desktop_file_paths,
    )


def _get_transformed_dom(path: str):
    dom = _get_dom(path)
    transform = _get_xslt()
    return transform(dom)


def _get_dom(path: str) -> lxml.etree.ElementTree:
    try:
        return lxml.etree.parse(path)
    except lxml.etree.ParseError as e:
        raise _errors.AppstreamFileParseError(path) from e


def _get_xslt():
    global _xslt
    if _xslt is not None:
        return _xslt

    xslt = lxml.etree.parse(StringIO(_XSLT))
    return lxml.etree.XSLT(xslt)


def _get_value_from_xml_element(tree, key) -> Optional[str]:
    node = tree.find(key)
    if node is not None and node.text:
        return node.text.strip()
    else:
        return None


def _get_desktop_file_ids_from_nodes(nodes) -> List[str]:
    desktop_file_ids = []  # type: List[str]
    for node in nodes:
        if "type" in node.attrib and node.attrib["type"] == "desktop-id":
            desktop_file_ids.append(node.text.strip())
    return desktop_file_ids


def _desktop_file_id_to_path(desktop_file_id: str, *, workdir: str) -> Optional[str]:
    # For details about desktop file ids and their corresponding paths, see
    # https://standards.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id  # noqa
    for xdg_data_dir in ("usr/local/share", "usr/share"):
        desktop_file_path = os.path.join(
            xdg_data_dir, "applications", desktop_file_id.replace("-", "/")
        )
        # Check if it exists in workdir, but do not add it to the resulting path
        # as it later needs to exist in the prime directory to effectively be
        # used.
        if os.path.exists(os.path.join(workdir, desktop_file_path)):
            return desktop_file_path
    else:
        return None


def _extract_icon(dom, workdir: str, desktop_file_paths: List[str]) -> Optional[str]:
    icon_node = dom.find("icon")
    if icon_node is not None and "type" in icon_node.attrib:
        icon_node_type = icon_node.attrib["type"]
    else:
        icon_node_type = None

    icon = icon_node.text.strip() if icon_node is not None else None

    if icon_node_type == "remote":
        # TODO Currently we only support local and stock icons.
        # See bug https://bugs.launchpad.net/snapcraft/+bug/1742348
        # for supporting remote icons.
        # --elopio -20180109
        return None
    elif icon_node_type == "stock":
        return _get_icon_from_theme(workdir, "hicolor", icon)

    # If an icon path is specified and the icon file exists, we'll use that, otherwise
    # we'll fall back to what's listed in the desktop file.
    if icon is None:
        return _get_icon_from_desktop_file(workdir, desktop_file_paths)
    elif os.path.exists(os.path.join(workdir, icon.lstrip("/"))):
        return icon
    else:
        return _get_icon_from_desktop_file(workdir, desktop_file_paths)


def _get_icon_from_desktop_file(
    workdir: str, desktop_file_paths: List[str]
) -> Optional[str]:
    # Icons in the desktop file can be either a full path to the icon file, or a name
    # to be searched in the standard locations. If the path is specified, use that,
    # otherwise look for the icon in the hicolor theme (also covers icon type="stock").
    # See https://standards.freedesktop.org/icon-theme-spec/icon-theme-spec-latest.html
    # for further information.
    for path in desktop_file_paths:
        entry = DesktopEntry()
        entry.parse(os.path.join(workdir, path))
        icon = entry.getIcon()
        icon_path = (
            icon
            if os.path.isabs(icon)
            else _get_icon_from_theme(workdir, "hicolor", icon)
        )
        return icon_path

    return None


def _get_icon_from_theme(workdir: str, theme: str, icon: str) -> Optional[str]:
    # Icon themes can carry icons in different pre-rendered sizes or scalable. Scalable
    # implementation is optional, so we'll try the largest pixmap and then scalable if
    # no other sizes are available.
    theme_dir = os.path.join("usr", "share", "icons", theme)
    if not os.path.exists(os.path.join(workdir, theme_dir)):
        return None

    # TODO: use index.theme
    entries = os.listdir(os.path.join(workdir, theme_dir))
    # size is NxN
    x_entries = (e.split("x") for e in entries if "x" in e)
    sized_entries = (e[0] for e in x_entries if e[0] == e[1])
    sizes = {}
    for icon_size in sized_entries:
        with contextlib.suppress(ValueError):
            sizes[int(icon_size)] = "{0}x{0}".format(icon_size)

    if sizes:
        size = max(sizes.items(), key=operator.itemgetter(1))[0]
        icon_size = sizes[size]
        suffixes = [".png", ".xpm"]
    elif "scalable" in entries:
        icon_size = "scalable"
        suffixes = [".svg", ".svgz"]
    else:
        icon_size = None

    icon_path = None
    if icon_size:
        for suffix in suffixes:
            icon_path = os.path.join(theme_dir, icon_size, "apps", icon + suffix)
            if os.path.exists(os.path.join(workdir, icon_path)):
                break

    return icon_path
