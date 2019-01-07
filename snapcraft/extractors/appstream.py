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
from io import StringIO
from textwrap import dedent

import lxml.etree

from ._metadata import ExtractedMetadata
from snapcraft.extractors import _errors


_xslt = None
_XSLT = dedent(
    """\
<?xml version="1.0"?>
<xsl:stylesheet version="1.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>
<xsl:output omit-xml-declaration="yes" indent="yes"/>
<xsl:strip-space  elements="*"/>

<xsl:template match="@* | node()">
    <xsl:copy>
        <xsl:apply-templates select="@* | node()[not(@xml:lang)] | node()[@xml:lang = en]" />
    </xsl:copy>
</xsl:template>

<xsl:template match="p" xml:space="preserve">
<xsl:value-of select="." />
</xsl:template>

<xsl:template match="ul">
    <xsl:for-each select="li[not(@xml:lang)] | li[@xml:lang = en]">
- <xsl:value-of select="." />
    </xsl:for-each>
</xsl:template>

<xsl:template match="ol">
<xsl:text>&#xA;</xsl:text>
    <xsl:for-each select="li[not(@xml:lang)] | li[@xml:lang = en]">
<xsl:number count="li[not(@xml:lang)] | li[@xml:lang = en]" format="1. "/>
<xsl:value-of select="." />
<xsl:text>&#xA;</xsl:text>
    </xsl:for-each>
</xsl:template>

</xsl:stylesheet>
"""
)


def extract(path: str) -> ExtractedMetadata:
    if not path.endswith(".metainfo.xml") and not path.endswith(".appdata.xml"):
        raise _errors.UnhandledFileError(path, "appstream")

    dom = _get_transformed_dom(path)

    common_id = _get_value_from_xml_element(dom, "id")
    summary = _get_value_from_xml_element(dom, "summary")
    description = _get_value_from_xml_element(dom, "description")

    icon = None
    node = dom.find("icon")
    if node is not None and "type" in node.attrib and node.attrib["type"] == "local":
        # TODO Currently we only support local icons.
        # See bug https://bugs.launchpad.net/snapcraft/+bug/1742348
        # for supporting remote icons.
        # --elopio -20180109
        icon = node.text.strip()

    desktop_file_paths = []
    nodes = dom.findall("launchable")
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
