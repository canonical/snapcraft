# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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

"""Appstream metadata extractor."""

import contextlib
import operator
import os
from io import StringIO
from typing import List, Optional, cast

import lxml.etree
import validators
from craft_cli import emit
from xdg.DesktopEntry import DesktopEntry

from snapcraft import errors

from .extracted_metadata import ExtractedMetadata

_XSLT = """\
<?xml version="1.0"?>
<xsl:stylesheet version="1.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>
<xsl:strip-space elements="*"/>

<xsl:template match="@* | node()">
    <xsl:copy>
        <xsl:apply-templates select="@* | node()[not(@xml:lang)] | node()[@xml:lang = en]" />
    </xsl:copy>
</xsl:template>

<xsl:template match="comment()"/>

<xsl:template match="p">
<xsl:text>&#xA;</xsl:text>
<xsl:apply-templates />
<xsl:text>&#xA;</xsl:text>
</xsl:template>

<xsl:template match="ul">
<xsl:text>&#xA;</xsl:text>
<xsl:for-each select="li[not(@xml:lang)] | li[@xml:lang = en]">
<xsl:text>- </xsl:text>
<xsl:apply-templates />
<xsl:text>&#xA;</xsl:text>
</xsl:for-each>
</xsl:template>

<xsl:template match="ol">
<xsl:text>&#xA;</xsl:text>
<xsl:for-each select="li[not(@xml:lang)] | li[@xml:lang = en]">
<xsl:number count="li[not(@xml:lang)] | li[@xml:lang = en]" format="1. "/>
<xsl:apply-templates />
<xsl:text>&#xA;</xsl:text>
</xsl:for-each>
</xsl:template>

<xsl:template match="em">
<xsl:text>_</xsl:text>
<xsl:apply-templates />
<xsl:text>_</xsl:text>
</xsl:template>

<xsl:template match="code">
<xsl:apply-templates />
</xsl:template>

</xsl:stylesheet>
"""


def extract(relpath: str, *, workdir: str) -> Optional[ExtractedMetadata]:
    """Extract appstream metadata.

    :param file_relpath: Relative path to the file containing metadata.
    :param workdir: The part working directory where the metadata file is located.

    :return: The extracted metadata, if any.
    """
    if not relpath.endswith(".metainfo.xml") and not relpath.endswith(".appdata.xml"):
        return None

    dom = _get_transformed_dom(os.path.join(workdir, relpath))

    common_id = _get_value_from_xml_element(dom, "id")
    summary = _get_value_from_xml_element(dom, "summary")
    description = _get_value_from_xml_element(dom, "description")
    title = _get_value_from_xml_element(dom, "name")
    version = _get_latest_release_from_nodes(dom.findall("releases/release"))
    project_license = _get_value_from_xml_element(dom, "project_license")
    update_contact = _get_value_from_xml_element(dom, "update_contact")
    contact = None
    if update_contact:
        if validators.url(update_contact) or validators.email(update_contact):
            contact = [update_contact]
        else:
            emit.progress(
                f"Ignoring invalid url {update_contact!r} in update_contact from appstream metadata.",
                permanent=True,
            )

    issues = _get_urls_from_xml_element(dom.findall("url"), "bugtracker")
    donation = _get_urls_from_xml_element(dom.findall("url"), "donation")
    website = _get_urls_from_xml_element(dom.findall("url"), "homepage")
    source_code = _get_urls_from_xml_element(dom.findall("url"), "vcs-browser")

    desktop_file_paths = []
    desktop_file_ids = _get_desktop_file_ids_from_nodes(dom.findall("launchable"))
    # if there are no launchables, use the appstream id to take into
    # account the legacy appstream definitions
    if common_id and not desktop_file_ids:
        if common_id.endswith(".desktop"):
            desktop_file_ids.append(common_id)
        else:
            desktop_file_ids.append(common_id + ".desktop")

    for desktop_file_id in desktop_file_ids:
        desktop_file_path = _desktop_file_id_to_path(desktop_file_id, workdir=workdir)
        if desktop_file_path:
            desktop_file_paths.append(desktop_file_path)

    icon = _extract_icon(dom, workdir, desktop_file_paths)

    return ExtractedMetadata(
        common_id=common_id,
        title=title,
        summary=summary,
        description=description,
        version=version,
        icon=icon,
        license=project_license,
        contact=contact,
        issues=issues,
        donation=donation,
        website=website,
        source_code=source_code,
        desktop_file_paths=desktop_file_paths,
    )


def _get_transformed_dom(path: str):
    dom = _get_dom(path)
    transform = _get_xslt()
    return transform(dom)


# error: Function "lxml.etree.ElementTree" is not valid as a type
def _get_dom(path: str) -> lxml.etree.ElementTree:  # type: ignore
    try:
        return lxml.etree.parse(path)  # noqa S320
    except OSError as err:
        raise errors.SnapcraftError(str(err)) from err
    except lxml.etree.ParseError as err:
        raise errors.MetadataExtractionError(path, str(err)) from err


def _get_xslt():
    xslt = lxml.etree.parse(StringIO(_XSLT))  # noqa S320
    return lxml.etree.XSLT(xslt)


def _get_value_from_xml_element(tree, key) -> Optional[str]:
    node = tree.find(key)
    if node is not None and node.text:
        # Lines that should be empty end up with empty space after the
        # transformation. One example of this is seen for paragraphs (i.e.; <p>)
        # than hold list in then (i.e.; <ol> or <ul>) so we split all lines
        # here and strip any unwanted space.
        # TODO: Improve the XSLT to remove the need for this.
        return "\n".join([n.strip() for n in node.text.splitlines()]).strip()
    return None


def _get_urls_from_xml_element(nodes, url_type) -> Optional[List[str]]:
    urls = []  # type: List[str]
    for node in nodes:
        if (
            node is not None
            and node.attrib["type"] == url_type
            and node.text.strip() not in urls
        ):
            link = node.text.strip()
            if validators.url(link) or validators.email(link):
                urls.append(link)
            else:
                emit.progress(
                    f"Ignoring invalid url or email {link!r} in {url_type!r} from appstream metadata.",
                    permanent=True,
                )
    if urls:
        return urls
    return None


def _get_latest_release_from_nodes(nodes) -> Optional[str]:
    for node in nodes:
        if "version" in node.attrib:
            return node.attrib["version"]
    return None


def _get_desktop_file_ids_from_nodes(nodes) -> List[str]:
    desktop_file_ids = []  # type: List[str]
    for node in nodes:
        if "type" in node.attrib and node.attrib["type"] == "desktop-id":
            desktop_file_ids.append(node.text.strip())
    return desktop_file_ids


def _desktop_file_id_to_path(desktop_file_id: str, *, workdir: str) -> Optional[str]:
    # For details about desktop file ids and their corresponding paths, see
    # https://standards.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id
    for xdg_data_dir in ("usr/local/share", "usr/share"):
        desktop_file_path = os.path.join(
            xdg_data_dir, "applications", desktop_file_id.replace("-", "/")
        )
        # Check if it exists in workdir, but do not add it to the resulting path
        # as it later needs to exist in the prime directory to effectively be
        # used.
        if os.path.exists(os.path.join(workdir, desktop_file_path)):
            return desktop_file_path
    return None


def _extract_icon(dom, workdir: str, desktop_file_paths: List[str]) -> Optional[str]:
    icon_node = dom.find("icon")
    if icon_node is not None and "type" in icon_node.attrib:
        icon_node_type = icon_node.attrib["type"]
    else:
        icon_node_type = None

    icon = icon_node.text.strip() if icon_node is not None else None

    if icon_node_type == "remote":
        return icon

    if icon_node_type == "stock" and icon is not None:
        return _get_icon_from_theme(workdir, "hicolor", icon)

    # If an icon path is specified and the icon file exists, we'll use that, otherwise
    # we'll fall back to what's listed in the desktop file.
    if icon is None:
        return _get_icon_from_desktop_file(workdir, desktop_file_paths)

    if os.path.exists(os.path.join(workdir, icon.lstrip("/"))):
        return icon

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
        icon = cast(str, entry.getIcon())
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
    for icon_size_entry in sized_entries:
        with contextlib.suppress(ValueError):
            isize = int(icon_size_entry)
            sizes[isize] = f"{isize}x{isize}"

    icon_size = None
    suffixes = []
    if sizes:
        size = max(sizes.items(), key=operator.itemgetter(1))[0]
        icon_size = sizes[size]
        suffixes = [".png", ".xpm"]
    elif "scalable" in entries:
        icon_size = "scalable"
        suffixes = [".svg", ".svgz"]

    icon_path = None
    if icon_size:
        for suffix in suffixes:
            icon_path = os.path.join(theme_dir, icon_size, "apps", icon + suffix)
            if os.path.exists(os.path.join(workdir, icon_path)):
                break

    return icon_path
