# Copyright 2023-2025 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

import datetime
import os
import pathlib
import sys

import craft_parts_docs
import snapcraft

project = "Snapcraft"
author = "Canonical Group Ltd"

# Version string in sidebar
if os.environ.get("READTHEDOCS_VERSION_TYPE", "external") == "external":  # PR or local build
    # Because of Autotools, we can safely assume the version starts with `n.n`
    major, minor, *_ = snapcraft.__version__.split(".")
    release = f"{major}.{minor}"
else:  # Branch build
    rtd_version = os.environ.get("READTHEDOCS_VERSION", "latest")
    release = "dev" if rtd_version == "latest" else rtd_version

copyright = "2015-%s, %s" % (datetime.date.today().year, author)

# region Configuration for canonical-sphinx
ogp_site_url = "https://ubuntu.com/docs/snapcraft"
ogp_site_name = project

# Project slug; see https://meta.discourse.org/t/what-is-category-slug/87897
#
# TODO: If your documentation is hosted on https://docs.ubuntu.com/,
#       uncomment and update as needed.

slug = "docs/snapcraft"

html_context = {
    "product_page": "snapcraft.io",
    "github_url": "https://github.com/canonical/snapcraft",
    "repo_default_branch": "main",
    "repo_folder": "/docs/",
    "github_issues": "enabled",
    "matrix": "https://matrix.to/#/#snapcraft:ubuntu.com",
    "discourse": "https://forum.snapcraft.io",
    "display_contributors": False,
}

# Target repository for the edit button on pages
html_theme_options = {
    "source_edit_link": "https://github.com/canonical/snapcraft",
    "announcement": "<strong>Snapcraft 7 has ended standard support</strong>. For the latest features and documentation, upgrade to <a href='https://documentation.ubuntu.com/snapcraft'>Snapcraft 8</a>.<br>Legacy Snapcraft 7 documents that aren't migrated yet are stored in the <a href='forum.snapcraft.io'>Snapcraft forum</a>.",
}

html_static_path = ["_static"]

extensions = [
    "canonical_sphinx",
    "sphinx_sitemap",
    "pydantic_kitbash",
]

sphinx_tabs_disable_tab_closing = True
# endregion

extensions.extend(
    (
        "sphinx.ext.ifconfig",
        "sphinx.ext.intersphinx",
        "sphinxcontrib.details.directive",
        "sphinx_toolbox.collapse",
        "sphinxext.rediraffe",
    )
)

exclude_patterns = [
    "sphinx-resources",
]

html_css_files = [
    "css/announcement.css",
]

html_js_files = [
    "js/overwrite-links.js",
]

# region Options for extensions

# Client-side page redirects.
rediraffe_redirects = "redirects.txt"

# Sitemap configuration: https://sphinx-sitemap.readthedocs.io/
html_baseurl = f"{ogp_site_url}/{release}/"

sitemap_url_scheme = "{link}"

# endregion

# region Automated documentation

project_dir = pathlib.Path(__file__).parents[1].resolve()
sys.path.insert(0, str(project_dir.absolute()))

# Add directories to sys path to simplify kitbash arguments
model_dir = (project_dir / "snapcraft/models").resolve()
sys.path.append(str(model_dir.absolute()))

library_dir = (project_dir / ".venv/lib/python3.12/site-packages").resolve()
sys.path.append(str(library_dir.absolute()))

# endregion
