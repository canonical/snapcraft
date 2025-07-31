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
# The full version, including alpha/beta/rc tags
release = snapcraft.__version__
# The commit hash in the dev release version confuses the spellchecker
if ".post" in release:
    release = "dev"

copyright = "2015-%s, %s" % (datetime.date.today().year, author)

# region Configuration for canonical-sphinx
ogp_site_url = "https://documentation.ubuntu.com/snapcraft"
ogp_site_name = project

# Project slug; see https://meta.discourse.org/t/what-is-category-slug/87897
#
# TODO: If your documentation is hosted on https://docs.ubuntu.com/,
#       uncomment and update as needed.

slug = "snapcraft"

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
    "announcement": "<strong>Snapcraft 7 has ended standard support</strong>. For the latest features and documentation, upgrade to <a href='https://documentation.ubuntu.com/snapcraft'>Snapcraft 8</a>.",
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

# region Options for extensions

# Client-side page redirects.
rediraffe_redirects = "redirects.txt"

# Sitemap configuration: https://sphinx-sitemap.readthedocs.io/
html_baseurl = "https://documentation.ubuntu.com/snapcraft/"

if "READTHEDOCS_VERSION" in os.environ:
    version = os.environ["READTHEDOCS_VERSION"]
    sitemap_url_scheme = "{version}{link}"
else:
    sitemap_url_scheme = "latest/{link}"

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
