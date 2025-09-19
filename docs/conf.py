# This file is part of starbase.
#
# Copyright 2024 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 3, as published
# by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
# SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import datetime

project = "Starbase"
author = "Canonical"

copyright = "2023-%s, %s" % (datetime.date.today().year, author)

# region Configuration for canonical-sphinx

ogp_site_url = "https://canonical-starbase.readthedocs-hosted.com/"
ogp_site_name = project
ogp_image = "https://assets.ubuntu.com/v1/253da317-image-document-ubuntudocs.svg"

html_context = {
    # The following items are required for public-facing products. Replace the
    # placeholder links with those specific to your product.
    "product_page": "github.com/canonical/starbase",
    "github_url": "https://github.com/canonical/starbase",
    "github_issues": "https://github.com/canonical/starbase/issues",
    "matrix": "https://matrix.to/#/#starcraft-development:ubuntu.com",
    "discourse": "",  # Leave this blank to hide it from the dropdown
}

# Target repository for the edit button on pages
html_theme_options = {
    "source_edit_link": "https://github.com/canonical/starbase",
}

html_static_path = ["_static"]
templates_path = ["_templates"]

# Static resources for Google Analytics
html_css_files = [
    'css/cookie-banner.css'
]

html_js_files = [
    'js/bundle.js',
]

extensions = [
    "canonical_sphinx",
]

# endregion

# region General configuration
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions.extend(
    [
        "sphinx.ext.intersphinx",
        "sphinx.ext.viewcode",
        "sphinx.ext.coverage",
        "sphinx.ext.doctest",
        "sphinx-pydantic",
        "sphinx_sitemap",
        "sphinx_toolbox",
        "sphinx_toolbox.more_autodoc",
        "sphinx.ext.autodoc",  # Must be loaded after more_autodoc
        "sphinxext.rediraffe",
    ]
)

# endregion

# region Options for extensions
# Intersphinx extension
# https://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html#configuration

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
}

# Type hints configuration
set_type_checking_flag = True
typehints_fully_qualified = False
always_document_param_types = True

# Github config
github_username = "canonical"
github_repository = "starbase"

# Client-side page redirects.
rediraffe_redirects = "redirects.txt"

# The full path to the RTD site.
# TODO: Change this to your project's RTD URL. If the RTD site isn't live yet, follow
# the pattern here. If the documentation has moved to documentation.ubuntu.com, enter
# the URL at that domain. It's OK to use this for private projects.
# https://sphinx-sitemap.readthedocs.io
html_baseurl = "https://canonical-starbase.readthedocs-hosted.com/"

# Compose the URL for remote RTD and local builds.
# TODO: If your project doesn't have a `latest` RTD branch set up, change to its default
# branch.
# https://sphinx-sitemap.readthedocs.io
if "READTHEDOCS_VERSION" in os.environ:
    version = os.environ["READTHEDOCS_VERSION"]
    sitemap_url_scheme = "{version}{link}"
else:
    sitemap_url_scheme = "latest/{link}"

# endregion

# We have many links on sites that frequently respond with 503s to GitHub runners.
# https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-linkcheck_retries
linkcheck_retries = 20
linkcheck_anchors_ignore = ["#", ":"]
linkcheck_ignore = [
    # Ignore releases, since we'll include the next release before it exists.
    r"^https://github.com/canonical/[a-z]*craft[a-z-]*/releases/.*",
    # Entire domains to ignore due to flakiness or issues
    r"^https://www.gnu.org/",
    r"^https://crates.io/",
    r"^https://([\w-]*\.)?npmjs.org",
    r"^https://rsync.samba.org",
    r"^https://ubuntu.com",
]
