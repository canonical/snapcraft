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


import datetime
import os
import pathlib
import sys

import craft_application_docs
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
}

html_static_path = ["_static"]
templates_path = ["_templates"]

# Static resources
html_css_files = [
    "css/cookie-banner.css",
    "css/support-chart.css",
]

html_js_files = [
    "js/bundle.js",
    "https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js",
    "js/support-chart.js",
]

extensions = [
    "canonical_sphinx",
    "sphinx_sitemap",
    "sphinx_substitution_extensions",
    "sphinx_toolbox.collapse",
    "sphinx.ext.ifconfig",
    "sphinx.ext.intersphinx",
    "sphinxcontrib.details.directive",
    "sphinxext.rediraffe",
    "pydantic_kitbash",
]

sphinx_tabs_disable_tab_closing = True
# endregion

rst_epilog = """
.. include:: /reuse/links.txt
"""

exclude_patterns = [
    "sphinx-docs-starter-pack",
    # Excluded because Snapcraft doesn't use overlays
    "common/craft-parts/overlay_parameters.rst",
    # Excluded here because they are either included explicitly in other
    # documents (so they generate "duplicate label" errors) or they aren't
    # used in this documentation at all (so they generate "unreferenced"
    # errors).
    "common/craft-application/*",
    "common/craft-parts/explanation/dump_plugin.rst",
    "common/craft-parts/explanation/file-migration.rst",
    "common/craft-parts/explanation/gradle_plugin.rst",
    "common/craft-parts/explanation/how_parts_are_built.rst",
    "common/craft-parts/explanation/lifecycle.rst",
    "common/craft-parts/explanation/overlay_parameters.rst",
    "common/craft-parts/explanation/overlays.rst",
    "common/craft-parts/explanation/parts.rst",
    "common/craft-parts/how-to/craftctl.rst",
    "common/craft-parts/how-to/override_build.rst",
    "common/craft-parts/reference/part_properties.rst",
    "common/craft-parts/reference/parts_steps.rst",
    "common/craft-parts/reference/step_execution_environment.rst",
    "common/craft-parts/reference/step_output_directories.rst",
    "common/craft-parts/reference/plugins/maven_plugin.rst",
    "common/craft-parts/reference/plugins/maven_use_plugin.rst",
    "common/craft-parts/reference/plugins/poetry_plugin.rst",
    "common/craft-parts/reference/plugins/python_plugin.rst",
    "common/craft-parts/reference/plugins/python_v2_plugin.rst",
    "common/craft-parts/reference/plugins/uv_plugin.rst",
    # Extra non-craft-parts exclusions can be added after this comment
    # Staged files for Discourse migration
    "how-to/crafting/add-a-part.rst",
    "how-to/publishing/build-snaps-remotely.rst",
    "release-notes/snapcraft-9-0.rst",
]

# region Options for extensions

# Client-side page redirects.
rediraffe_redirects = "redirects.txt"

# Sitemap configuration: https://sphinx-sitemap.readthedocs.io/
html_baseurl = os.environ.get("READTHEDOCS_CANONICAL_URL", "/")

# Builds URLs as {html_baseurl}/<page-location>
sitemap_url_scheme = "{link}"

# Exclude generated pages from the sitemap:
sitemap_excludes = [
    '404/',
    'genindex/',
    'search/',
]

# endregion

# region Automated documentation

project_dir = pathlib.Path(__file__).parents[1].resolve()
sys.path.insert(0, str(project_dir.absolute()))

def generate_cli_docs(nil):
    gen_cli_docs_path = (project_dir / "tools/docs/gen_cli_docs.py").resolve()
    os.system("%s %s" % (gen_cli_docs_path, project_dir / "docs"))

def setup(app):
    app.connect("builder-inited", generate_cli_docs)

# Setup libraries documentation snippets for use in snapcraft docs.
common_docs_path = pathlib.Path(__file__).parent / "common"
craft_application_docs_path = pathlib.Path(craft_application_docs.__file__).parent / "craft-application"
craft_parts_docs_path = pathlib.Path(craft_parts_docs.__file__).parent / "craft-parts"
(common_docs_path / "craft-application").unlink(missing_ok=True)
(common_docs_path / "craft-parts").unlink(missing_ok=True)
(common_docs_path / "craft-application").symlink_to(
    craft_application_docs_path, target_is_directory=True
)
(common_docs_path / "craft-parts").symlink_to(
    craft_parts_docs_path, target_is_directory=True
)

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
