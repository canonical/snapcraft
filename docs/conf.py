import datetime
import os
import pathlib
import sys

import craft_application_docs
import craft_parts_docs
import snapcraft

# Configuration for the Sphinx documentation builder.
# All configuration specific to your project should be done in this file.
#
# A complete list of built-in Sphinx configuration values:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
#
# Our starter pack uses the custom Canonical Sphinx extension
# to keep all documentation based on it consistent and on brand:
# https://github.com/canonical/canonical-sphinx


#######################
# Project information #
#######################

# Project name
project = "Snapcraft"
author = "Canonical Ltd."

# Sidebar documentation title; best kept reasonably short
# The full version, including alpha/beta/rc tags
release = snapcraft.__version__
# The commit hash in the dev release version confuses the spellchecker
if ".post" in release:
    release = "dev"

html_title = project + " documentation"

# Copyright string; shown at the bottom of the page
copyright = "2015-%s, %s" % (datetime.date.today().year, author)


# Documentation website URL
ogp_site_url = "https://documentation.ubuntu.com/snapcraft"

# Preview name of the documentation website
ogp_site_name = project

# Preview image URL
ogp_image = "https://assets.ubuntu.com/v1/cc828679-docs_illustration.svg"

# Product favicon; shown in bookmarks, browser tabs, etc.
# html_favicon = '.sphinx/_static/favicon.png'

# Dictionary of values to pass into the Sphinx context for all pages:
# https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-html_context
html_context = {
    "product_page": "snapcraft.io",
    "discourse": "https://forum.snapcraft.io",
    "matrix": "https://matrix.to/#/#snapcraft:ubuntu.com",
    "github_url": "https://github.com/canonical/snapcraft",
    "repo_default_branch": "main",
    "repo_folder": "/docs/",
    "display_contributors": False,
    'github_issues': 'enabled',
}

#html_extra_path = []

html_theme_options = {
    "source_edit_link": "https://github.com/canonical/snapcraft",
}

# Project slug; see https://meta.discourse.org/t/what-is-category-slug/87897
# slug = ''


#######################
# Sitemap configuration: https://sphinx-sitemap.readthedocs.io/
#######################

# Use RTD canonical URL to ensure duplicate pages have a specific canonical URL
html_baseurl = os.environ.get("READTHEDOCS_CANONICAL_URL", "/")

# sphinx-sitemap uses html_baseurl to generate the full URL for each page:
sitemap_url_scheme = '{link}'

# Include `lastmod` dates in the sitemap:
# sitemap_show_lastmod = True

# Exclude generated pages from the sitemap:
sitemap_excludes = [
    '404/',
    'genindex/',
    'search/',
]


#######################
# Template and asset locations
#######################

html_static_path = ["_static"]
templates_path = ["_templates"]


#############
# Redirects #
#############

rediraffe_redirects = "redirects.txt"


###########################
# Link checker exceptions #
###########################

# A regex list of URLs that are ignored by 'make linkcheck'
linkcheck_anchors_ignore = [
    "#",
    ":",
    r"https://github\.com/.*",
]
linkcheck_ignore = [
    # GitHub aggressively rate limits us
    r"^https://github.com/",
    # Entire domains to ignore due to flakiness or issues
    r"^https://www.gnu.org/",
    r"^https://crates.io/",
    r"^https://([\w-]*\.)?npmjs.org",
    r"^https://rsync.samba.org",
    r"^https://ubuntu.com",
    r"^https://www.freedesktop.org/",
    r"^https://www.npmjs.com/",
    "https://matrix.to/#",
]

# give linkcheck multiple tries on failure
linkcheck_retries = 20


########################
# Configuration extras #
########################

# Custom Sphinx extensions; see
# https://www.sphinx-doc.org/en/master/usage/extensions/index.html

extensions = [
    "canonical_sphinx",
    "notfound.extension",
    "sphinx_design",
    # "sphinx_tabs.tabs",
    # "sphinxcontrib.jquery"
    "sphinxext.opengraph",
    # "sphinx_config_options",
    # "sphinx_contributor_listing",
    # "sphinx_filtered_toctree",
    # "sphinx_related_links",
    "sphinx_roles",
    "sphinx_terminal",
    # "sphinx_ubuntu_images",
    # "sphinx_youtube_links",
    # "sphinxcontrib.cairosvgconverter",
    # "sphinx_last_updated_by_git",
    "sphinx.ext.intersphinx",
    "sphinx_sitemap",
    # Custom Craft extensions
    "pydantic_kitbash",
    "sphinx-pydantic",
    "sphinxext.rediraffe",
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.ifconfig",
    "sphinx.ext.viewcode",
    "sphinx_substitution_extensions",
]

# Excludes files or directories from processing
exclude_patterns = [
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

# Adds custom CSS files, located under 'html_static_path'
html_css_files = [
    "css/cookie-banner.css",
    "css/support-chart.css",
]


# Adds custom JavaScript files, located under 'html_static_path'
html_js_files = [
    "js/bundle.js",
    "https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js",
    "js/support-chart.js",
]


# Specifies a reST snippet to be appended to each .rst file
rst_epilog = """
.. include:: /reuse/links.txt
"""

# Feedback button at the top; enabled by default
# disable_feedback_button = True

# Your manpage URL
# manpages_url = 'https://manpages.ubuntu.com/manpages/{codename}/en/' + \
#     'man{section}/{page}.{section}.html'

# Specifies a reST snippet to be prepended to each .rst file
# This defines a :center: role that centers table cell content.
# This defines a :h2: role that styles content for use with PDF generation.
rst_prolog = """
.. role:: center
   :class: align-center
.. role:: h2
    :class: hclass2
.. role:: woke-ignore
    :class: woke-ignore
.. role:: vale-ignore
    :class: vale-ignore
"""

# Workaround for https://github.com/canonical/canonical-sphinx/issues/34
if "discourse_prefix" not in html_context and "discourse" in html_context:
    html_context["discourse_prefix"] = f"{html_context['discourse']}/t/"

# Add configuration for intersphinx mapping
intersphinx_mapping = {
    # https://github.com/canonical/snapcraft/issues/6036
    # "snap": ("https://snapcraft.io/docs/", None),
    "charmcraft": ("https://documentation.ubuntu.com/charmcraft/stable/", None),
    "rockcraft": ("https://documentation.ubuntu.com/rockcraft/stable/", None)
}


##############################
# Custom Craft configuration #
##############################

# Type hints configuration
set_type_checking_flag = True
typehints_fully_qualified = False
always_document_param_types = True

# Automated documentation
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
