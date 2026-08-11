import datetime
import os
import textwrap

# Configuration for the Sphinx documentation builder.
# All configuration specific to your project should be done in this file.
#
# If you're new to Sphinx and don't want any advanced or custom features,
# just go through the items marked 'TODO'.
#
# A complete list of built-in Sphinx configuration values:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
#
# The Sphinx Stack uses the Canonical Sphinx theme to keep all documentation consistent
# and on brand:
# https://github.com/canonical/canonical-sphinx


#######################
# Project information #
#######################

# Project name
# TODO: Update with the official name of your project or product (e.g., "Ubuntu Server")
project = "Starbase"

# Author name; used in the default copyright statement in the page footer
author = "Canonical Ltd."

# Format the product name and version for the TOC and HTML title
# TODO: When the product begins versioning, uncomment this block.
# release = <starcraft>.__version__
# if ".post" in release:
#     release = "dev"
# else:
#     major, minor, *_ = release.split(".")
#     release = f"{major}.{minor}"

# The year in the copyright statement
copyright = f"2023-{datetime.date.today().year}"

# Sidebar documentation title
# To disable the title, set it to an empty string.
html_title = project + " documentation"

# Documentation website URL
ogp_site_url = "https://canonical-starbase.readthedocs-hosted.com/"

# Preview name of the documentation website
# TODO: To use a different name for the project in previews, update the next line.
ogp_site_name = project

# Preview image URL
# TODO: To customise the preview image, update the next line.
ogp_image = "https://assets.ubuntu.com/v1/cc828679-docs_illustration.svg"

# Product favicon; shown in bookmarks, browser tabs, etc.
# TODO: To customise the favicon, uncomment and update the next line.
# html_favicon = ".sphinx/_static/favicon.png"

# Dictionary of values to pass into the Sphinx context for all pages:
# https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-html_context
html_context = {
    # Product page URL; can be different from product docs URL
    # TODO: Change to your product website URL, dropping the 'https://' prefix (e.g.,
    #       'ubuntu.com/lxd'). If there's no such website, remove the {{ product_page }}
    #       link from the _templates/header.html file.
    "product_page": "github.com/canonical/starbase",
    # Product tag image; the orange part of your logo, shown in the page header
    # "product_tag": "_static/tag.png",
    # Your Discourse instance URL
    # TODO: Change to your Discourse instance URL or leave empty.
    "discourse": "",
    # Your Mattermost channel URL
    # TODO: Change to your Mattermost channel URL or leave empty.
    "mattermost": "https://chat.canonical.com/canonical/channels/documentation",
    # Your Matrix channel URL
    # TODO: Change to your Matrix channel URL or leave empty.
    "matrix": "https://matrix.to/#/#starcraft-development:ubuntu.com",
    # Your documentation GitHub repository URL. If set, links for viewing the
    # documentation source files and creating GitHub issues are added at the bottom of
    # each page.
    # TODO: Change to your documentation GitHub repository URL or leave empty.
    "github_url": "https://github.com/canonical/starbase",
    # Docs branch in the repo; used in links for viewing the source files
    "repo_default_branch": "main",
    # Docs location in the repo; used in links for viewing the source files
    "repo_folder": "/docs/",
    # List contributors on individual pages
    "display_contributors": False,
    # Required for feedback button
    "github_issues": "enabled",
    # Passes the top-level 'author' value to the theme
    "author": author,
    # Documentation license information
    "license": {
        # TODO: Specify your project's license.
        # For the name, we recommend using the standard shorthand identifier from
        # https://spdx.org/licenses
        "name": "LGPL-3.0",
        # TODO: Link directly to your project's license statement.
        "url": "https://github.com/canonical/starbase/blob/main/LICENSE",
    },
}

# TODO: To enable the edit button on pages, change the link to your public repository on
# GitHub or Launchpad.
html_theme_options = {
  "source_edit_link": "https://github.com/canonical/starbase",
}

# The project slug passed to the sphinx-notfound-page extension
# TODO: Set this to the project's RTD slug
slug = "starbase"


#########################
# Sitemap configuration #
#########################

# Use RTD canonical URL to ensure duplicate pages have a specific canonical URL
html_baseurl = os.environ.get("READTHEDOCS_CANONICAL_URL", "/")

# sphinx-sitemap uses html_baseurl to generate the full URL for each page:
sitemap_url_scheme = "{link}"

# Include `lastmod` dates in the sitemap:
# sitemap_show_lastmod = True

# TODO: Exclude pages that aren't user-facing from the sitemap (e.g., module pages
# generated by autodoc).
# Pages excluded from the sitemap:
sitemap_excludes = [
    "404/",
    "genindex/",
    "search/",
]


################################
# Template and asset locations #
################################

# html_static_path = ["_static"]
# templates_path = ["_templates"]


#############
# Redirects #
#############

# Add redirects to the 'redirects.txt' file
# https://sphinxext-rediraffe.readthedocs.io/en/latest/

# To set up redirects in the Read the Docs project dashboard:
# https://docs.readthedocs.io/en/stable/guides/redirects.html

rediraffe_redirects = "redirects.txt"

# Strips '/index.html' from destination URLs when building with 'dirhtml'
rediraffe_dir_only = True

############################
# sphinx-llm configuration #
############################

# This description is included in llms.txt to provide some initial context for your
# product docs.
# TODO: Add a description in the form "This is the documentation for <product name>,
# <first sentence of home page>".
llms_txt_description = textwrap.dedent(
    """\
    This is the documentation for Starbase, a template repository for setting up
    and maintaining Starcraft projects.
    """
)

# The base URL for references built by sphinx-markdown-builder.
if os.environ.get("READTHEDOCS"):
    markdown_http_base = html_baseurl

###########################
# Link checker exceptions #
###########################

# Whole sites and individuals URLs to ignore
linkcheck_ignore = [
    # Entire domains to ignore due to flakiness or issues
    r"^https://github.com",
    r"^https://www.gnu.org/",
    r"^https://crates.io/",
    r"^https://([\w-]*\.)?npmjs.org",
    r"^https://rsync.samba.org",
    r"^https://ubuntu.com",
    r"^https://matrix.to/#",
    r"^https://gitlab.gnome.org",
]

# Anchor strings to ignore
# linkcheck_anchors_ignore = []

# Give linkcheck multiple tries on failure
linkcheck_retries = 20

# Report timeouts as 'timeout' instead of 'broken'
linkcheck_report_timeouts_as_broken = False


########################
# Configuration extras #
########################

# Custom Sphinx extensions; see
# https://www.sphinx-doc.org/en/master/usage/extensions/index.html
extensions = [
    "canonical_sphinx",
    "notfound.extension",
    "sphinx_design",
    "sphinx_rerediraffe",
    # "sphinx_tabs.tabs",
    # "sphinxcontrib.jquery"
    "sphinxext.opengraph",
    # "sphinx_config_options",
    # "sphinx_contributor_listing",
    # "sphinx_filtered_toctree",
    "sphinx_llm.txt",
    "sphinx_related_links",
    "sphinx_roles",
    "sphinx_terminal",
    # "sphinx_ubuntu_images",
    # "sphinx_youtube_links",
    # "sphinxcontrib.cairosvgconverter",
    # "sphinx_last_updated_by_git",
    "sphinx.ext.intersphinx",
    "sphinx_sitemap",
    # Custom Craft extensions
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.viewcode",
]

# Excludes files or directories from processing
exclude_patterns = [
    "README.md",  # Docs README
    "reuse",
]

# Adds custom CSS files, located remotely or in 'html_static_path'.
# TODO: Uncomment to enable Google Analytics on public projects.
# html_css_files = [
#     "https://assets.ubuntu.com/v1/d86746ef-cookie_banner.css",
# ]

# Adds custom JavaScript files, located remotely or in 'html_static_path'.
# TODO: Uncomment to enable Google Analytics on public projects.
# html_js_files = [
#     "https://assets.ubuntu.com/v1/287a5e8f-bundle.js",
# ]

# Appends extra markup to the end of every document written in reST
# rst_epilog = """
# """

# Feedback button at the top; enabled by default
# disable_feedback_button = True

# Your manpage URL
# manpages_url = "https://manpages.ubuntu.com/manpages/{codename}/en/" + \
#     "man{section}/{page}.{section}.html"

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

# Add configuration for intersphinx mapping
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "starflow": ("https://documentation.ubuntu.com/starflow/latest", None),
}

# Block Intersphinx from looking up external sources with internal references. In other
# words, only :external+<project>... will search in other projects.
intersphinx_disabled_reftypes = ["std:*"]


##############################
# Custom Craft configuration #
##############################

# Type hints configuration
set_type_checking_flag = True
typehints_fully_qualified = False
always_document_param_types = True
