import datetime
import logging
import os
import pathlib
import sys

import craft_parts_docs

project_dir = pathlib.Path("..").resolve()
sys.path.insert(0, str(project_dir.absolute()))

import snapcraft

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Snapcraft"
author = "Canonical Group Ltd"
copyright = "%s, %s" % (datetime.date.today().year, author)
release = snapcraft.__version__

# Open Graph configuration - defines what is displayed in the website preview
ogp_site_url = "https://canonical-snapcraft.readthedocs-hosted.com"
ogp_site_name = project
ogp_image = "https://assets.ubuntu.com/v1/253da317-image-document-ubuntudocs.svg"

# Update with the favicon for your product
html_favicon = "sphinx-resources/.sphinx/_static/favicon.png"

html_context = {
    "discourse": "https://forum.snapcraft.io",
    "github_url": "https://github.com/snapcore/snapcraft",
    "github_version": "main",
    "github_folder": "/docs/",
    "github_issues": "enabled",
}

# Used for related links - no need to change
if "discourse" in html_context:
    html_context["discourse_prefix"] = html_context["discourse"] + "/t/"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx_design",
    "sphinx_tabs.tabs",
    "sphinx_reredirects",
    "youtube-links",
    "related-links",
    "custom-rst-roles",
    "terminal-output",
    "sphinx_copybutton",
    "sphinx.ext.autodoc",  # Must be loaded after more_autodoc
    "sphinxext.opengraph",
    "myst_parser",
]

myst_enable_extensions = ["substitution", "deflist"]

exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "sphinx-resources",
]

rst_epilog = """
.. include:: /reuse/links.txt
"""

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# Links to ignore when checking links
linkcheck_ignore = ["http://127.0.0.1:8000"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# Find the current builder
builder = "dirhtml"
if "-b" in sys.argv:
    builder = sys.argv[sys.argv.index("-b") + 1]

# Setting templates_path for epub makes the build fail
if builder == "dirhtml" or builder == "html":
    templates_path = ["sphinx-resources/.sphinx/_templates"]

html_theme = "furo"
html_last_updated_fmt = ""
html_permalinks_icon = "¶"
html_theme_options = {
    "light_css_variables": {
        "color-sidebar-background-border": "none",
        "font-stack": "Ubuntu, -apple-system, Segoe UI, Roboto, Oxygen, Cantarell, Fira Sans, Droid Sans, Helvetica Neue, sans-serif",
        "font-stack--monospace": "Ubuntu Mono, Consolas, Monaco, Courier, monospace",
        "color-foreground-primary": "#111",
        "color-foreground-secondary": "var(--color-foreground-primary)",
        "color-foreground-muted": "#333",
        "color-background-secondary": "#FFF",
        "color-background-hover": "#f2f2f2",
        "color-brand-primary": "#111",
        "color-brand-content": "#06C",
        "color-api-background": "#cdcdcd",
        "color-inline-code-background": "rgba(0,0,0,.03)",
        "color-sidebar-link-text": "#111",
        "color-sidebar-item-background--current": "#ebebeb",
        "color-sidebar-item-background--hover": "#f2f2f2",
        "toc-font-size": "var(--font-size--small)",
        "color-admonition-title-background--note": "var(--color-background-primary)",
        "color-admonition-title-background--tip": "var(--color-background-primary)",
        "color-admonition-title-background--important": "var(--color-background-primary)",
        "color-admonition-title-background--caution": "var(--color-background-primary)",
        "color-admonition-title--note": "#24598F",
        "color-admonition-title--tip": "#24598F",
        "color-admonition-title--important": "#C7162B",
        "color-admonition-title--caution": "#F99B11",
        "color-highlighted-background": "#EbEbEb",
        "color-link-underline": "var(--color-background-primary)",
        "color-link-underline--hover": "var(--color-background-primary)",
        "color-version-popup": "#772953",
    },
    "dark_css_variables": {
        "color-foreground-secondary": "var(--color-foreground-primary)",
        "color-foreground-muted": "#CDCDCD",
        "color-background-secondary": "var(--color-background-primary)",
        "color-background-hover": "#666",
        "color-brand-primary": "#fff",
        "color-brand-content": "#06C",
        "color-sidebar-link-text": "#f7f7f7",
        "color-sidebar-item-background--current": "#666",
        "color-sidebar-item-background--hover": "#333",
        "color-admonition-background": "transparent",
        "color-admonition-title-background--note": "var(--color-background-primary)",
        "color-admonition-title-background--tip": "var(--color-background-primary)",
        "color-admonition-title-background--important": "var(--color-background-primary)",
        "color-admonition-title-background--caution": "var(--color-background-primary)",
        "color-admonition-title--note": "#24598F",
        "color-admonition-title--tip": "#24598F",
        "color-admonition-title--important": "#C7162B",
        "color-admonition-title--caution": "#F99B11",
        "color-highlighted-background": "#666",
        "color-link-underline": "var(--color-background-primary)",
        "color-link-underline--hover": "var(--color-background-primary)",
        "color-version-popup": "#F29879",
    },
}

html_static_path = ["sphinx-resources/.sphinx/_static"]
html_css_files = [
    "custom.css",
    "github_issue_links.css",
]

html_js_files = []
if "github_issues" in html_context and html_context["github_issues"]:
    html_js_files.append("github_issue_links.js")


# Set up redirects (https://documatt.gitlab.io/sphinx-reredirects/usage.html)
# For example: "explanation/old-name.html": "../how-to/prettify.html",
# redirects = {}

extensions.extend(
    (
        "sphinx.ext.ifconfig",
        "sphinxcontrib.details.directive",
    )
)

exclude_patterns.extend(
    (
        # Excluded because Snapcraft doesn't use overlays
        "common/craft-parts/overlay_parameters.rst",
        # Excluded here because they are either included explicitly in other
        # documents (so they generate "duplicate label" errors) or they aren't
        # used in this documentation at all (so they generate "unreferenced"
        # errors).
        "common/craft-parts/explanation/overlay_parameters.rst",
        "common/craft-parts/explanation/overlays.rst",
        "common/craft-parts/how-to/craftctl.rst",
        "common/craft-parts/reference/parts_steps.rst",
        "common/craft-parts/reference/step_execution_environment.rst",
        "common/craft-parts/reference/step_output_directories.rst",
        "common/craft-parts/reference/plugins/python_plugin.rst",
        # Extra non-craft-parts exclusions can be added after this comment
    )
)


def generate_cli_docs(nil):
    gen_cli_docs_path = (project_dir / "tools" / "docs" / "gen_cli_docs.py").resolve()
    os.system("%s %s" % (gen_cli_docs_path, project_dir / "docs"))


def setup(app):
    app.connect("builder-inited", generate_cli_docs)


# Setup libraries documentation snippets for use in snapcraft docs.
common_docs_path = pathlib.Path(__file__).parent / "common"
craft_parts_docs_path = pathlib.Path(craft_parts_docs.__file__).parent / "craft-parts"
(common_docs_path / "craft-parts").unlink(missing_ok=True)
(common_docs_path / "craft-parts").symlink_to(
    craft_parts_docs_path, target_is_directory=True
)
