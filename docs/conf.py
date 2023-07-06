import datetime
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path("..").absolute()))

import snapcraft

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Snapcraft'
author = 'Canonical Group Ltd'
copyright = "%s, %s" % (datetime.date.today().year, author)
release = snapcraft.__version__

# Open Graph configuration - defines what is displayed in the website preview
ogp_site_url = "https://canonical-snapcraft.readthedocs-hosted.com"
ogp_site_name = project
ogp_image = "https://assets.ubuntu.com/v1/253da317-image-document-ubuntudocs.svg"

# Update with the favicon for your product
html_favicon = ".sphinx/_static/favicon.png"

html_context = {
    # Change to the discourse instance you want to be able to link to
    # (use an empty value if you don't want to link)
    "discourse": "https://forum.snapcraft.io",
    # Change to the GitHub info for your project
    "github_url": "https://github.com/canonical/snapcraft",
    # Change to the branch for this version of the documentation
    "github_version": "main",
    # Change to the folder that contains the documentation (usually "/" or "/docs/")
    "github_folder": "/docs/",
    # Change to an empty value if your GitHub repo doesn't have issues enabled
    "github_issues": "enabled"
}

# Used for related links - no need to change
if 'discourse' in html_context:
    html_context['discourse_prefix'] = html_context['discourse'] + "/t/"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_design',
    'sphinx_tabs.tabs',
    'sphinx_reredirects',
    'youtube-links',
    'related-links',
    'custom-rst-roles',
    'terminal-output',
    'sphinx_copybutton',
    'sphinxext.opengraph',
    'myst_parser'
    ]

myst_enable_extensions = [
    "substitution",
    "deflist"
]

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', '.sphinx']

rst_epilog = """
.. include:: /reuse/links.txt
"""

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# Links to ignore when checking links
linkcheck_ignore = [
    'http://127.0.0.1:8000'
    ]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# Find the current builder
builder = "dirhtml"
if '-b' in sys.argv:
    builder = sys.argv[sys.argv.index('-b')+1]

# Setting templates_path for epub makes the build fail
if builder == "dirhtml" or builder == "html":
    templates_path = [".sphinx/_templates"]

html_theme = 'furo'
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
        "color-version-popup": "#772953"
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
        "color-version-popup": "#F29879"
    },
}

html_static_path = ['.sphinx/_static']
html_css_files = [
    'custom.css',
    'github_issue_links.css',
]

html_js_files = []
if "github_issues" in html_context and html_context["github_issues"]:
    html_js_files.append('github_issue_links.js')



# Set up redirects (https://documatt.gitlab.io/sphinx-reredirects/usage.html)
# For example: "explanation/old-name.html": "../how-to/prettify.html",
redirects = {}
