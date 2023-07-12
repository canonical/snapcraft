import datetime
import os
import pathlib
import sys

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

extensions = [
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx.ext.coverage",
    "sphinx.ext.doctest",
    "sphinx_design",
    "sphinx_copybutton",
    "sphinx-pydantic",
    "sphinx.ext.autodoc",  # Must be loaded after more_autodoc
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", ".sphinx"]

show_authors = False

rst_epilog = """
.. include:: /reuse/links.txt
"""

# Links to ignore when checking links
linkcheck_ignore = ["http://127.0.0.1:8000"]

html_theme = "furo"
html_static_path = ["_static"]
html_css_files = [
    "css/custom.css",
]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
}

# Type hints configuration
set_type_checking_flag = True
typehints_fully_qualified = False
always_document_param_types = True

# Github config
github_username = "snapcore"
github_repository = "snapcraft"


def generate_cli_docs(nil):
    gen_cli_docs_path = (project_dir / "tools" / "docs" / "gen_cli_docs.py").resolve()
    os.system("%s %s" % (gen_cli_docs_path, project_dir / "docs"))


def setup(app):
    app.connect("builder-inited", generate_cli_docs)
