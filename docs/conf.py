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

project_dir = pathlib.Path("..").resolve()
sys.path.insert(0, str(project_dir.absolute()))

project = "Snapcraft"
author = "Canonical Group Ltd"
copyright = "%s, %s" % (datetime.date.today().year, author)
release = snapcraft.__version__

# region Configuration for canonical-sphinx
ogp_site_url = "https://canonical-snapcraft.readthedocs-hosted.com/"
ogp_site_name = project

html_context = {
    "product_page": "snapcraft.io",
    "discourse": "https://forum.snapcraft.io",
    "matrix": "https://matrix.to/#/#snapcraft:ubuntu.com",
    "github_url": "https://github.com/canonical/snapcraft",
    "display_contributors": False,
}

extensions = [
    "canonical_sphinx",
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

rst_epilog = """
.. include:: /reuse/links.txt
"""

exclude_patterns = [
    "sphinx-resources",
    # Excluded because Snapcraft doesn't use overlays
    "common/craft-parts/overlay_parameters.rst",
    # Excluded here because they are either included explicitly in other
    # documents (so they generate "duplicate label" errors) or they aren't
    # used in this documentation at all (so they generate "unreferenced"
    # errors).
    "common/craft-parts/explanation/lifecycle.rst",
    "common/craft-parts/explanation/overlay_parameters.rst",
    "common/craft-parts/explanation/overlays.rst",
    "common/craft-parts/explanation/how_parts_are_built.rst",
    "common/craft-parts/explanation/parts.rst",
    "common/craft-parts/how-to/craftctl.rst",
    "common/craft-parts/reference/parts_steps.rst",
    "common/craft-parts/reference/step_execution_environment.rst",
    "common/craft-parts/reference/step_output_directories.rst",
    "common/craft-parts/reference/plugins/poetry_plugin.rst",
    "common/craft-parts/reference/plugins/python_plugin.rst",
    "common/craft-parts/reference/plugins/maven_plugin.rst",
    "common/craft-parts/reference/plugins/uv_plugin.rst",
    # Extra non-craft-parts exclusions can be added after this comment
    # Staged files for Discourse migration
    "tutorials/create-a-snap.rst",
    "how-to/craft/add-a-part.rst",
    "how-to/craft/define-a-snaps-metadata.rst",
    "how-to/craft/manage-dependencies.rst",
    "how-to/craft/manage-files.rst",
    "how-to/craft/override-the-build-step-with-craftctl.rst",
    "how-to/craft/use-extensions.rst",
    "how-to/change-bases/index.rst",
    "how-to/change-bases/change-from-core20-to-core22.rst",
    "how-to/change-bases/change-from-core22-to-core24.rst",
    "how-to/debug/debug-a-snap.rst",
    "how-to/debug/debug-with-gdb.rst",
    "how-to/debug/use-the-classic-linter.rst",
    "how-to/debug/use-the-library-linter.rst",
    "how-to/publish/build-snaps-remotely.rst",
    "how-to/publish/gather-snap-metrics-and-statistics.rst",
    "how-to/publish/manage-releases.rst",
    "how-to/publish/manage-snaps-on-the-snap-store.rst",
    "how-to/publish/publish-a-snap.rst",
    "how-to/publish/publish-a-snap-with-ci.rst",
    "reference/advanced-grammar.rst",
    "reference/build-environment-options.rst",
    "reference/hooks.rst",
    "reference/package-repositories.rst",
    "reference/part-environment-variables.rst",
    "reference/project-file.rst",
    "reference/snapcraft-yaml.rst",
    "explanation/classic-confinement.rst",
    "explanation/interfaces.rst",
]


intersphinx_mapping = {
    "craft-parts": ("https://canonical-craft-parts.readthedocs-hosted.com/en/latest/", None),
}


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

# Client-side page redirects.
rediraffe_redirects = "redirects.txt"
