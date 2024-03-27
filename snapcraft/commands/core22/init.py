# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Snapcraft init command."""

import abc
from pathlib import Path
from textwrap import dedent

from craft_cli import BaseCommand, emit
from overrides import overrides

from snapcraft import errors
from snapcraft.parts.yaml_utils import get_snap_project

_TEMPLATE_YAML = dedent(
    """\
    name: my-snap-name # you probably want to 'snapcraft register <name>'
    base: core22 # the base snap is the execution environment for this snap
    version: '0.1' # just for humans, typically '1.2+git' or '1.3.2'
    summary: Single-line elevator pitch for your amazing snap # 79 char long summary
    description: |
      This is my-snap's description. You have a paragraph or two to tell the
      most important story about your snap. Keep it under 100 words though,
      we live in tweetspace and your description wants to look good in the snap
      store.

    grade: devel # must be 'stable' to release into candidate/stable channels
    confinement: devmode # use 'strict' once you have the right plugs and slots

    parts:
      my-part:
        # See 'snapcraft plugins'
        plugin: nil
    """
)


class InitCommand(BaseCommand, abc.ABC):
    """Initialize a snapcraft project."""

    name = "init"
    help_msg = "Initialize a snapcraft project."
    overview = "Initialize a snapcraft project in the current directory."

    @overrides
    def run(self, parsed_args):
        """Initialize a snapcraft project in the current directory.

        :raises SnapcraftError: If a snapcraft.yaml already exists.
        """
        emit.progress("Checking for an existing 'snapcraft.yaml'.")

        # if a project is found, then raise an error
        try:
            project = get_snap_project()
            raise errors.SnapcraftError(
                "could not initialize a new snapcraft project because "
                f"{str(project.project_file)!r} already exists"
            )
        # the `ProjectMissing` error means a new project can be initialized
        except errors.ProjectMissing:
            emit.progress("Could not find an existing 'snapcraft.yaml'.")

        snapcraft_yaml_path = Path("snap/snapcraft.yaml")

        emit.progress(f"Creating {str(snapcraft_yaml_path)!r}.")

        snapcraft_yaml_path.parent.mkdir(exist_ok=True)
        snapcraft_yaml_path.write_text(_TEMPLATE_YAML, encoding="utf-8")

        emit.message(f"Created {str(snapcraft_yaml_path)!r}.")
        emit.message(
            "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
            "information about the snapcraft.yaml format."
        )
