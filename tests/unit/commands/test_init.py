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

import sys
from pathlib import Path
from textwrap import dedent
from unittest.mock import call

import pytest

from snapcraft import cli
from snapcraft.models.project import Project
from snapcraft.parts.yaml_utils import _SNAP_PROJECT_FILES, apply_yaml, process_yaml


@pytest.fixture(autouse=True)
def mock_argv(mocker):
    return mocker.patch.object(sys, "argv", ["snapcraft", "init"])


def test_init_default(emitter, new_dir):
    """Test the 'snapcraft init' command."""
    snapcraft_yaml = Path("snap/snapcraft.yaml")

    cli.run()

    assert snapcraft_yaml.exists()
    # unmarshal the snapcraft.yaml to verify its contents
    data = apply_yaml(process_yaml(snapcraft_yaml), "amd64", "amd64")
    project = Project.unmarshal(data)
    assert project == Project.unmarshal(
        {
            "name": "my-snap-name",
            "base": "core22",
            "version": "0.1",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": dedent(
                """\
            This is my-snap's description. You have a paragraph or two to tell the
            most important story about your snap. Keep it under 100 words though,
            we live in tweetspace and your description wants to look good in the snap
            store.
            """
            ),
            "grade": "devel",
            "confinement": "devmode",
            "parts": {"my-part": {"plugin": "nil"}},
            "architectures": [{"build_on": ["amd64"], "build_for": ["amd64"]}],
        }
    )
    emitter.assert_interactions(
        [
            call("progress", "Checking for an existing 'snapcraft.yaml'."),
            call("progress", "Could not find an existing 'snapcraft.yaml'."),
            call("progress", "Creating 'snap/snapcraft.yaml'."),
            call("message", "Created 'snap/snapcraft.yaml'."),
            call(
                "message",
                "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
                "information about the snapcraft.yaml format.",
            ),
        ]
    )


def test_init_snap_dir_exists(emitter, new_dir):
    """'snapcraft init' should work even if the 'snap/' directory already exists."""
    snapcraft_yaml = Path("snap/snapcraft.yaml")
    Path("snap").mkdir()

    cli.run()

    assert snapcraft_yaml.exists()
    emitter.assert_message("Created 'snap/snapcraft.yaml'.")


@pytest.mark.parametrize(
    "snapcraft_yaml", [project.project_file for project in _SNAP_PROJECT_FILES]
)
def test_init_exists(capsys, emitter, new_dir, snapcraft_yaml):
    """Raise an error if a snapcraft.yaml file already exists."""
    snapcraft_yaml.parent.mkdir(parents=True, exist_ok=True)
    snapcraft_yaml.touch()

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert (
        "could not initialize a new snapcraft project because "
        f"{str(snapcraft_yaml)!r} already exists"
    ) in err
    emitter.assert_progress("Checking for an existing 'snapcraft.yaml'.")
