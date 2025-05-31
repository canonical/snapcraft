# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023-2024 Canonical Ltd.
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

import pathlib
import sys
from textwrap import dedent

import pytest

from snapcraft import application
from snapcraft.models.project import Project
from snapcraft.parts.yaml_utils import apply_yaml, process_yaml


@pytest.fixture
def valid_new_dir(tmp_path, monkeypatch):
    """Change to a new temporary directory whose name is a valid snap name."""
    new_dir = tmp_path / "test-snap-name-dir"
    new_dir.mkdir()
    monkeypatch.chdir(new_dir)
    return new_dir


def _create_command(
    *,
    profile: str | None = None,
    project_dir: str | None = None,
    name: str | None = None,
):
    """Build a snapcraft init command."""
    cmd = ["snapcraft", "init"]
    if profile:
        cmd.extend(["--profile", profile])
    if project_dir:
        cmd.append(project_dir)
    if name:
        cmd.extend(["--name", name])
    return cmd


@pytest.mark.parametrize("profile", [None, "simple"])
@pytest.mark.parametrize("name", [None, "test-snap-name"])
@pytest.mark.parametrize("project_dir", [None, "test-project-dir"])
def test_init_default(profile, name, project_dir, emitter, valid_new_dir, mocker):
    """Test the 'snapcraft init' command."""
    if name:
        expected_name = name
    elif project_dir:
        expected_name = project_dir
    else:
        expected_name = str(valid_new_dir.name)
    snapcraft_yaml = pathlib.Path(project_dir or valid_new_dir) / "snap/snapcraft.yaml"
    cmd = _create_command(profile=profile, project_dir=project_dir, name=name)
    mocker.patch.object(sys, "argv", cmd)
    app = application.create_app()

    app.run()

    assert snapcraft_yaml.exists()
    # unmarshal the snapcraft.yaml to verify its contents
    data = apply_yaml(process_yaml(snapcraft_yaml), "amd64", "amd64")
    project = Project.unmarshal(data)
    assert project == Project.unmarshal(
        {
            "name": expected_name,
            "base": "core24",
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
            "platforms": {"amd64": {"build-on": "amd64", "build-for": "amd64"}},
        }
    )
    emitter.assert_progress("Checking for an existing 'snapcraft.yaml'.")
    emitter.assert_debug("Could not find an existing 'snapcraft.yaml'.")
    emitter.assert_message(
        "See https://documentation.ubuntu.com/snapcraft/stable/reference/project-file "
        "for reference information about the snapcraft.yaml format."
    )
    emitter.assert_message("Successfully initialised project.")
