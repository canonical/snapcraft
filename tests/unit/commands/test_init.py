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
from pathlib import Path
from textwrap import dedent

import pytest

from snapcraft import application
from snapcraft.models.project import Project
from snapcraft.parts.yaml_utils import _SNAP_PROJECT_FILES, apply_yaml, process_yaml


@pytest.mark.parametrize("profile", [None, "simple"])
@pytest.mark.parametrize("name", [None, "test-snap-name"])
@pytest.mark.parametrize("project_dir", [None, "test-project-dir"])
def test_init_default(profile, name, project_dir, emitter, new_dir, mocker):
    """Test the 'snapcraft init' command."""
    cmd = ["snapcraft", "init"]
    if profile:
        cmd.extend(["--profile", profile])
    if name:
        cmd.extend(["--name", name])
    if project_dir:
        cmd.append(project_dir)
        snapcraft_yaml = pathlib.Path(project_dir) / "snap/snapcraft.yaml"
    else:
        snapcraft_yaml = Path("snap/snapcraft.yaml")
    mocker.patch.object(sys, "argv", cmd)
    app = application.create_app()

    app.run()

    assert snapcraft_yaml.exists()
    # unmarshal the snapcraft.yaml to verify its contents
    data = apply_yaml(process_yaml(snapcraft_yaml), "amd64", "amd64")
    project = Project.unmarshal(data)
    assert project == Project.unmarshal(
        {
            "name": "my-snap-name",
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
    if name:
        emitter.assert_progress(
            "Ignoring '--name' parameter because it is not supported yet.",
            permanent=True,
        )
    emitter.assert_progress("Checking for an existing 'snapcraft.yaml'.")
    emitter.assert_progress("Could not find an existing 'snapcraft.yaml'.")
    emitter.assert_message("Successfully initialised project.")
    emitter.assert_message(
        "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
        "information about the snapcraft.yaml format."
    )


@pytest.mark.parametrize("profile", [None, "simple"])
@pytest.mark.parametrize("name", [None, "test-snap-name"])
@pytest.mark.parametrize("project_dir", [None, "test-project-dir"])
def test_init_snap_dir_exists(profile, name, project_dir, emitter, new_dir, mocker):
    """'snapcraft init' should work even if the 'snap/' directory already exists."""
    cmd = ["snapcraft", "init"]
    if profile:
        cmd.extend(["--profile", profile])
    if name:
        cmd.extend(["--name", name])
    if project_dir:
        cmd.append(project_dir)
        snapcraft_yaml = pathlib.Path(project_dir) / "snap/snapcraft.yaml"
    else:
        snapcraft_yaml = Path("snap/snapcraft.yaml")
    snapcraft_yaml.parent.mkdir(parents=True)
    mocker.patch.object(sys, "argv", cmd)
    app = application.create_app()

    app.run()

    assert snapcraft_yaml.exists()
    emitter.assert_message("Successfully initialised project.")
    emitter.assert_message(
        "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
        "information about the snapcraft.yaml format."
    )


@pytest.mark.parametrize(
    "snapcraft_yaml", [project.project_file for project in _SNAP_PROJECT_FILES]
)
@pytest.mark.parametrize("profile", [None, "simple"])
@pytest.mark.parametrize("name", [None, "test-snap-name"])
@pytest.mark.parametrize("project_dir", [None, "test-project-dir"])
def test_init_exists(
    profile, name, project_dir, capsys, emitter, new_dir, snapcraft_yaml, mocker
):
    """Raise an error if a snapcraft.yaml file already exists."""
    cmd = ["snapcraft", "init"]
    if profile:
        cmd.extend(["--profile", profile])
    if name:
        cmd.extend(["--name", name])
    if project_dir:
        cmd.append(project_dir)
        snapcraft_yaml_path = pathlib.Path(project_dir) / snapcraft_yaml
    else:
        snapcraft_yaml_path = snapcraft_yaml
    mocker.patch.object(sys, "argv", cmd)
    snapcraft_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    snapcraft_yaml_path.touch()
    app = application.create_app()

    app.run()

    out, err = capsys.readouterr()
    assert not out
    assert (
        "could not initialise a new snapcraft project because "
        f"{str(snapcraft_yaml)!r} already exists"
    ) in err
    emitter.assert_progress("Checking for an existing 'snapcraft.yaml'.")
