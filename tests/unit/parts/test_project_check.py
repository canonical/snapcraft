# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2018-2022 Canonical Ltd.
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

from pathlib import Path
from textwrap import dedent

import pytest
import yaml

from snapcraft import errors
from snapcraft.parts.project_check import run_project_checks
from snapcraft.projects import Project


@pytest.fixture
def snapcraft_yaml(new_dir):
    """Generic snapcraft application project with a nil part and no apps."""
    content = dedent(
        """\
            name: project-name
            base: core22
            version: "1.0"
            summary: project checks
            description: sanity checks
            grade: stable
            confinement: strict

            parts:
              nil:
                plugin: nil
            """
    )

    return yaml.safe_load(content)


def test_no_snap_dir(emitter, snapcraft_yaml):
    project = Project.unmarshal(snapcraft_yaml)
    run_project_checks(project, assets_dir=Path("snap"))
    emitter.assert_interactions([])


def test_icon(new_dir):
    content = dedent(
        """\
            name: project-name
            base: core22
            version: "1.0"
            summary: sanity checks
            description: sanity checks
            grade: stable
            confinement: strict
            icon: foo.png

            parts:
              nil:
                plugin: nil
            """
    )

    yaml_data = yaml.safe_load(content)
    project = Project.unmarshal(yaml_data)

    # Test without icon raises error
    with pytest.raises(errors.SnapcraftError) as raised:
        run_project_checks(project, assets_dir=Path("snap"))

    assert str(raised.value) == "Specified icon 'foo.png' does not exist."

    # Test with icon passes.
    (new_dir / "foo.png").touch()
    run_project_checks(project, assets_dir=Path("snap"))


def test_accepted_artifacts(new_dir, emitter, snapcraft_yaml):
    project = Project.unmarshal(snapcraft_yaml)
    assets_dir = Path("snap")

    file_assets = [
        ".snapcraft/state",
        "gui/icon.png",
        "gui/other-icon.png",
        "plugins/plugin1.py",
        "plugins/data-file",
        "hooks/configure",
        "hooks/new-hook",
        "keys/key1.asc",
        "keys/key2.asc",
        "local/file",
        "local/dir/file",
    ]

    for file_asset in file_assets:
        asset_path = assets_dir / file_asset
        asset_path.parent.mkdir(parents=True, exist_ok=True)
        asset_path.touch()

    run_project_checks(project, assets_dir=Path("snap"))

    assert emitter.interactions == []


def test_unexpected_things(new_dir, emitter, snapcraft_yaml):
    project = Project.unmarshal(snapcraft_yaml)
    assets_dir = Path("snap")

    file_assets = [
        "dir1/foo",
        "dir1/keys/key1.asc",
        "dir1/keys/key2.asc",
        "dir1/local/dir/file",
        "dir1/local/file",
        "dir1/plugins/data-file",
        "dir1/plugins/plugin1.py",
        "dir2/foo",
        "dir2/hooks/configure",
        "dir2/hooks/new-hook",
        "gui/icon.jpg",
    ]

    for file_asset in file_assets:
        asset_path = assets_dir / file_asset
        asset_path.parent.mkdir(parents=True, exist_ok=True)
        asset_path.touch()

    run_project_checks(project, assets_dir=Path("snap"))
    assert emitter.assert_message(
        "The 'snap' directory is meant specifically for snapcraft, but it contains\n"
        "the following non-snapcraft-related paths:\n"
        "- dir1\n"
        "- dir1/foo\n"
        "- dir1/keys\n"
        "- dir1/keys/key1.asc\n"
        "- dir1/keys/key2.asc\n"
        "- dir1/local\n"
        "- dir1/local/dir\n"
        "- dir1/local/dir/file\n"
        "- dir1/local/file\n"
        "- dir1/plugins\n"
        "- dir1/plugins/data-file\n"
        "- dir1/plugins/plugin1.py\n"
        "- dir2\n"
        "- dir2/foo\n"
        "- dir2/hooks\n"
        "- dir2/hooks/configure\n"
        "- dir2/hooks/new-hook\n"
        "- gui/icon.jpg\n"
        "\n"
        "This is unsupported and may cause unexpected behavior. If you must store\n"
        "these files within the 'snap' directory, move them to 'snap/local'\n"
        "which is ignored by snapcraft.",
        intermediate=True,
    )
