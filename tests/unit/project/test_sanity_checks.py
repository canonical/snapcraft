# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

import logging
import pathlib
from textwrap import dedent

import pytest

import snapcraft.internal.errors
from snapcraft.project import Project
from snapcraft.project._sanity_checks import conduct_project_sanity_check


@pytest.fixture
def snapcraft_yaml_path(tmp_work_path, request):
    """Generic snapcraft application project with a nil part and no apps."""
    snapcraft_yaml_path = tmp_work_path / "snap/snapcraft.yaml"
    snapcraft_yaml_path.parent.mkdir(parents=True)
    with snapcraft_yaml_path.open("w") as snapcraft_file:
        print(
            dedent(
                """\
            name: project-name
            base: core18
            version: "1.0"
            summary: sanity checks
            description: sanity checks
            grade: stable
            confinement: strict

            parts:
              nil:
                plugin: nil
            """
            ),
            file=snapcraft_file,
        )

    return snapcraft_yaml_path.relative_to(tmp_work_path)


@pytest.fixture(params=[True, False])
def project(snapcraft_yaml_path, request):
    """Return a project in host and managed-host modes."""
    snapcraft_project = Project(
        is_managed_host=request.param,
        snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix(),
    )
    return snapcraft_project


@pytest.fixture
def caplog_warning(caplog):
    """Return caplog set to logging.WARNING."""
    caplog.set_level(logging.WARNING)
    return caplog


def test_no_snap_dir(caplog_warning, project):
    conduct_project_sanity_check(project)
    assert len(caplog_warning.records) == 0


def test_icon(tmp_work_path):
    snapcraft_yaml_path = tmp_work_path / "snap/snapcraft.yaml"
    snapcraft_yaml_path.parent.mkdir(parents=True)
    with snapcraft_yaml_path.open("w") as snapcraft_file:
        print(
            dedent(
                """\
            name: project-name
            base: core18
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
            ),
            file=snapcraft_file,
        )

    project = Project(
        is_managed_host=False, snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix(),
    )

    # Test without icon raises error
    with pytest.raises(snapcraft.internal.errors.SnapcraftEnvironmentError) as exc_info:
        conduct_project_sanity_check(project)

    assert exc_info.value.get_brief() == "Specified icon 'foo.png' does not exist."

    # Test with icon passes.
    (tmp_work_path / "foo.png").touch()
    conduct_project_sanity_check(project)


def test_accepted_artifacts(caplog_warning, project):
    assets_dir = pathlib.Path("snap")

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

    conduct_project_sanity_check(project)

    assert len(caplog_warning.records) == 0


def test_unexpected_things(caplog_warning, project):
    assets_dir = pathlib.Path("snap")

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

    conduct_project_sanity_check(project)
    assert caplog_warning.records[0].message == (
        "The 'snap' directory is meant specifically for snapcraft, but it "
        "contains the following non-snapcraft-related paths, which is "
        "unsupported and will cause unexpected behavior:\n"
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
        "If you must store these files within the 'snap' directory, move them "
        "to 'snap/local', which is ignored by snapcraft."
    )
