# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import pytest

from snapcraft.project import errors, get_snapcraft_yaml


@pytest.fixture(
    params=[
        "snapcraft.yaml",
        "snap/snapcraft.yaml",
        ".snapcraft.yaml",
        "build-aux/snap/snapcraft.yaml",
    ]
)
def stub_snapcraft_yaml_file(tmp_work_path, request):
    """Return stub snapcraft.yaml file variants to find."""
    snapcraft_yaml = request.param
    snapcraft_yaml_path = tmp_work_path / snapcraft_yaml
    snapcraft_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    snapcraft_yaml_path.touch()
    return snapcraft_yaml


def test_get_snapcraft_yaml(stub_snapcraft_yaml_file):
    assert get_snapcraft_yaml() == stub_snapcraft_yaml_file


def test_config_raises_on_missing_snapcraft_yaml(tmp_work_path):
    """Test that an error is raised if snap/snapcraft.yaml is missing"""
    with pytest.raises(errors.MissingSnapcraftYamlError):
        get_snapcraft_yaml()


@pytest.fixture(
    params=["snap/snapcraft.yaml", ".snapcraft.yaml", "build-aux/snap/snapcraft.yaml"]
)
def duplicate_stub_snapcraft_yaml_file(stub_snapcraft_yaml_file, request):
    """Return duplicate stub snapcraft.yaml file variants to find."""
    snapcraft_yaml = request.param

    if snapcraft_yaml == stub_snapcraft_yaml_file:
        snapcraft_yaml = "snapcraft.yaml"

    snapcraft_yaml_path = pathlib.Path(snapcraft_yaml)
    snapcraft_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    snapcraft_yaml_path.touch()
    return snapcraft_yaml


def test_duplicates(duplicate_stub_snapcraft_yaml_file):
    with pytest.raises(errors.DuplicateSnapcraftYamlError):
        get_snapcraft_yaml()
