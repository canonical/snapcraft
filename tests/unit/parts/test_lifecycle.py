# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import argparse
import textwrap
from pathlib import Path
from typing import Any, Dict
from unittest.mock import call

import pytest

from snapcraft import errors
from snapcraft.parts import lifecycle as parts_lifecycle
from snapcraft.projects import Project

_SNAPCRAFT_YAML_FILENAMES = [
    "snap/snapcraft.yaml",
    "build-aux/snap/snapcraft.yaml",
    "snapcraft.yaml",
    ".snapcraft.yaml",
]


@pytest.fixture
def snapcraft_yaml():
    def write_file(
        *, base: str, filename: str = "snap/snapcraft.yaml"
    ) -> Dict[str, Any]:
        content = textwrap.dedent(
            f"""
            name: mytest
            version: '0.1'
            base: {base}
            summary: Just some test data
            description: This is just some test data.
            grade: stable
            confinement: strict

            parts:
              part1:
                plugin: nil
            """
        )
        yaml_path = Path(filename)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.write_text(content)

        return {
            "name": "mytest",
            "title": None,
            "base": base,
            "compression": "xz",
            "version": "0.1",
            "contact": None,
            "donation": None,
            "issues": None,
            "source-code": None,
            "website": None,
            "summary": "Just some test data",
            "description": "This is just some test data.",
            "type": "app",
            "confinement": "strict",
            "icon": None,
            "layout": None,
            "license": None,
            "grade": "stable",
            "adopt-info": None,
            "architectures": [],
            "assumes": [],
            "hooks": None,
            "passthrough": None,
            "apps": None,
            "plugs": None,
            "slots": None,
            "parts": {"part1": {"plugin": "nil"}},
            "epoch": None,
        }

    yield write_file


def test_config_not_found(new_dir):
    """If snapcraft.yaml is not found, raise an error."""
    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run("pull", argparse.Namespace())

    assert str(raised.value) == (
        "Could not find snap/snapcraft.yaml. Are you sure you are in the right "
        "directory?"
    )
    assert raised.value.resolution == "To start a new project, use `snapcraft init`"


@pytest.mark.parametrize("filename", _SNAPCRAFT_YAML_FILENAMES)
def test_snapcraft_yaml_load(new_dir, snapcraft_yaml, filename, mocker):
    """Snapcraft.yaml should be parsed as a valid yaml file."""
    yaml_data = snapcraft_yaml(base="core22", filename=filename)
    run_command_mock = mocker.patch("snapcraft.parts.lifecycle._run_command")

    parts_lifecycle.run("pull", argparse.Namespace(parts=["part1"]))

    project = Project.unmarshal(yaml_data)

    assert run_command_mock.mock_calls == [
        call("pull", project, argparse.Namespace(parts=["part1"]))
    ]


def test_snapcraft_yaml_parse_error(new_dir, snapcraft_yaml, mocker):
    """If snapcraft.yaml is not a valid yaml, raise an error."""
    snapcraft_yaml(base="invalid: true")
    run_command_mock = mocker.patch("snapcraft.parts.lifecycle._run_command")

    with pytest.raises(errors.SnapcraftError) as raised:
        parts_lifecycle.run("pull", argparse.Namespace(parts=["part1"]))

    assert str(raised.value) == (
        "YAML parsing error: mapping values are not allowed here\n"
        '  in "snap/snapcraft.yaml", line 4, column 14'
    )
    assert run_command_mock.mock_calls == []


def test_legacy_base_not_core22(new_dir, snapcraft_yaml):
    """Only core22 is processed by the new code, use legacy otherwise."""
    snapcraft_yaml(base="core20")
    with pytest.raises(errors.LegacyFallback) as raised:
        parts_lifecycle.run("pull", argparse.Namespace())

    assert str(raised.value) == "base is not core22"


@pytest.mark.parametrize(
    "cmd,step",
    [
        ("pull", "pull"),
        ("build", "build"),
        ("stage", "stage"),
        ("prime", "prime"),
    ],
)
def test_lifecycle_run_command_step(cmd, step, snapcraft_yaml, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.meta.snap_yaml.write")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")

    parts_lifecycle._run_command(cmd, project, argparse.Namespace())

    assert run_mock.mock_calls == [call(step)]
    assert pack_mock.mock_calls == []


def test_lifecycle_run_command_pack(snapcraft_yaml, new_dir, mocker):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))
    run_mock = mocker.patch("snapcraft.parts.PartsLifecycle.run")
    mocker.patch("snapcraft.meta.snap_yaml.write")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")

    parts_lifecycle._run_command(
        "pack", project, argparse.Namespace(directory=None, output=None)
    )

    assert run_mock.mock_calls == [call("prime")]
    assert pack_mock.mock_calls == [
        call(new_dir / "work/prime", output=None, compression="xz")
    ]
