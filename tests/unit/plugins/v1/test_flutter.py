# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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
from unittest.mock import call

import pytest

from snapcraft.internal import errors
from snapcraft.internal.meta.snap import Snap
from snapcraft.plugins.v1 import flutter
from snapcraft.project import Project


def test_schema():
    assert flutter.FlutterPlugin.schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "additionalProperties": False,
        "properties": {
            "flutter-channel": {"enum": ["dev", "master"], "type": "string",},
            "flutter-revision": {"default": None, "type": "string"},
            "flutter-target": {"default": "lib/main.dart", "type": "string"},
        },
        "required": ["source"],
        "type": "object",
    }


def test_get_pull_properties():
    assert flutter.FlutterPlugin.get_pull_properties() == [
        "flutter-channel",
        "flutter-revision",
    ]


def test_get_build_properties():
    assert flutter.FlutterPlugin.get_build_properties() == ["flutter-target"]


@pytest.fixture
def flutter_options():
    """Return Options object suitable for the flutter plugin."""

    class Options:
        flutter_channel = "stable"
        flutter_target = "lib/main.dart"
        flutter_revision = None

    return Options()


@pytest.fixture
def flutter_plugin(tmp_work_path, project, flutter_options):
    """Return an instance of FlutterPlugin setup with different bases."""
    return flutter.FlutterPlugin("test-part", flutter_options, project)


def test_pull(mock_subprocess_run, flutter_plugin):
    flutter_plugin.pull()

    assert mock_subprocess_run.mock_calls == [
        call(
            ["flutter", "channel", flutter_plugin.options.flutter_channel], check=True
        ),
        call(["flutter", "config", "--enable-linux-desktop"], check=True),
        call(["flutter", "upgrade"], check=True),
        call(["flutter", "doctor"], check=True),
        call(["flutter", "pub", "get"], check=True),
    ]


def test_pull_with_revision(mock_subprocess_run, flutter_plugin):
    flutter_plugin.options.flutter_revision = "foo"
    flutter_plugin.pull()

    assert mock_subprocess_run.mock_calls == [
        call(
            ["flutter", "channel", flutter_plugin.options.flutter_channel], check=True
        ),
        call(["flutter", "config", "--enable-linux-desktop"], check=True),
        call(["flutter", "upgrade"], check=True),
        call(["flutter", "doctor"], check=True),
        call("yes | flutter version foo", shell=True, check=True,),
        call(["flutter", "pub", "get"], check=True),
    ]


def test_build(mock_run, flutter_plugin):
    # "mock" create the bundle directory and app.
    app_path = (
        pathlib.Path(flutter_plugin.builddir) / "build/linux/release/bundle/my_app"
    )
    app_path.parent.mkdir(parents=True)
    app_path.touch()

    # Create the installdir.
    pathlib.Path(flutter_plugin.installdir).mkdir(parents=True)

    flutter_plugin.build()

    assert mock_run.mock_calls == [
        call(
            [
                "flutter",
                "build",
                "linux",
                "--release",
                "-v",
                "-t",
                flutter_plugin.options.flutter_target,
            ]
        )
    ]
    assert (pathlib.Path(flutter_plugin.installdir) / "bin/my_app").exists()


def test_unsupported_base_raises(flutter_options):
    project = Project()
    project._snap_meta = Snap(name="test-snap", base="bad-base", confinement="strict")

    with pytest.raises(errors.PluginBaseError):
        flutter.FlutterPlugin("test-part", flutter_options, project)
