# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018, 2020 Canonical Ltd
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

import os
import pathlib
from unittest import mock

import pytest

from snapcraft.internal import errors
from snapcraft.internal.meta.snap import Snap
from snapcraft.plugins.v1 import gradle
from snapcraft.project import Project

from . import PluginsV1BaseTestCase


def test_schema():
    assert gradle.GradlePlugin.schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "additionalProperties": False,
        "properties": {
            "gradle-openjdk-version": {"default": "", "type": "string"},
            "gradle-options": {
                "default": [],
                "items": {"type": "string"},
                "minitems": 1,
                "type": "array",
                "uniqueItems": True,
            },
            "gradle-output-dir": {"default": "build/libs", "type": "string"},
            "gradle-version": {"type": "string"},
            "gradle-version-checksum": {"type": "string"},
        },
        "required": ["source"],
        "type": "object",
    }


def test_pull_properties():
    assert gradle.GradlePlugin.get_pull_properties() == [
        "gradle-version",
        "gradle-version-checksum",
        "gradle-openjdk-version",
    ]


def test_build_properties():
    assert gradle.GradlePlugin.get_build_properties() == [
        "gradle-options",
        "gradle-output-dir",
    ]


def _get_expected_java_version(gradle_plugin) -> str:
    base = gradle_plugin.project._snap_meta.base
    ant_openjdk_version = gradle_plugin.options.gradle_openjdk_version

    if ant_openjdk_version:
        expected_java_version = ant_openjdk_version
    elif base in ("core", "core16"):
        expected_java_version = "9"
    else:
        expected_java_version = "11"

    return expected_java_version


_BASE_JAVA_COMBINATIONS = [
    ("", "core"),
    ("8", "core"),
    ("", "core16"),
    ("8", "core16"),
    ("", "core18"),
    ("11", "core18"),
]


@pytest.fixture(params=_BASE_JAVA_COMBINATIONS)
def gradle_plugin(tmp_path, request):
    """Return an instance of GradlePlugin setup with different bases and java versions."""
    java_version, base = request.param

    class Options:
        gradle_options = []
        gradle_output_dir = "build/libs"
        gradle_version = gradle._DEFAULT_GRADLE_VERSION
        gradle_version_checksum = gradle._DEFAULT_GRADLE_CHECKSUM
        gradle_openjdk_version = java_version

    os.chdir(tmp_path)
    project = Project()
    project._snap_meta = Snap(name="test-snap", base=base, confinement="strict")

    return gradle.GradlePlugin("test-part", Options(), project)


@pytest.fixture()
def gradle_plugin_with_assets(gradle_plugin, mock_zip):
    """Return an instance of GradlePlugin with artifacts setup using gradle_plugin."""
    pathlib.Path(gradle_plugin.sourcedir).mkdir(parents=True)

    expected_java_version = _get_expected_java_version(gradle_plugin)

    fake_java_path = (
        pathlib.Path(gradle_plugin.installdir)
        / f"usr/lib/jvm/java-{expected_java_version}-openjdk-amd64/bin/java"
    )
    fake_java_path.parent.mkdir(parents=True)
    fake_java_path.touch()

    gradle_tar_path = (
        pathlib.Path(gradle_plugin.partdir)
        / f"gradle/apache-gradle-{gradle_plugin.options.gradle_version}-bin.zip"
    )
    gradle_tar_path.parent.mkdir(parents=True)
    gradle_tar_path.touch()

    return gradle_plugin


def test_stage_and_build_packages(gradle_plugin):
    expected_java_version = _get_expected_java_version(gradle_plugin)

    assert gradle_plugin.stage_packages == [
        f"openjdk-{expected_java_version}-jre-headless"
    ]
    assert gradle_plugin.build_packages == [
        f"openjdk-{expected_java_version}-jdk-headless",
        "ca-certificates-java",
    ]


def test_build_gradlew(mock_run, gradle_plugin_with_assets):
    plugin = gradle_plugin_with_assets
    (pathlib.Path(plugin.sourcedir) / "gradlew").touch()

    def fake_run(l, **kwargs):
        os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
        open(os.path.join(plugin.builddir, "build", "libs", "dummy.jar"), "w").close()

    mock_run.side_effect = fake_run

    plugin.build()

    mock_run.assert_called_once_with(
        ["./gradlew", "jar"], cwd=plugin.builddir, env=None
    )


def test_build_gradle(mock_run, gradle_plugin_with_assets):
    plugin = gradle_plugin_with_assets

    def fake_run(l, **kwargs):
        os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
        open(os.path.join(plugin.builddir, "build", "libs", "dummy.jar"), "w").close()

    mock_run.side_effect = fake_run

    plugin.build()

    mock_run.assert_called_once_with(
        ["gradle", "jar"], cwd=plugin.builddir, env=mock.ANY
    )


def test_build_war_gradle(mock_run, gradle_plugin_with_assets):
    plugin = gradle_plugin_with_assets

    def fake_run(l, **kwargs):
        os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
        open(os.path.join(plugin.builddir, "build", "libs", "dummy.war"), "w").close()

    mock_run.side_effect = fake_run

    plugin.build()

    mock_run.assert_called_once_with(
        ["gradle", "jar"], cwd=plugin.builddir, env=mock.ANY
    )


def test_build_war_gradlew(mock_run, gradle_plugin_with_assets):
    plugin = gradle_plugin_with_assets
    (pathlib.Path(plugin.sourcedir) / "gradlew").touch()

    def fake_run(l, **kwargs):
        os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
        open(os.path.join(plugin.builddir, "build", "libs", "dummy.war"), "w").close()

    mock_run.side_effect = fake_run

    plugin.build()

    mock_run.assert_called_once_with(
        ["./gradlew", "jar"], cwd=plugin.builddir, env=None
    )


class TestGradleProxies:

    scenarios = [
        (
            "http proxy url",
            dict(
                env_var=("http_proxy", "http://test_proxy"),
                expected_args=["-Dhttp.proxyHost=test_proxy"],
            ),
        ),
        (
            "http proxy url and port",
            dict(
                env_var=("http_proxy", "http://test_proxy:3000"),
                expected_args=["-Dhttp.proxyHost=test_proxy", "-Dhttp.proxyPort=3000"],
            ),
        ),
        (
            "authenticated http proxy url",
            dict(
                env_var=("http_proxy", "http://user:pass@test_proxy:3000"),
                expected_args=[
                    "-Dhttp.proxyHost=test_proxy",
                    "-Dhttp.proxyPort=3000",
                    "-Dhttp.proxyUser=user",
                    "-Dhttp.proxyPassword=pass",
                ],
            ),
        ),
        (
            "https proxy url",
            dict(
                env_var=("https_proxy", "https://test_proxy"),
                expected_args=["-Dhttps.proxyHost=test_proxy"],
            ),
        ),
        (
            "https proxy url and port",
            dict(
                env_var=("https_proxy", "https://test_proxy:3000"),
                expected_args=[
                    "-Dhttps.proxyHost=test_proxy",
                    "-Dhttps.proxyPort=3000",
                ],
            ),
        ),
        (
            "authenticated https proxy url",
            dict(
                env_var=("https_proxy", "http://user:pass@test_proxy:3000"),
                expected_args=[
                    "-Dhttps.proxyHost=test_proxy",
                    "-Dhttps.proxyPort=3000",
                    "-Dhttps.proxyUser=user",
                    "-Dhttps.proxyPassword=pass",
                ],
            ),
        ),
    ]

    def test_build_gradlew(
        self, monkeypatch, mock_run, gradle_plugin_with_assets, env_var, expected_args
    ):
        monkeypatch.setenv(*env_var)

        plugin = gradle_plugin_with_assets
        (pathlib.Path(plugin.sourcedir) / "gradlew").touch()

        def fake_run(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.jar"), "w"
            ).close()

        mock_run.side_effect = fake_run

        plugin.build()

        mock_run.assert_called_once_with(
            ["./gradlew"] + expected_args + ["jar"], cwd=plugin.builddir, env=None
        )

    def test_build_gradle(
        self, monkeypatch, mock_run, gradle_plugin_with_assets, env_var, expected_args
    ):
        monkeypatch.setenv(*env_var)

        plugin = gradle_plugin_with_assets

        def fake_run(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.jar"), "w"
            ).close()

        mock_run.side_effect = fake_run

        plugin.build()

        mock_run.assert_called_once_with(
            ["gradle"] + expected_args + ["jar"], cwd=plugin.builddir, env=mock.ANY
        )


class GradlePluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            gradle_options = []
            gradle_output_dir = "build/libs"
            gradle_version = gradle._DEFAULT_GRADLE_VERSION
            gradle_version_checksum = gradle._DEFAULT_GRADLE_CHECKSUM
            gradle_openjdk_version = ""

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            gradle.GradlePlugin,
            "test-part",
            self.options,
            self.project,
        )


class TestUnsupportedJDKVersionError:
    scenarios = (
        (
            "core",
            dict(
                base="core",
                version="11",
                expected_message=(
                    "The gradle-openjdk-version plugin property was set to '11'.\n"
                    "Valid values for the 'core' base are: '8' or '9'."
                ),
            ),
        ),
        (
            "core16",
            dict(
                base="core16",
                version="11",
                expected_message=(
                    "The gradle-openjdk-version plugin property was set to '11'.\n"
                    "Valid values for the 'core16' base are: '8' or '9'."
                ),
            ),
        ),
        (
            "core18",
            dict(
                base="core18",
                version="9",
                expected_message=(
                    "The gradle-openjdk-version plugin property was set to '9'.\n"
                    "Valid values for the 'core18' base are: '11' or '8'."
                ),
            ),
        ),
    )

    def test_use_invalid_openjdk_version_fails(self, base, version, expected_message):
        class Options:
            gradle_options = []
            gradle_output_dir = "build/libs"
            gradle_version = gradle._DEFAULT_GRADLE_VERSION
            gradle_version_checksum = gradle._DEFAULT_GRADLE_CHECKSUM
            gradle_openjdk_version = version

        project = Project()
        project._snap_meta = Snap(name="test-snap", base=base, confinement="strict")

        with pytest.raises(gradle.UnsupportedJDKVersionError) as error:
            gradle.GradlePlugin("test-part", Options(), project)
            assert str(error) == expected_message
