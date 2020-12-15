# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018-2020 Canonical Ltd
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

import fixtures
import pytest
from testtools.matchers import Equals, HasLength

from snapcraft.internal import errors
from snapcraft.internal.meta.snap import Snap
from snapcraft.plugins.v1 import ant
from snapcraft.project import Project
from tests import unit

from . import PluginsV1BaseTestCase


class AntPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = ant.AntPlugin.schema()

        properties = schema["properties"]
        for expected in ["ant-properties", "ant-build-targets"]:
            self.assertTrue(
                expected in properties,
                "Expected {!r} to be included in properties".format(expected),
            )

        properties_type = schema["properties"]["ant-properties"]["type"]
        self.assertThat(
            properties_type,
            Equals("object"),
            'Expected "ant-properties" "type" to be "object", '
            'but it was "{}"'.format(properties_type),
        )
        build_targets_type = schema["properties"]["ant-build-targets"]["type"]
        self.assertThat(
            build_targets_type,
            Equals("array"),
            'Expected "ant-build-targets" "type" to be "object", '
            'but it was "{}"'.format(build_targets_type),
        )

    def test_get_pull_properties(self):
        expected_pull_properties = [
            "ant-channel",
            "ant-version",
            "ant-version-checksum",
            "ant-openjdk-version",
        ]
        resulting_pull_properties = ant.AntPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = [
            "ant-build-targets",
            "ant-properties",
            "ant-buildfile",
        ]
        resulting_build_properties = ant.AntPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


def _get_expected_java_version(ant_plugin) -> str:
    base = ant_plugin.project._snap_meta.base
    ant_openjdk_version = ant_plugin.options.ant_openjdk_version

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
def ant_plugin(tmp_work_path, request):
    """Return an instance of AntPlugin setup with different bases and java versions."""
    java_version, base = request.param

    class Options:
        ant_properties = {}
        ant_build_targets = None
        ant_channel = None
        ant_version = "1.10.5"
        ant_version_checksum = "sha512/a7f1e0cec9d5ed1b3ab6cddbb9364f127305a997bbc88ecd734f9ef142ec0332375e01ace3592759bb5c3307cd9c1ac0a78a30053f304c7030ea459498e4ce4e"
        ant_openjdk_version = java_version
        ant_buildfile = None

    project = Project()
    project._snap_meta = Snap(name="test-snap", base=base, confinement="strict")

    return ant.AntPlugin("test-part", Options(), project)


@pytest.fixture()
def ant_plugin_with_assets(ant_plugin):
    """Return an instance of AntPlugin with artifacts setup using ant_plugin."""
    pathlib.Path(ant_plugin.sourcedir).mkdir(parents=True)

    expected_java_version = _get_expected_java_version(ant_plugin)

    fake_java_path = (
        pathlib.Path(ant_plugin.installdir)
        / f"usr/lib/jvm/java-{expected_java_version}-openjdk-amd64/bin/java"
    )
    fake_java_path.parent.mkdir(parents=True)
    fake_java_path.touch()

    ant_tar_path = (
        pathlib.Path(ant_plugin.partdir)
        / f"ant/apache-ant-{ant_plugin.options.ant_version}-bin.tar.bz2"
    )
    ant_tar_path.parent.mkdir(parents=True)
    ant_tar_path.touch()

    return ant_plugin


def test_stage_and_build_packages(ant_plugin):
    expected_java_version = _get_expected_java_version(ant_plugin)

    assert ant_plugin.stage_packages == [
        f"openjdk-{expected_java_version}-jre-headless"
    ]
    assert ant_plugin.build_packages == [
        f"openjdk-{expected_java_version}-jdk-headless"
    ]


def test_build(mock_tar, mock_run, ant_plugin_with_assets):
    plugin = ant_plugin_with_assets

    def fake_run(cmd, *args, **kwargs):
        os.makedirs(os.path.join(plugin.builddir, "target"))
        open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

    mock_run.side_effect = fake_run

    plugin.build()

    mock_run.assert_called_once_with(["ant"], cwd=plugin.builddir, env=mock.ANY)
    mock_tar.assert_called_once_with(
        ant._ANT_ARCHIVE_FORMAT_URL.format(version=plugin.options.ant_version),
        mock.ANY,
        source_checksum=plugin.options.ant_version_checksum,
    )


def test_build_with_options(mock_tar, mock_run, ant_plugin_with_assets):
    plugin = ant_plugin_with_assets
    plugin.options.ant_build_targets = ["artifacts", "jar"]
    plugin.options.ant_properties = {"basedir": "."}

    plugin.build()

    mock_run.assert_called_once_with(
        ["ant", "artifacts", "jar", "-Dbasedir=."], cwd=plugin.builddir, env=mock.ANY
    )
    mock_tar.assert_called_once_with(
        ant._ANT_ARCHIVE_FORMAT_URL.format(version=plugin.options.ant_version),
        mock.ANY,
        source_checksum=plugin.options.ant_version_checksum,
    )


def test_build_with_explicit_buildfile(mock_tar, mock_run, ant_plugin_with_assets):
    plugin = ant_plugin_with_assets
    plugin.options.ant_buildfile = "test.xml"

    plugin.build()

    mock_run.assert_called_once_with(
        ["ant", "-f", "test.xml"], cwd=plugin.builddir, env=mock.ANY
    )
    mock_tar.assert_called_once_with(
        ant._ANT_ARCHIVE_FORMAT_URL.format(version=plugin.options.ant_version),
        mock.ANY,
        source_checksum=plugin.options.ant_version_checksum,
    )


class AntPluginSnapTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class Options:
            ant_properties = {}
            ant_build_targets = None
            ant_channel = None
            ant_version = None
            ant_version_checksum = None
            ant_openjdk_version = None
            ant_buildfile = None

        self.options = Options()

        self.run_mock = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.common.run")
        ).mock
        self.tar_mock = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.sources.Tar")
        ).mock

    def create_assets(self, plugin):
        os.makedirs(plugin.sourcedir)

        fake_java_path = os.path.join(
            plugin.installdir,
            "usr",
            "lib",
            "jvm",
            "java-{}-openjdk-amd64".format(plugin._java_version),
            "bin",
            "java",
        )
        os.makedirs(os.path.dirname(fake_java_path))
        open(fake_java_path, "w").close()

    def test_build_with_default_snap(self):
        plugin = ant.AntPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        self.tar_mock.assert_not_called()
        self.run_mock.assert_called_once_with(
            ["ant"], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertEqual(plugin.build_snaps, ["ant/" + ant._DEFAULT_ANT_SNAP_CHANNEL])

    def test_build_with_explicit_snap(self):
        self.options.ant_channel = "other/channel"
        plugin = ant.AntPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        self.tar_mock.assert_not_called()
        self.run_mock.assert_called_once_with(
            ["ant"], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertEqual(plugin.build_snaps, ["ant/other/channel"])


class AntPluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            ant_properties = {}
            ant_build_targets = None
            ant_channel = None
            ant_version = "1.10.5"
            ant_version_checksum = "sha512/a7f1e0cec9d5ed1b3ab6cddbb9364f127305a997bbc88ecd734f9ef142ec0332375e01ace3592759bb5c3307cd9c1ac0a78a30053f304c7030ea459498e4ce4e"
            ant_openjdk_version = ""

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            ant.AntPlugin,
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
                    "The ant-openjdk-version plugin property was set to '11'.\n"
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
                    "The ant-openjdk-version plugin property was set to '11'.\n"
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
                    "The ant-openjdk-version plugin property was set to '9'.\n"
                    "Valid values for the 'core18' base are: '11' or '8'."
                ),
            ),
        ),
    )

    def test_use_invalid_openjdk_version_fails(self, base, version, expected_message):
        class Options:
            ant_properties = {}
            ant_build_targets = None
            ant_channel = None
            ant_version = "1.10.5"
            ant_version_checksum = "sha512/a7f1e0cec9d5ed1b3ab6cddbb9364f127305a997bbc88ecd734f9ef142ec0332375e01ace3592759bb5c3307cd9c1ac0a78a30053f304c7030ea459498e4ce4e"
            ant_openjdk_version = version

        project = Project()
        project._snap_meta = Snap(name="test-snap", base=base, confinement="strict")

        with pytest.raises(ant.UnsupportedJDKVersionError) as error:
            ant.AntPlugin("test-part", Options(), project)
            assert str(error) == expected_message
