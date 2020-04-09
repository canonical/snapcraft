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
import tarfile
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import ant
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


class AntPluginBaseTest(PluginsV1BaseTestCase):
    scenarios = (
        (
            "core java version 8 ",
            dict(base="core", java_version="8", expected_java_version="8"),
        ),
        (
            "core java version 8 ",
            dict(base="core", java_version="8", expected_java_version="8"),
        ),
        (
            "core java version default ",
            dict(base="core", java_version="", expected_java_version="9"),
        ),
        (
            "core16 java version 8 ",
            dict(base="core16", java_version="8", expected_java_version="8"),
        ),
        (
            "core16 java version 8 ",
            dict(base="core16", java_version="8", expected_java_version="8"),
        ),
        (
            "core16 java version default ",
            dict(base="core16", java_version="", expected_java_version="9"),
        ),
        (
            "core18 java version 8 ",
            dict(base="core18", java_version="8", expected_java_version="8"),
        ),
        (
            "core18 java version 11",
            dict(base="core18", java_version="11", expected_java_version="11"),
        ),
        (
            "core18 java version default",
            dict(base="core18", java_version="", expected_java_version="11"),
        ),
    )

    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = self.base

        class Options:
            ant_properties = {}
            ant_build_targets = None
            ant_channel = None
            ant_version = "1.10.5"
            ant_version_checksum = "sha512/a7f1e0cec9d5ed1b3ab6cddbb9364f127305a997bbc88ecd734f9ef142ec0332375e01ace3592759bb5c3307cd9c1ac0a78a30053f304c7030ea459498e4ce4e"
            ant_openjdk_version = self.java_version
            ant_buildfile = None

        self.options = Options()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.internal.sources.Tar")
        self.tar_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def create_assets(self, plugin):
        os.makedirs(plugin.sourcedir)

        fake_java_path = os.path.join(
            plugin.installdir,
            "usr",
            "lib",
            "jvm",
            "java-{}-openjdk-amd64".format(self.expected_java_version),
            "bin",
            "java",
        )
        os.makedirs(os.path.dirname(fake_java_path))
        open(fake_java_path, "w").close()

        ant_tar_path = os.path.join(
            plugin.partdir,
            "ant",
            "apache-ant-{}-bin.tar.bz2".format(plugin.options.ant_version),
        )
        os.makedirs(os.path.dirname(ant_tar_path))
        tarfile.TarFile(ant_tar_path, "w").close()

    def test_stage_and_build_packages(self):
        plugin = ant.AntPlugin("test-part", self.options, self.project)

        self.assertThat(
            plugin.stage_packages,
            Equals(["openjdk-{}-jre-headless".format(self.expected_java_version)]),
        )
        self.assertThat(
            plugin.build_packages,
            Equals(["openjdk-{}-jdk-headless".format(self.expected_java_version)]),
        )

    def test_build(self):
        plugin = ant.AntPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        self.run_mock.assert_called_once_with(
            ["ant"], cwd=plugin.builddir, env=mock.ANY
        )
        self.tar_mock.assert_called_once_with(
            ant._ANT_ARCHIVE_FORMAT_URL.format(version=self.options.ant_version),
            mock.ANY,
            source_checksum=self.options.ant_version_checksum,
        )

    def test_build_with_options(self):
        self.options.ant_build_targets = ["artifacts", "jar"]
        self.options.ant_properties = {"basedir": "."}

        plugin = ant.AntPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.build()

        self.run_mock.assert_called_once_with(
            ["ant", "artifacts", "jar", "-Dbasedir=."],
            cwd=plugin.builddir,
            env=mock.ANY,
        )
        self.tar_mock.assert_called_once_with(
            ant._ANT_ARCHIVE_FORMAT_URL.format(version=self.options.ant_version),
            mock.ANY,
            source_checksum=self.options.ant_version_checksum,
        )

    def test_build_with_explicit_buildfile(self):
        self.options.ant_buildfile = "test.xml"

        plugin = ant.AntPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.build()

        self.run_mock.assert_called_once_with(
            ["ant", "-f", "test.xml"], cwd=plugin.builddir, env=mock.ANY
        )
        self.tar_mock.assert_called_once_with(
            ant._ANT_ARCHIVE_FORMAT_URL.format(version=self.options.ant_version),
            mock.ANY,
            source_checksum=self.options.ant_version_checksum,
        )

    def test_env(self):
        plugin = ant.AntPlugin("test-part", self.options, self.project)

        os.makedirs(os.path.join(plugin.installdir, "jar"))
        open(os.path.join(plugin.installdir, "jar", "lib1.jar"), "w").close()
        open(os.path.join(plugin.installdir, "jar", "lib2.jar"), "w").close()
        env = plugin.env(plugin.partdir)
        self.assertIn(
            "CLASSPATH={}/jar/lib1.jar:{}/jar/lib2.jar:$CLASSPATH".format(
                plugin.partdir, plugin.partdir
            ),
            env,
        )

    def test_build_env_proxies(self):
        env_vars = (
            ("http_proxy", "http://user:pass@localhost:3132"),
            ("https_proxy", "http://user2:pass2@localhost2:3133"),
        )
        for key, value in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(key, value))

        plugin = ant.AntPlugin("test-part", self.options, self.project)

        env = plugin._build_environment()
        self.assertThat(env, Contains("ANT_OPTS"))
        self.assertThat(
            env["ANT_OPTS"],
            Equals(
                "-Dhttp.proxyHost=localhost -Dhttp.proxyPort=3132 "
                "-Dhttp.proxyUser=user -Dhttp.proxyPassword=pass "
                "-Dhttps.proxyHost=localhost2 -Dhttps.proxyPort=3133 "
                "-Dhttps.proxyUser=user2 -Dhttps.proxyPassword=pass2"
            ),
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


class UnsupportedJDKVersionErrorTest(PluginsV1BaseTestCase):

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

    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = self.base

        class Options:
            ant_properties = {}
            ant_build_targets = None
            ant_channel = None
            ant_version = "1.10.5"
            ant_version_checksum = "sha512/a7f1e0cec9d5ed1b3ab6cddbb9364f127305a997bbc88ecd734f9ef142ec0332375e01ace3592759bb5c3307cd9c1ac0a78a30053f304c7030ea459498e4ce4e"
            ant_openjdk_version = self.version

        self.options = Options()

    def test_use_invalid_openjdk_version_fails(self):
        raised = self.assertRaises(
            ant.UnsupportedJDKVersionError,
            ant.AntPlugin,
            "test-part",
            self.options,
            self.project,
        )
        self.assertThat(str(raised), Equals(self.expected_message))
