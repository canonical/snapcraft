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

import io
import os
import tarfile
from textwrap import dedent
from unittest import mock
from xml.etree import ElementTree

import fixtures
from testtools.matchers import Equals, FileExists, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import maven
from tests import unit
from . import PluginsV1BaseTestCase


class MavenPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = maven.MavenPlugin.schema()

        properties = schema["properties"]
        self.assertTrue(
            "maven-options" in properties,
            'Expected "maven-options" to be included in ' "properties",
        )

        maven_options = properties["maven-options"]

        self.assertTrue(
            "type" in maven_options, 'Expected "type" to be included in "maven-options"'
        )
        self.assertThat(
            maven_options["type"],
            Equals("array"),
            'Expected "maven-options" "type" to be "array", but '
            'it was "{}"'.format(maven_options["type"]),
        )

        self.assertTrue(
            "minitems" in maven_options,
            'Expected "minitems" to be included in "maven-options"',
        )
        self.assertThat(
            maven_options["minitems"],
            Equals(1),
            'Expected "maven-options" "minitems" to be 1, but '
            'it was "{}"'.format(maven_options["minitems"]),
        )

        self.assertTrue(
            "uniqueItems" in maven_options,
            'Expected "uniqueItems" to be included in "maven-options"',
        )
        self.assertTrue(
            maven_options["uniqueItems"],
            'Expected "maven-options" "uniqueItems" to be "True"',
        )

        maven_targets = properties["maven-targets"]

        self.assertTrue(
            "type" in maven_targets, 'Expected "type" to be included in "maven-targets"'
        )
        self.assertThat(
            maven_targets["type"],
            Equals("array"),
            'Expected "maven-targets" "type" to be "array", but '
            'it was "{}"'.format(maven_targets["type"]),
        )

        self.assertTrue(
            "minitems" in maven_targets,
            'Expected "minitems" to be included in "maven-targets"',
        )
        self.assertThat(
            maven_targets["minitems"],
            Equals(1),
            'Expected "maven-targets" "minitems" to be 1, but '
            'it was "{}"'.format(maven_targets["minitems"]),
        )

        self.assertTrue(
            "uniqueItems" in maven_targets,
            'Expected "uniqueItems" to be included in "maven-targets"',
        )
        self.assertTrue(
            maven_targets["uniqueItems"],
            'Expected "maven-targets" "uniqueItems" to be "True"',
        )

    def test_get_pull_properties(self):
        expected_pull_properties = [
            "maven-version",
            "maven-version-checksum",
            "maven-openjdk-version",
        ]
        resulting_pull_properties = maven.MavenPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ["maven-options", "maven-targets"]
        resulting_build_properties = maven.MavenPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


class MavenPluginTest(PluginsV1BaseTestCase):

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
            maven_options = []
            maven_targets = [""]
            maven_version = maven._DEFAULT_MAVEN_VERSION
            maven_version_checksum = maven._DEFAULT_MAVEN_CHECKSUM
            maven_openjdk_version = self.java_version

        self.options = Options()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.sources.Tar")
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

        maven_tar_path = os.path.join(
            plugin.partdir,
            "maven",
            "apache-maven-{}-bin.tar.gz".format(plugin.options.maven_version),
        )
        os.makedirs(os.path.dirname(maven_tar_path))
        tarfile.TarFile(maven_tar_path, "w").close()

    @staticmethod
    def _canonicalize_settings(settings):
        with io.StringIO(settings) as f:
            tree = ElementTree.parse(f)
        for element in tree.iter():
            if element.text is not None and element.text.isspace():
                element.text = None
            if element.tail is not None and element.tail.isspace():
                element.tail = None
        with io.StringIO() as f:
            tree.write(
                f,
                encoding="unicode",
                default_namespace="http://maven.apache.org/SETTINGS/1.0.0",
            )
            return f.getvalue() + "\n"

    def assertSettingsEqual(self, expected, actual_file):
        with open(actual_file) as fr:
            observed = fr.read()

        print(repr(self._canonicalize_settings(expected)))
        print(repr(self._canonicalize_settings(observed)))
        self.assertThat(
            self._canonicalize_settings(observed),
            Equals(self._canonicalize_settings(expected)),
        )

    def test_stage_and_build_packages(self):
        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.assertThat(
            plugin.stage_packages,
            Equals(["openjdk-{}-jre-headless".format(self.expected_java_version)]),
        )
        self.assertThat(
            plugin.build_packages,
            Equals(["openjdk-{}-jdk-headless".format(self.expected_java_version)]),
        )

    def test_build(self):
        env_vars = (("http_proxy", None), ("https_proxy", None))
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "jar.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        self.run_mock.assert_called_once_with(
            ["mvn", "package"], cwd=plugin.builddir, env=mock.ANY
        )

    def test_build_fail(self):
        env_vars = (("http_proxy", None), ("https_proxy", None))
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        self.assertRaises(RuntimeError, plugin.build)

        self.run_mock.assert_called_once_with(
            ["mvn", "package"], cwd=plugin.builddir, env=mock.ANY
        )

    def test_build_war(self):
        env_vars = (("http_proxy", None), ("https_proxy", None))
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "war.war"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        self.run_mock.assert_called_once_with(
            ["mvn", "package"], cwd=plugin.builddir, env=mock.ANY
        )

    def test_build_with_targets(self):
        env_vars = (("http_proxy", None), ("https_proxy", None))
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        opts = self.options
        opts.maven_targets = ["child1", "child2"]
        plugin = maven.MavenPlugin("test-part", opts, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "child1", "target"))
            os.makedirs(os.path.join(plugin.builddir, "child2", "target"))
            open(
                os.path.join(plugin.builddir, "child1", "target", "child1.jar"), "w"
            ).close()
            open(
                os.path.join(plugin.builddir, "child2", "target", "child2.jar"), "w"
            ).close()

        self.run_mock.side_effect = side

        plugin.build()

        self.run_mock.assert_called_once_with(
            ["mvn", "package"], cwd=plugin.builddir, env=mock.ANY
        )

    def test_build_with_http_proxy(self):
        env_vars = (
            ("http_proxy", "http://localhost:3132"),
            ("https_proxy", None),
            ("no_proxy", None),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        settings_path = os.path.join(plugin.partdir, "m2", "settings.xml")
        self.run_mock.assert_called_once_with(
            ["mvn", "package", "-s", settings_path], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertThat(settings_path, FileExists())
        expected_content = dedent(
            """\
            <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                      xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">
              <interactiveMode>false</interactiveMode>
              <proxies>
                <proxy>
                  <id>http_proxy</id>
                  <active>true</active>
                  <protocol>http</protocol>
                  <host>localhost</host>
                  <port>3132</port>
                  <nonProxyHosts>localhost</nonProxyHosts>
                </proxy>
              </proxies>
            </settings>
        """
        )
        self.assertSettingsEqual(expected_content, settings_path)

    def test_build_with_http_proxy_and_no_proxy(self):
        env_vars = (
            ("http_proxy", "http://localhost:3132"),
            ("https_proxy", None),
            ("no_proxy", "internal"),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        settings_path = os.path.join(plugin.partdir, "m2", "settings.xml")
        self.run_mock.assert_called_once_with(
            ["mvn", "package", "-s", settings_path], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertThat(settings_path, FileExists())
        expected_content = dedent(
            """\
            <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                      xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">
              <interactiveMode>false</interactiveMode>
              <proxies>
                <proxy>
                  <id>http_proxy</id>
                  <active>true</active>
                  <protocol>http</protocol>
                  <host>localhost</host>
                  <port>3132</port>
                  <nonProxyHosts>internal</nonProxyHosts>
                </proxy>
              </proxies>
            </settings>
        """
        )
        self.assertSettingsEqual(expected_content, settings_path)

    def test_build_with_http_proxy_and_no_proxies(self):
        env_vars = (
            ("http_proxy", "http://localhost:3132"),
            ("https_proxy", None),
            ("no_proxy", "internal, pseudo-dmz"),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        settings_path = os.path.join(plugin.partdir, "m2", "settings.xml")
        self.run_mock.assert_called_once_with(
            ["mvn", "package", "-s", settings_path], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertThat(settings_path, FileExists())
        expected_content = dedent(
            """\
            <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                      xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">
              <interactiveMode>false</interactiveMode>
              <proxies>
                <proxy>
                  <id>http_proxy</id>
                  <active>true</active>
                  <protocol>http</protocol>
                  <host>localhost</host>
                  <port>3132</port>
                  <nonProxyHosts>internal|pseudo-dmz</nonProxyHosts>
                </proxy>
              </proxies>
            </settings>
            """
        )
        self.assertSettingsEqual(expected_content, settings_path)

    def test_build_with_http_and_https_proxy(self):
        env_vars = (
            ("http_proxy", "http://localhost:3132"),
            ("https_proxy", "http://localhost:3133"),
            ("no_proxy", None),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        settings_path = os.path.join(plugin.partdir, "m2", "settings.xml")
        self.run_mock.assert_called_once_with(
            ["mvn", "package", "-s", settings_path], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertThat(settings_path, FileExists())
        expected_content = dedent(
            """\
            <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                      xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">
              <interactiveMode>false</interactiveMode>
              <proxies>
                <proxy>
                  <id>http_proxy</id>
                  <active>true</active>
                  <protocol>http</protocol>
                  <host>localhost</host>
                  <port>3132</port>
                  <nonProxyHosts>localhost</nonProxyHosts>
                </proxy>
                <proxy>
                  <id>https_proxy</id>
                  <active>true</active>
                  <protocol>https</protocol>
                  <host>localhost</host>
                  <port>3133</port>
                  <nonProxyHosts>localhost</nonProxyHosts>
                </proxy>
              </proxies>
            </settings>
            """
        )
        self.assertSettingsEqual(expected_content, settings_path)

    def test_build_with_authenticated_proxies(self):
        env_vars = (
            ("http_proxy", "http://user1:pass1@localhost:3132"),
            ("https_proxy", "http://user2:pass2@localhost:3133"),
            ("no_proxy", None),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        def side(l, **kwargs):
            os.makedirs(os.path.join(plugin.builddir, "target"))
            open(os.path.join(plugin.builddir, "target", "dummy.jar"), "w").close()

        self.run_mock.side_effect = side

        plugin.build()

        settings_path = os.path.join(plugin.partdir, "m2", "settings.xml")
        self.run_mock.assert_called_once_with(
            ["mvn", "package", "-s", settings_path], cwd=plugin.builddir, env=mock.ANY
        )
        self.assertThat(settings_path, FileExists())
        expected_content = dedent(
            """\
            <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                      xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">
              <interactiveMode>false</interactiveMode>
              <proxies>
                <proxy>
                  <id>http_proxy</id>
                  <active>true</active>
                  <protocol>http</protocol>
                  <host>localhost</host>
                  <port>3132</port>
                  <username>user1</username>
                  <password>pass1</password>
                  <nonProxyHosts>localhost</nonProxyHosts>
                </proxy>
                <proxy>
                  <id>https_proxy</id>
                  <active>true</active>
                  <protocol>https</protocol>
                  <host>localhost</host>
                  <port>3133</port>
                  <username>user2</username>
                  <password>pass2</password>
                  <nonProxyHosts>localhost</nonProxyHosts>
                </proxy>
              </proxies>
            </settings>
            """
        )
        self.assertSettingsEqual(expected_content, settings_path)


class MavenPluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            source = "dir"
            maven_version = "3.3"
            maven_version_checksum = "sha1/1234567890"
            maven_openjdk_version = "10"

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            maven.MavenPlugin,
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
                    "The maven-openjdk-version plugin property was set to '11'.\n"
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
                    "The maven-openjdk-version plugin property was set to '11'.\n"
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
                    "The maven-openjdk-version plugin property was set to '9'.\n"
                    "Valid values for the 'core18' base are: '11' or '8'."
                ),
            ),
        ),
    )

    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = self.base

        class Options:
            maven_options = []
            maven_targets = [""]
            maven_version = maven._DEFAULT_MAVEN_VERSION
            maven_version_checksum = maven._DEFAULT_MAVEN_CHECKSUM
            maven_openjdk_version = self.version

        self.options = Options()

    def test_use_invalid_openjdk_version_fails(self):
        raised = self.assertRaises(
            maven.UnsupportedJDKVersionError,
            maven.MavenPlugin,
            "test-part",
            self.options,
            self.project,
        )
        self.assertThat(str(raised), Equals(self.expected_message))
