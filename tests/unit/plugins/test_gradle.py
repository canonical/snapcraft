# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
from unittest import mock

import fixtures
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import gradle
from tests import unit


class BaseGradlePluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            gradle_options = []
            gradle_output_dir = "build/libs"

        self.options = Options()

        self.project_options = snapcraft.ProjectOptions()

        # unset http and https proxies.
        self.useFixture(fixtures.EnvironmentVariable("http_proxy", None))
        self.useFixture(fixtures.EnvironmentVariable("https_proxy", None))


class GradlePluginTestCase(BaseGradlePluginTestCase):
    def test_schema(self):
        schema = gradle.GradlePlugin.schema()

        properties = schema["properties"]
        self.assertTrue(
            "gradle-options" in properties,
            'Expected "gradle-options" to be included in ' "properties",
        )

        gradle_options = properties["gradle-options"]

        self.assertTrue(
            "type" in gradle_options,
            'Expected "type" to be included in "gradle-options"',
        )
        self.assertThat(
            gradle_options["type"],
            Equals("array"),
            'Expected "gradle-options" "type" to be "array", but '
            'it was "{}"'.format(gradle_options["type"]),
        )

        self.assertTrue(
            "minitems" in gradle_options,
            'Expected "minitems" to be included in "gradle-options"',
        )
        self.assertThat(
            gradle_options["minitems"],
            Equals(1),
            'Expected "gradle-options" "minitems" to be 1, but '
            'it was "{}"'.format(gradle_options["minitems"]),
        )

        self.assertTrue(
            "uniqueItems" in gradle_options,
            'Expected "uniqueItems" to be included in "gradle-options"',
        )
        self.assertTrue(
            gradle_options["uniqueItems"],
            'Expected "gradle-options" "uniqueItems" to be "True"',
        )

        output_dir = properties["gradle-output-dir"]
        self.assertTrue(
            "type" in output_dir, 'Expected "type" to be included in "gradle-options"'
        )
        self.assertThat(
            output_dir["type"],
            Equals("string"),
            'Expected "gradle-options" "type" to be "string", '
            'but it was "{}"'.format(output_dir["type"]),
        )

    def test_get_build_properties(self):
        expected_build_properties = ["gradle-options", "gradle-output-dir"]
        resulting_build_properties = gradle.GradlePlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_gradlew(self, run_mock):
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        filename = os.path.join(os.getcwd(), "gradlew")
        open(filename, "w").close()

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.jar"), "w"
            ).close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([mock.call(["./gradlew", "jar"])])

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_gradle(self, run_mock):
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.jar"), "w"
            ).close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([mock.call(["gradle", "jar"])])

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_war_gradle(self, run_mock):
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.war"), "w"
            ).close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([mock.call(["gradle", "jar"])])

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_war_gradlew(self, run_mock):
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        filename = os.path.join(os.getcwd(), "gradlew")
        open(filename, "w").close()

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.war"), "w"
            ).close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([mock.call(["./gradlew", "jar"])])

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_fail_gradlew(self, run_mock):
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        filename = os.path.join(os.getcwd(), "gradlew")
        open(filename, "w").close()

        os.makedirs(plugin.sourcedir)
        self.assertRaises(RuntimeError, plugin.build)

        run_mock.assert_has_calls([mock.call(["./gradlew", "jar"])])

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_fail_gradle(self, run_mock):
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        os.makedirs(plugin.sourcedir)
        self.assertRaises(RuntimeError, plugin.build)

        run_mock.assert_has_calls([mock.call(["gradle", "jar"])])


class GradleProxyTestCase(BaseGradlePluginTestCase):

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

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_with_http_proxy_gradle(self, run_mock):
        var, value = self.env_var
        self.useFixture(fixtures.EnvironmentVariable(var, value))
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.war"), "w"
            ).close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls(
            [mock.call(["gradle"] + self.expected_args + ["jar"])]
        )

    @mock.patch.object(gradle.GradlePlugin, "run")
    def test_build_with_http_proxy_gradlew(self, run_mock):
        var, value = self.env_var
        self.useFixture(fixtures.EnvironmentVariable(var, value))
        plugin = gradle.GradlePlugin("test-part", self.options, self.project_options)

        filename = os.path.join(os.getcwd(), "gradlew")
        open(filename, "w").close()

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, "build", "libs"))
            open(
                os.path.join(plugin.builddir, "build", "libs", "dummy.war"), "w"
            ).close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls(
            [mock.call(["./gradlew"] + self.expected_args + ["jar"])]
        )
