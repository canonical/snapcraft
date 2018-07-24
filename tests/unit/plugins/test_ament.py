# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
from testtools.matchers import Contains, Equals, FileExists, FileContains, HasLength

import snapcraft
from snapcraft.plugins import ament
from tests import unit


class AmentPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class props:
            version = "release-beta3"

        self.properties = props()
        self.project = snapcraft.ProjectOptions()

        patcher = mock.patch("snapcraft.plugins._ros.ros2.Bootstrapper")
        self.bootstrapper_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = ament.AmentPlugin.schema()

        properties = schema["properties"]
        self.assertThat(properties, HasLength(1))
        self.assertThat(properties, Contains("version"))

    def test_schema_version(self):
        schema = ament.AmentPlugin.schema()

        # Check version property
        rosdistro = schema["properties"]["version"]
        expected = ("type", "default")
        self.assertThat(rosdistro, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(rosdistro, Contains(prop))
        self.assertThat(rosdistro["type"], Equals("string"))
        self.assertThat(rosdistro["default"], Equals("release-beta3"))

    def test_pull_properties(self):
        expected_pull_properties = ["version"]
        actual_pull_properties = ament.AmentPlugin.get_pull_properties()

        self.assertThat(
            actual_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, actual_pull_properties)

    def test_sources_include_ros_archive(self):
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        self.assertThat(
            plugin.PLUGIN_STAGE_SOURCES,
            Contains("deb http://packages.ros.org/ros/ubuntu/ xenial main"),
        )

    def test_bootstrap_build_packages_are_included(self):
        self.bootstrapper_mock.return_value.get_build_packages.return_value = ["foo"]
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)
        self.assertThat(plugin.build_packages, Contains("foo"))

    def test_bootstrap_stage_packages_are_included(self):
        self.bootstrapper_mock.return_value.get_stage_packages.return_value = ["foo"]
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)
        self.assertThat(plugin.stage_packages, Contains("foo"))

    def test_pull_fetches_ros2_underlay(self):
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        plugin.pull()

        self.bootstrapper_mock.return_value.pull.assert_called_once_with()
        self.bootstrapper_mock.return_value.build.assert_not_called()

    def test_clean_pull_cleans_bootstrap(self):
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        plugin.clean_pull()

        mock_clean = self.bootstrapper_mock.return_value.clean
        mock_clean.assert_called_once_with()

    @mock.patch("snapcraft.internal.common.run")
    def test_build_builds_ros2_underlay(self, mock_run):
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        self.bootstrapper_mock.return_value.build.return_value = "bootstrap"
        os.mkdir("bootstrap")
        with open(os.path.join("bootstrap", "test-file"), "w") as f:
            f.write("hello")

        plugin.build()

        self.bootstrapper_mock.return_value.pull.assert_not_called()
        self.bootstrapper_mock.return_value.build.assert_called_once_with()

        # Assert that the bootstrap dir was copied into the installdir
        self.assertThat(os.path.join(plugin.installdir, "test-file"), FileExists())
        self.assertThat(
            os.path.join(plugin.installdir, "test-file"), FileContains("hello")
        )

        class check_env:
            def __eq__(self, env):
                return env["PYTHONPATH"] == os.path.join(
                    os.path.sep, "usr", "lib", "python3", "dist-packages"
                )

        # Verify that the source space was built as expected, and that the
        # system's PYTHONPATH was included while building
        mock_run.assert_called_once_with(
            [
                "ament",
                "build",
                plugin.sourcedir,
                "--build-space",
                plugin.builddir,
                "--install-space",
                plugin.installdir,
                "--cmake-args",
                "-DCMAKE_BUILD_TYPE=Release",
            ],
            cwd=mock.ANY,
            env=check_env(),
        )

    def test_prepare_build_rewrites_ament_current_prefix(self):
        """Make sure AMENT_CURRENT_PREFIX is rewritten to be the installdir

        Otherwise ament embeds absolute paths in there, which results in a
        non-relocatable snap.
        """
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        self.bootstrapper_mock.return_value.build.return_value = "bootstrap"
        os.mkdir("bootstrap")
        with open(os.path.join("bootstrap", "test-file"), "w") as f:
            f.write("${AMENT_CURRENT_PREFIX:=bootstrap}")

        plugin._prepare_build()

        self.assertThat(os.path.join(plugin.installdir, "test-file"), FileExists())
        self.assertThat(
            os.path.join(plugin.installdir, "test-file"),
            FileContains("${{AMENT_CURRENT_PREFIX:={}}}".format(plugin.installdir)),
        )

    def test_finish_build_rewrites_ament_current_prefix(self):
        """Make sure AMENT_CURRENT_PREFIX is rewritten to $SNAP

        Otherwise ament keeps an absolute path in there, requiring us to define
        it at runtime to a valid location. Also, without this, multiple parts
        would clash.
        """
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        self.bootstrapper_mock.return_value.build.return_value = "bootstrap"
        os.mkdir("bootstrap")
        with open(os.path.join("bootstrap", "test-file"), "w") as f:
            f.write("${AMENT_CURRENT_PREFIX:=bootstrap}")

        plugin._prepare_build()
        plugin._finish_build()

        self.assertThat(os.path.join(plugin.installdir, "test-file"), FileExists())
        self.assertThat(
            os.path.join(plugin.installdir, "test-file"),
            FileContains("${AMENT_CURRENT_PREFIX:=$SNAP}"),
        )

    def test_environment(self):
        plugin = ament.AmentPlugin("test-part", self.properties, self.project)

        python_path = os.path.join(
            plugin.installdir, "usr", "lib", "python2.7", "dist-packages"
        )
        os.makedirs(python_path)

        # Joining and re-splitting to get hacked script in there as well
        environment = "\n".join(plugin.env(plugin.installdir)).split("\n")

        self.assertThat(
            environment, Contains('PYTHONUSERBASE="{}"'.format(plugin.installdir))
        )
        self.assertThat(
            environment,
            Contains(
                'PYTHONPATH="{}:{}${{PYTHONPATH:+:$PYTHONPATH}}"'.format(
                    os.path.join(
                        plugin.installdir, "usr", "lib", "python3", "dist-packages"
                    ),
                    os.path.join(plugin.installdir, "lib", "python3", "dist-packages"),
                )
            ),
        )
        self.assertThat(
            environment, Contains("    . {}/setup.sh".format(plugin.installdir))
        )
