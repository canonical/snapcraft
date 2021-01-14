# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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
import os
import pathlib
from unittest import mock

import fixtures
import pytest
from testscenarios import multiply_scenarios
from testtools.matchers import Contains, Equals, FileExists, HasLength, LessThan, Not

from snapcraft import repo
from snapcraft.internal import errors
from snapcraft.plugins.v1 import _ros, colcon
from tests import unit

from . import PluginsV1BaseTestCase


class ColconPluginTestBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class props:
            colcon_rosdistro = "crystal"
            colcon_packages = ["my_package"]
            colcon_packages_ignore = []
            colcon_source_space = "src"
            source_subdir = None
            colcon_cmake_args = []
            colcon_catkin_cmake_args = []
            colcon_ament_cmake_args = []
            build_attributes = []
            disable_parallel = False

        self.properties = props()
        self.ros_version = "2"
        self.ubuntu_distro = "bionic"

        self.ubuntu_mock = self.useFixture(
            fixtures.MockPatch("snapcraft.repo.Ubuntu")
        ).mock

        self.dependencies_mock = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.plugins.v1.colcon._find_system_dependencies", return_value={}
            )
        ).mock

        self.rosdep_mock = self.useFixture(
            fixtures.MockPatch("snapcraft.plugins.v1._ros.rosdep.Rosdep")
        ).mock

        self.pip_mock = self.useFixture(
            fixtures.MockPatch("snapcraft.plugins.v1._python.Pip")
        ).mock
        self.pip_mock.return_value.list.return_value = {}

    def assert_rosdep_setup(
        self,
        rosdistro,
        ros_version,
        package_path,
        rosdep_path,
        ubuntu_distro,
        base,
        target_arch,
    ):
        self.rosdep_mock.assert_has_calls(
            [
                mock.call(
                    ros_distro=rosdistro,
                    ros_version=ros_version,
                    ros_package_path=package_path,
                    rosdep_path=rosdep_path,
                    ubuntu_distro=ubuntu_distro,
                    base=base,
                    target_arch=target_arch,
                ),
                mock.call().setup(),
            ]
        )

    def assert_pip_setup(self, python_major_version, part_dir, install_dir, stage_dir):
        self.pip_mock.assert_has_calls(
            [
                mock.call(
                    python_major_version=python_major_version,
                    part_dir=part_dir,
                    install_dir=install_dir,
                    stage_dir=stage_dir,
                ),
                mock.call().setup(),
            ]
        )


class ColconPluginTest(ColconPluginTestBase):
    def test_schema(self):
        schema = colcon.ColconPlugin.schema()

        properties = schema["properties"]
        expected = (
            "colcon-rosdistro",
            "colcon-packages",
            "colcon-source-space",
            "colcon-cmake-args",
            "colcon-catkin-cmake-args",
            "colcon-ament-cmake-args",
            "colcon-packages-ignore",
        )
        self.assertThat(properties, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(properties, Contains(prop))

    def test_unsupported_base(self):
        self.project._snap_meta.base = "unsupported-base"

        raised = self.assertRaises(
            errors.PluginBaseError,
            colcon.ColconPlugin,
            "test-part",
            self.properties,
            self.project,
        )

        self.assertThat(raised.part_name, Equals("test-part"))
        self.assertThat(raised.base, Equals("unsupported-base"))

    def test_unsupported_base_and_rosdistro(self):
        self.project._snap_meta.base = "core"

        raised = self.assertRaises(
            colcon.ColconPluginBaseError,
            colcon.ColconPlugin,
            "test-part",
            self.properties,
            self.project,
        )

        self.assertThat(raised.part_name, Equals("test-part"))
        self.assertThat(raised.base, Equals("core"))
        self.assertThat(raised.rosdistro, Equals("crystal"))

    def test_schema_colcon_rosdistro(self):
        schema = colcon.ColconPlugin.schema()

        # Check colcon-rosdistro property
        colcon_packages = schema["properties"]["colcon-rosdistro"]
        expected = ("type", "default", "enum")
        self.assertThat(colcon_packages, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(colcon_packages, Contains(prop))
        self.assertThat(colcon_packages["type"], Equals("string"))
        self.assertThat(colcon_packages["default"], Equals("crystal"))

        enum = colcon_packages["enum"]
        self.assertThat(enum, HasLength(4))
        self.assertThat(enum, Contains("bouncy"))
        self.assertThat(enum, Contains("crystal"))
        self.assertThat(enum, Contains("dashing"))
        self.assertThat(enum, Contains("eloquent"))

    def test_schema_colcon_packages(self):
        schema = colcon.ColconPlugin.schema()

        # Check colcon-packages property
        colcon_packages = schema["properties"]["colcon-packages"]
        expected = ("type", "minitems", "uniqueItems", "items")
        self.assertThat(colcon_packages, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(colcon_packages, Contains(prop))
        self.assertThat(colcon_packages["type"], Equals("array"))
        self.assertThat(colcon_packages["minitems"], Equals(1))
        self.assertTrue(colcon_packages["uniqueItems"])
        self.assertThat(colcon_packages["items"], Contains("type"))
        self.assertThat(colcon_packages["items"]["type"], Equals("string"))

    def test_schema_source_space(self):
        schema = colcon.ColconPlugin.schema()

        # Check colcon-source-space property
        source_space = schema["properties"]["colcon-source-space"]
        expected = ("type", "default")
        self.assertThat(source_space, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(source_space, Contains(prop))
        self.assertThat(source_space["type"], Equals("string"))
        self.assertThat(source_space["default"], Equals("src"))

    def test_schema_colcon_cmake_args(self):
        schema = colcon.ColconPlugin.schema()

        # Check colcon-cmake-args property
        colcon_cmake_args = schema["properties"]["colcon-cmake-args"]
        expected = ("type", "default", "minitems", "items")
        self.assertThat(colcon_cmake_args, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(colcon_cmake_args, Contains(prop))
        self.assertThat(colcon_cmake_args["type"], Equals("array"))
        self.assertThat(colcon_cmake_args["default"], Equals([]))
        self.assertThat(colcon_cmake_args["minitems"], Equals(1))
        self.assertThat(colcon_cmake_args["items"], Contains("type"))
        self.assertThat(colcon_cmake_args["items"]["type"], Equals("string"))

    def test_schema_colcon_catkin_cmake_args(self):
        schema = colcon.ColconPlugin.schema()

        # Check colcon-catkin-cmake-args property
        colcon_cmake_args = schema["properties"]["colcon-catkin-cmake-args"]
        expected = ("type", "default", "minitems", "items")
        self.assertThat(colcon_cmake_args, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(colcon_cmake_args, Contains(prop))
        self.assertThat(colcon_cmake_args["type"], Equals("array"))
        self.assertThat(colcon_cmake_args["default"], Equals([]))
        self.assertThat(colcon_cmake_args["minitems"], Equals(1))
        self.assertThat(colcon_cmake_args["items"], Contains("type"))
        self.assertThat(colcon_cmake_args["items"]["type"], Equals("string"))

    def test_schema_colcon_ament_cmake_args(self):
        schema = colcon.ColconPlugin.schema()

        # Check colcon-ament-cmake-args property
        colcon_cmake_args = schema["properties"]["colcon-ament-cmake-args"]
        expected = ("type", "default", "minitems", "items")
        self.assertThat(colcon_cmake_args, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(colcon_cmake_args, Contains(prop))
        self.assertThat(colcon_cmake_args["type"], Equals("array"))
        self.assertThat(colcon_cmake_args["default"], Equals([]))
        self.assertThat(colcon_cmake_args["minitems"], Equals(1))
        self.assertThat(colcon_cmake_args["items"], Contains("type"))
        self.assertThat(colcon_cmake_args["items"]["type"], Equals("string"))

    def test_schema_colcon_packages_ignore(self):
        schema = colcon.ColconPlugin.schema()

        # check colcon-packages-ignore property
        colcon_packages_ignore = schema["properties"]["colcon-packages-ignore"]
        expected = ("type", "default", "minitems", "items", "uniqueItems")
        self.assertThat(colcon_packages_ignore, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(colcon_packages_ignore, Contains(prop))
        self.assertThat(colcon_packages_ignore["type"], Equals("array"))
        self.assertThat(colcon_packages_ignore["default"], Equals([]))
        self.assertThat(colcon_packages_ignore["minitems"], Equals(1))
        self.assertTrue(colcon_packages_ignore["uniqueItems"])
        self.assertThat(colcon_packages_ignore["items"], Contains("type"))
        self.assertThat(colcon_packages_ignore["items"]["type"], Equals("string"))

    def test_eol_ros_distro_warning(self):
        self.properties.colcon_rosdistro = "crystal"

        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)
        colcon.ColconPlugin("test-part", self.properties, self.project)
        self.assertThat(
            fake_logger.output,
            Contains(
                "The 'crystal' ROS distro has reached end-of-life and is no longer supported. Use at your own risk."
            ),
        )

    def test_get_pull_properties(self):
        expected_pull_properties = [
            "colcon-rosdistro",
            "colcon-packages",
            "colcon-source-space",
        ]
        actual_pull_properties = colcon.ColconPlugin.get_pull_properties()

        self.assertThat(
            actual_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, actual_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = [
            "colcon-cmake-args",
            "colcon-catkin-cmake-args",
            "colcon-ament-cmake-args",
            "colcon-packages-ignore",
        ]
        actual_build_properties = colcon.ColconPlugin.get_build_properties()

        self.assertThat(
            actual_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, actual_build_properties)

    def test_pull_invalid_dependency(self):
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo"}}

        self.ubuntu_mock.fetch_stage_packages.side_effect = repo.errors.PackageNotFoundError(
            "foo"
        )

        raised = self.assertRaises(colcon.ColconAptDependencyFetchError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to fetch apt dependencies: The package 'foo' was not found."
            ),
        )

    def test_clean_pull(self):
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo", "bar", "baz"}}

        plugin.pull()
        os.makedirs(plugin._rosdep_path)

        plugin.clean_pull()
        self.assertFalse(os.path.exists(plugin._rosdep_path))

    def test_valid_colcon_workspace_src(self):
        # sourcedir is expected to be the root of the Colcon workspace. Since
        # it contains a 'src' directory, this is a valid Colcon workspace.
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))
        # An exception will be raised if pull can't handle the valid workspace.
        plugin.pull()

    def test_invalid_colcon_workspace_no_src(self):
        # sourcedir is expected to be the root of the Colcon workspace. Since
        # it does not contain a `src` folder and `source-space` is 'src', this
        # should fail.
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        raised = self.assertRaises(colcon.ColconPackagePathNotFoundError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to find package path: {!r}".format(
                    os.path.join(plugin.sourcedir, "src")
                )
            ),
        )

    def test_valid_colcon_workspace_source_space(self):
        self.properties.colcon_source_space = "foo"

        # sourcedir is expected to be the root of the Colcon workspace.
        # Normally this would mean it contained a `src` directory, but it can
        # be remapped via the `colcon-source-space` key.
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, self.properties.colcon_source_space))
        # An exception will be raised if pull can't handle the source space.
        plugin.pull()

    def test_invalid_colcon_workspace_invalid_source_space(self):
        self.properties.colcon_source_space = "foo"

        # sourcedir is expected to be the root of the Colcon workspace. Since
        # it does not contain a `src` folder and source_space wasn't
        # specified, this should fail.
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        raised = self.assertRaises(colcon.ColconPackagePathNotFoundError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to find package path: {!r}".format(
                    os.path.join(plugin.sourcedir, self.properties.colcon_source_space)
                )
            ),
        )

    def test_invalid_colcon_workspace_invalid_source_space_build_all(self):
        self.properties.colcon_source_space = "foo"
        self.properties.colcon_packages = None

        # sourcedir is expected to be the root of the Colcon workspace. Since
        # it does not contain a `src` folder and source_space wasn't
        # specified, this should fail.
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        raised = self.assertRaises(colcon.ColconPackagePathNotFoundError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to find package path: {!r}".format(
                    os.path.join(plugin.sourcedir, self.properties.colcon_source_space)
                )
            ),
        )

    def test_invalid_colcon_workspace_invalid_source_no_packages(self):
        """Test that an invalid source space is fine iff no packages."""
        self.properties.colcon_source_space = "foo"
        self.properties.colcon_packages = []

        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)

        # Normally pulling should fail, but since there are no packages to
        # build, even an invalid workspace should be okay.
        plugin.pull()

    def test_invalid_colcon_workspace_source_space_same_as_source(self):
        self.properties.colcon_source_space = "."

        # sourcedir is expected to be the root of the Colcon workspace. Since
        # source_space was specified to be the same as the root, this should
        # fail.
        self.assertRaises(
            colcon.ColconWorkspaceIsRootError,
            colcon.ColconPlugin,
            "test-part",
            self.properties,
            self.project,
        )

    @mock.patch.object(colcon.ColconPlugin, "run")
    @mock.patch.object(colcon.ColconPlugin, "run_output", return_value="foo")
    @mock.patch.object(colcon.ColconPlugin, "_prepare_build")
    @mock.patch.object(colcon.ColconPlugin, "_finish_build")
    def test_build_multiple(
        self, finish_build_mock, prepare_build_mock, run_output_mock, run_mock
    ):
        self.properties.colcon_packages.append("package_2")

        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        class check_pkg_arguments:
            def __init__(self, test):
                self.test = test

            def __eq__(self, args):
                index = args.index("--packages-select")
                packages = args[index + 1 : index + 3]
                self.test.assertIn("my_package", packages)
                self.test.assertIn("package_2", packages)
                return True

        run_mock.assert_called_with(check_pkg_arguments(self))

        self.assertFalse(
            self.dependencies_mock.called,
            "Dependencies should have been discovered in the pull() step",
        )

        finish_build_mock.assert_called_once_with()

    @mock.patch.object(colcon.ColconPlugin, "run_output", return_value="bar")
    def test_run_environment(self, run_mock):
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)

        with mock.patch.object(
            plugin, "_source_setup_sh", wraps=plugin._source_setup_sh
        ) as sh_mock:
            # Joining and re-splitting to get hacked script in there as well
            environment = "\n".join(plugin.env(plugin.installdir)).split("\n")
            sh_mock.assert_called_with(plugin.installdir)

        underlay_setup = os.path.join(plugin.options.colcon_rosdistro, "setup.sh")
        overlay_setup = os.path.join("snap", "local_setup.sh")

        # Verify that the python executables and root are set before any setup.sh is
        # sourced. Also verify that the underlay setup is sourced before the overlay.
        ament_python_index = [
            i for i, line in enumerate(environment) if "AMENT_PYTHON_EXECUTABLE" in line
        ][0]
        colcon_python_index = [
            i
            for i, line in enumerate(environment)
            if "COLCON_PYTHON_EXECUTABLE" in line
        ][0]
        root_index = [
            i for i, line in enumerate(environment) if "SNAP_COLCON_ROOT" in line
        ][0]
        underlay_source_setup_index = [
            i for i, line in enumerate(environment) if underlay_setup in line
        ][0]
        overlay_source_setup_index = [
            i for i, line in enumerate(environment) if overlay_setup in line
        ][0]
        self.assertThat(ament_python_index, LessThan(underlay_source_setup_index))
        self.assertThat(colcon_python_index, LessThan(underlay_source_setup_index))
        self.assertThat(root_index, LessThan(underlay_source_setup_index))
        self.assertThat(
            underlay_source_setup_index, LessThan(overlay_source_setup_index)
        )

    def test_source_setup_sh(self):
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)

        underlay = os.path.join(
            "test-root", "opt", "ros", plugin.options.colcon_rosdistro
        )
        underlay_setup = os.path.join(underlay, "setup.sh")
        overlay = os.path.join("test-root", "opt", "ros", "snap")
        overlay_setup = os.path.join(overlay, "local_setup.sh")

        # Make sure $@ is zeroed, then setup.sh sourced, then $@ is restored
        lines_of_interest = [
            "set --",
            'if [ -f "{}" ]; then'.format(underlay_setup),
            '. "{}"'.format(underlay_setup),
            "fi",
            'if [ -f "{}" ]; then'.format(overlay_setup),
            '. "{}"'.format(overlay_setup),
            "fi",
            'eval "set -- $BACKUP_ARGS"',
        ]

        actual_lines = []

        for line in plugin._source_setup_sh("test-root").split("\n"):
            line = line.strip()
            if line in lines_of_interest:
                actual_lines.append(line)

        self.assertThat(
            actual_lines,
            Equals(lines_of_interest),
            "Expected setups to be sourced after args were zeroed, followed by the "
            "args being restored.",
        )


class PrepareBuildTest(ColconPluginTestBase):
    def setUp(self):
        super().setUp()
        self.plugin = colcon.ColconPlugin("test-part", self.properties, self.project)

    @mock.patch("snapcraft.internal.mangling.rewrite_python_shebangs")
    def test_in_snap_python_is_used(self, shebangs_mock):
        # Mangling has its own tests. Here we just need to make sure
        # _prepare_build actually uses it.
        self.plugin._prepare_build()
        shebangs_mock.assert_called_once_with(self.plugin.installdir)

    def test_underlay_cmake_paths_are_rewritten(self):
        os.makedirs(os.path.join(self.plugin._ros_underlay, "test"))

        # Place a few .cmake files with incorrect paths, and some files that
        # shouldn't be changed.
        files = [
            {
                "path": "fooConfig.cmake",
                "contents": '"/usr/lib/foo"',
                "expected": '"{}/usr/lib/foo"'.format(self.plugin.installdir),
            },
            {
                "path": "bar.cmake",
                "contents": '"/usr/lib/bar"',
                "expected": '"/usr/lib/bar"',
            },
            {
                "path": "test/bazConfig.cmake",
                "contents": '"/test/baz;/usr/lib/baz"',
                "expected": '"{0}/test/baz;{0}/usr/lib/baz"'.format(
                    self.plugin.installdir
                ),
            },
            {"path": "test/quxConfig.cmake", "contents": "qux", "expected": "qux"},
            {
                "path": "test/installedConfig.cmake",
                "contents": '"{}/foo"'.format(self.plugin.installdir),
                "expected": '"{}/foo"'.format(self.plugin.installdir),
            },
        ]

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._prepare_build()

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    def test_cmake_paths_are_rewritten(self):
        # Place a few .cmake files with incorrect paths, and some files that
        # shouldn't be changed.
        files = [
            {
                "path": "fooConfig.cmake",
                "contents": '"/usr/lib/foo"',
                "expected": '"{}/usr/lib/foo"'.format(self.plugin.installdir),
            },
            {"path": "bar", "contents": '"/usr/lib/bar"', "expected": '"/usr/lib/bar"'},
            {
                "path": "test/bazConfig.cmake",
                "contents": '"/test/baz;/usr/lib/baz"',
                "expected": '"{0}/test/baz;{0}/usr/lib/baz"'.format(
                    self.plugin.installdir
                ),
            },
            {"path": "test/quxConfig.cmake", "contents": "qux", "expected": "qux"},
            {
                "path": "test/installedConfig.cmake",
                "contents": '"{}/foo"'.format(self.plugin.installdir),
                "expected": '"{}/foo"'.format(self.plugin.installdir),
            },
            {
                "path": "test/poco.cmake",
                "contents": 'INTERFACE_LINK_LIBRARIES "pthread;dl;rt;/usr/lib/x86_64-linux-gnu/libpcre.so;/usr/lib/x86_64-linux-gnu/libz.so"',
                "expected": 'INTERFACE_LINK_LIBRARIES "pthread;dl;rt;{0}/usr/lib/x86_64-linux-gnu/libpcre.so;{0}/usr/lib/x86_64-linux-gnu/libz.so"'.format(
                    self.plugin.installdir
                ),
            },
        ]

        for file_info in files:
            path = os.path.join(
                self.plugin.installdir, "usr", "lib", "cmake", file_info["path"]
            )
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._prepare_build()

        for file_info in files:
            path = os.path.join(
                self.plugin.installdir, "usr", "lib", "cmake", file_info["path"]
            )
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    def test_ament_prefix_is_rewritten(self):
        os.makedirs(os.path.join(self.plugin._ros_underlay, "test"))

        # Place files with incorrect prefixes, and some that shouldn't be changed.
        files = [
            {
                "path": "setup.sh",
                "contents": ": ${AMENT_CURRENT_PREFIX:=/opt/ros/crystal}",
                "expected": ': ${AMENT_CURRENT_PREFIX:="$SNAP_COLCON_ROOT/opt/ros/crystal"}',
            },
            {
                "path": "test/local_setup.sh",
                "contents": ': ${AMENT_CURRENT_PREFIX:="/foo/bar"}',
                "expected": ': ${AMENT_CURRENT_PREFIX:="$SNAP_COLCON_ROOT/foo/bar"}',
            },
            {
                "path": "test/no-change.sh",
                "contents": ": ${AMENT_CURRENT_PREFIX:=$NOT_A_PATH}",
                "expected": ": ${AMENT_CURRENT_PREFIX:=$NOT_A_PATH}",
            },
        ]

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._prepare_build()

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))


class FinishBuildTest(ColconPluginTestBase):
    def setUp(self):
        super().setUp()
        self.plugin = colcon.ColconPlugin("test-part", self.properties, self.project)

    @mock.patch("snapcraft.internal.mangling.rewrite_python_shebangs")
    def test_in_snap_python_is_used(self, shebangs_mock):
        # Mangling has its own tests. Here we just need to make sure
        # _prepare_build actually uses it.
        self.plugin._finish_build()
        shebangs_mock.assert_called_once_with(self.plugin.installdir)

    def test_cmake_paths_are_rewritten(self):
        os.makedirs(os.path.join(self.plugin._ros_underlay, "test"))

        # Place a few .cmake files with incorrect paths, and some files that
        # shouldn't be changed.
        files = [
            {
                "path": "fooConfig.cmake",
                "contents": '"{}/usr/lib/foo"'.format(self.plugin.installdir),
                "expected": '"$ENV{SNAPCRAFT_STAGE}/usr/lib/foo"',
            },
            {
                "path": "bar.cmake",
                "contents": '"/usr/lib/bar"',
                "expected": '"/usr/lib/bar"',
            },
            {
                "path": "test/bazConfig.cmake",
                "contents": '"{0}/test/baz;{0}/usr/lib/baz"'.format(
                    self.plugin.installdir
                ),
                "expected": '"$ENV{SNAPCRAFT_STAGE}/test/baz;'
                '$ENV{SNAPCRAFT_STAGE}/usr/lib/baz"',
            },
            {"path": "test/quxConfig.cmake", "contents": "qux", "expected": "qux"},
            {
                "path": "test/installedConfig.cmake",
                "contents": '"$ENV{SNAPCRAFT_STAGE}/foo"',
                "expected": '"$ENV{SNAPCRAFT_STAGE}/foo"',
            },
        ]

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._finish_build()

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

        # Verify that no sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            Not(FileExists()),
        )

    def test_ament_prefix_is_rewritten(self):
        os.makedirs(os.path.join(self.plugin._ros_underlay, "test"))

        # Place files with incorrect prefixes, and some that shouldn't be changed.
        files = [
            {
                "path": "setup.sh",
                "contents": ": ${AMENT_CURRENT_PREFIX:=/opt/ros/crystal}",
                "expected": ': ${AMENT_CURRENT_PREFIX:="$SNAP_COLCON_ROOT/opt/ros/crystal"}',
            },
            {
                "path": "test/local_setup.sh",
                "contents": ': ${{AMENT_CURRENT_PREFIX:="{}/foo/bar"}}'.format(
                    self.plugin.installdir
                ),
                "expected": ': ${AMENT_CURRENT_PREFIX:="$SNAP_COLCON_ROOT/foo/bar"}',
            },
            {
                "path": "test/no-change.sh",
                "contents": ": ${AMENT_CURRENT_PREFIX:=$NOT_A_PATH}",
                "expected": ": ${AMENT_CURRENT_PREFIX:=$NOT_A_PATH}",
            },
        ]

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._finish_build()

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    def test_colcon_prefix_is_rewritten(self):
        os.makedirs(os.path.join(self.plugin._ros_underlay, "test"))

        # Place files with incorrect prefixes, and some that shouldn't be changed.
        files = [
            {
                "path": "setup.sh",
                "contents": "_colcon_prefix_sh_COLCON_CURRENT_PREFIX=/foo/bar",
                "expected": '_colcon_prefix_sh_COLCON_CURRENT_PREFIX="$SNAP_COLCON_ROOT/foo/bar"',
            },
            {
                "path": "test/local_setup.sh",
                "contents": '_colcon_package_sh_COLCON_CURRENT_PREFIX="{}/baz"'.format(
                    self.plugin.installdir
                ),
                "expected": '_colcon_package_sh_COLCON_CURRENT_PREFIX="$SNAP_COLCON_ROOT/baz"',
            },
            {
                "path": "test/another_local_setup.sh",
                "contents": 'COLCON_CURRENT_PREFIX="{}/qux"'.format(
                    self.plugin.installdir
                ),
                "expected": 'COLCON_CURRENT_PREFIX="$SNAP_COLCON_ROOT/qux"',
            },
            {
                "path": "test/no-change1.sh",
                "contents": "_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=$NOT_A_PATH",
                "expected": "_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=$NOT_A_PATH",
            },
            {
                "path": "test/no-change2.sh",
                "contents": "_colcon_package_bash_COLCON_CURRENT_PREFIX=/foo/bar",
                "expected": "_colcon_package_bash_COLCON_CURRENT_PREFIX=/foo/bar",
            },
            {
                "path": "test/no-change3.sh",
                "contents": "COLCON_CURRENT_PREFIX=/foo/bar",
                "expected": "COLCON_CURRENT_PREFIX=/foo/bar",
            },
        ]

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._finish_build()

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    def test_colcon_python_executable_is_rewritten(self):
        os.makedirs(os.path.join(self.plugin._ros_underlay, "test"))

        # Place files with incorrect prefixes, and some that shouldn't be changed.
        files = [
            {
                "path": "setup.sh",
                "contents": "_colcon_python_executable=/usr/bin/python3",
                "expected": '_colcon_python_executable="$SNAP_COLCON_ROOT/usr/bin/python3"',
            },
            {
                "path": "test/local_setup.sh",
                "contents": '_colcon_python_executable="{}/baz/python3"'.format(
                    self.plugin.installdir
                ),
                "expected": '_colcon_python_executable="$SNAP_COLCON_ROOT/baz/python3"',
            },
            {
                "path": "test/no-change.sh",
                "contents": "_colcon_python_executable=$NOT_A_PATH",
                "expected": "_colcon_python_executable=$NOT_A_PATH",
            },
        ]

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._finish_build()

        for file_info in files:
            path = os.path.join(self.plugin._ros_underlay, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    @mock.patch.object(colcon.ColconPlugin, "run")
    @mock.patch.object(colcon.ColconPlugin, "run_output", return_value="foo")
    def test_finish_build_python_sitecustomize(self, run_output_mock, run_mock):
        self.pip_mock.return_value.list.return_value = {"test-package"}

        # Create site.py, indicating that python3 was a stage-package
        site_py_path = os.path.join(
            self.plugin.installdir, "usr", "lib", "python3.6", "site.py"
        )
        os.makedirs(os.path.dirname(site_py_path), exist_ok=True)
        open(site_py_path, "w").close()

        # Also create python3 site-packages, indicating that pip packages were
        # installed.
        os.makedirs(
            os.path.join(self.plugin.installdir, "lib", "python3.6", "site-packages"),
            exist_ok=True,
        )

        self.plugin._finish_build()

        # Verify that sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python3.6", "sitecustomize.py"
            ),
            FileExists(),
        )


class PullTestCase(ColconPluginTestBase):
    def test_pull_debian_dependencies(self):
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo", "bar", "baz"}}

        plugin.pull()

        self.assert_rosdep_setup(
            plugin.options.colcon_rosdistro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
            plugin.project._get_build_base(),
            plugin.project._get_stage_packages_target_arch(),
        )

        # Verify that dependencies were found as expected. TODO: Would really
        # like to use ANY here instead of verifying explicit arguments, but
        # Python issue #25195 won't let me.
        self.assertThat(self.dependencies_mock.call_count, Equals(1))
        self.assertThat(self.dependencies_mock.call_args[0][0], Equals({"my_package"}))

        # Verify that the dependencies were installed
        self.assertThat(
            self.ubuntu_mock.fetch_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=plugin.stage_packages_path,
                        package_names={"bar", "baz", "foo"},
                        base=plugin.project._get_build_base(),
                        target_arch=plugin.project._get_stage_packages_target_arch(),
                    )
                ]
            ),
        )
        self.assertThat(
            self.ubuntu_mock.unpack_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=plugin.stage_packages_path,
                        install_path=pathlib.Path(plugin.installdir),
                    )
                ]
            ),
        )

    def test_pull_local_dependencies(self):
        self.properties.colcon_packages.append("package_2")
        self.properties.source_subdir = "subdir"

        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "subdir", "src"))

        # No system dependencies (only local)
        self.dependencies_mock.return_value = {}

        plugin.pull()

        self.assert_rosdep_setup(
            plugin.options.colcon_rosdistro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "subdir", "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
            plugin.project._get_build_base(),
            plugin.project._get_stage_packages_target_arch(),
        )

        # Verify that dependencies were found as expected. TODO: Would really
        # like to use ANY here instead of verifying explicit arguments, but
        # Python issue #25195 won't let me.
        self.assertThat(self.dependencies_mock.call_count, Equals(1))
        self.assertThat(
            self.dependencies_mock.call_args[0][0], Equals({"my_package", "package_2"})
        )

        # Verify that no .deb packages were installed
        self.assertTrue(
            mock.call().unpack(plugin.installdir) not in self.ubuntu_mock.mock_calls
        )

    def test_pull_pip_dependencies(self):
        plugin = colcon.ColconPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"pip": {"foo", "bar", "baz"}}

        plugin.pull()

        self.assert_rosdep_setup(
            plugin.options.colcon_rosdistro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
            plugin.project._get_build_base(),
            plugin.project._get_stage_packages_target_arch(),
        )

        self.assert_pip_setup(
            "3", plugin.partdir, plugin.installdir, plugin.project.stage_dir
        )

        # Verify that dependencies were found as expected. TODO: Would really
        # like to use ANY here instead of verifying explicit arguments, but
        # Python issue #25195 won't let me.
        self.assertThat(self.dependencies_mock.call_count, Equals(1))
        self.assertThat(self.dependencies_mock.call_args[0][0], Equals({"my_package"}))

        # Verify that the pip dependencies were installed
        self.pip_mock.return_value.download.assert_called_once_with(
            {"foo", "bar", "baz"}
        )
        self.pip_mock.return_value.install.assert_called_once_with(
            {"foo", "bar", "baz"}
        )


@pytest.fixture
def options():
    class Options:
        colcon_rosdistro = "crystal"
        colcon_packages = ["my_package"]
        colcon_packages_ignore = []
        colcon_source_space = "src"
        source_subdir = None
        colcon_cmake_args = []
        colcon_catkin_cmake_args = []
        colcon_ament_cmake_args = []
        build_attributes = []
        disable_parallel = False

    return Options()


class TestBuildArgs:

    package_scenarios = [
        ("one package", {"colcon_packages": ["my_package"]}),
        ("no packages", {"colcon_packages": []}),
        ("all packages", {"colcon_packages": None}),
    ]

    build_type_scenarios = [
        ("release", {"build_attributes": []}),
        ("debug", {"build_attributes": ["debug"]}),
    ]

    cmake_args_scenarios = [
        ("with colcon-cmake-args", {"colcon_cmake_args": ["-DCMAKE"]}),
        ("without colcon-cmake-args", {"colcon_cmake_args": []}),
    ]

    catkin_args_scenarios = [
        ("with catkin-cmake-args", {"colcon_catkin_cmake_args": ["-DCATKIN"]}),
        ("without catkin-cmake-args", {"colcon_catkin_cmake_args": []}),
    ]

    ament_args_scenarios = [
        ("with ament-cmake-args", {"colcon_ament_cmake_args": ["-DAMENT"]}),
        ("without ament-cmake-args", {"colcon_ament_cmake_args": []}),
    ]

    packages_ignore_scenarios = [
        ("with packages-ignore", {"colcon_packages_ignore": ["my_package"]}),
        ("without packages-ignore", {"colcon_packages_ignore": []}),
    ]

    parallel_scenarios = (
        ("disabled", dict(disable_parallel=True)),
        ("enabled", dict(disable_parallel=False)),
    )
    scenarios = multiply_scenarios(
        package_scenarios,
        build_type_scenarios,
        cmake_args_scenarios,
        catkin_args_scenarios,
        ament_args_scenarios,
        packages_ignore_scenarios,
        parallel_scenarios,
    )

    @mock.patch.object(colcon.ColconPlugin, "run")
    @mock.patch.object(colcon.ColconPlugin, "run_output", return_value="foo")
    @mock.patch.object(colcon.ColconPlugin, "_prepare_build")
    @mock.patch.object(colcon.ColconPlugin, "_finish_build")
    def test_build(
        self,
        finish_build_mock,
        prepare_build_mock,
        run_output_mock,
        run_mock,
        tmp_work_path,
        project_core18,
        options,
        disable_parallel,
        build_attributes,
        colcon_packages_ignore,
        colcon_ament_cmake_args,
        colcon_catkin_cmake_args,
        colcon_cmake_args,
        colcon_packages,
    ):
        options.colcon_packages = colcon_packages
        options.colcon_packages_ignore = colcon_packages_ignore
        options.build_attributes.extend(build_attributes)
        options.colcon_cmake_args = colcon_cmake_args
        options.colcon_catkin_cmake_args = colcon_catkin_cmake_args
        options.colcon_ament_cmake_args = colcon_ament_cmake_args
        options.disable_parallel = disable_parallel

        plugin = colcon.ColconPlugin("test-part", options, project_core18)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        prepare_build_mock.assert_called_once_with()

        if "debug" in build_attributes:
            build_type = "Debug"
        else:
            build_type = "Release"

        expected_command = ["colcon", "build", "--merge-install"]
        if colcon_packages:
            expected_command += ["--packages-select"] + colcon_packages
        if colcon_packages_ignore:
            expected_command += ["--packages-ignore"] + colcon_packages_ignore
        expected_command += [
            "--build-base",
            plugin.builddir,
            "--base-paths",
            f"{plugin.sourcedir}/src",
            "--install-base",
            f"{plugin.installdir}/opt/ros/snap",
            "--parallel-workers={}".format(1 if disable_parallel else 2),
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={build_type}",
        ]
        if colcon_cmake_args:
            expected_command += ["-DCMAKE"]
        if colcon_catkin_cmake_args:
            expected_command += ["--catkin-cmake-args", "-DCATKIN"]
        if colcon_ament_cmake_args:
            expected_command += ["--ament-cmake-args", "-DAMENT"]

        if colcon_packages or colcon_packages is None:
            assert run_mock.mock_calls[0][1][0] == expected_command

        else:
            run_mock.assert_not_called()

        finish_build_mock.assert_called_once_with()


class FindSystemDependenciesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.rosdep_mock = mock.MagicMock()
        self.rosdep_mock.get_dependencies.return_value = {"bar"}

    def test_find_system_dependencies_system_only(self):
        self.rosdep_mock.resolve_dependency.return_value = {"apt": {"baz"}}

        self.assertThat(
            colcon._find_system_dependencies({"foo"}, self.rosdep_mock),
            Equals({"apt": {"baz"}}),
        )

        self.rosdep_mock.get_dependencies.assert_called_once_with("foo")
        self.rosdep_mock.resolve_dependency.assert_called_once_with("bar")

    def test_find_system_dependencies_system_only_no_packages(self):
        self.rosdep_mock.resolve_dependency.return_value = {"apt": {"baz"}}

        self.assertThat(
            colcon._find_system_dependencies(None, self.rosdep_mock),
            Equals({"apt": {"baz"}}),
        )

        self.rosdep_mock.get_dependencies.assert_called_once_with()
        self.rosdep_mock.resolve_dependency.assert_called_once_with("bar")

    def test_find_system_dependencies_local_only(self):
        self.assertThat(
            colcon._find_system_dependencies({"foo", "bar"}, self.rosdep_mock),
            HasLength(0),
        )

        self.rosdep_mock.get_dependencies.assert_has_calls(
            [mock.call("foo"), mock.call("bar")], any_order=True
        )
        self.rosdep_mock.resolve_dependency.assert_not_called()

    def test_find_system_dependencies_missing_local_dependency(self):
        # Setup a dependency on a non-existing package, and it doesn't resolve
        # to a system dependency.'
        exception = _ros.rosdep.RosdepDependencyNotResolvedError("foo")
        self.rosdep_mock.resolve_dependency.side_effect = exception

        raised = self.assertRaises(
            colcon.ColconInvalidSystemDependencyError,
            colcon._find_system_dependencies,
            {"foo"},
            self.rosdep_mock,
        )

        self.assertThat(
            str(raised),
            Equals(
                "Package 'bar' isn't a valid system dependency. Did "
                "you forget to add it to colcon-packages? If not, "
                "add the Ubuntu package containing it to "
                "stage-packages until you can get it into the rosdep "
                "database."
            ),
        )

    def test_find_system_dependencies_raises_if_unsupported_type(self):
        self.rosdep_mock.resolve_dependency.return_value = {"unsupported-type": {"baz"}}

        raised = self.assertRaises(
            colcon.ColconUnsupportedDependencyTypeError,
            colcon._find_system_dependencies,
            {"foo"},
            self.rosdep_mock,
        )

        self.assertThat(
            str(raised),
            Equals(
                "Package 'bar' resolved to an unsupported type of dependency: "
                "'unsupported-type'."
            ),
        )
