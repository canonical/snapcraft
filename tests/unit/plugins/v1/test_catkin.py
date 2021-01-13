# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

import ast
import builtins
import os
import pathlib
import subprocess
import sys
import tempfile
import textwrap
from unittest import mock

import pytest
import testtools
from testtools.matchers import (
    Contains,
    Equals,
    FileExists,
    HasLength,
    LessThan,
    MatchesRegex,
    Not,
)

import snapcraft
from snapcraft import repo
from snapcraft.internal import errors
from snapcraft.plugins.v1 import _ros, catkin
from tests import unit

from . import PluginsV1BaseTestCase


class CatkinPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        # Most Catkin test use core.
        self.project._snap_meta.base = "core"

        class props:
            catkin_packages = ["my_package"]
            source_space = "src"
            source_subdir = None
            include_roscore = False
            catkin_cmake_args = []
            underlay = None
            rosinstall_files = None
            recursive_rosinstall = False
            build_attributes = []
            catkin_ros_master_uri = "http://localhost:11311"
            disable_parallel = False

        self.properties = props()
        self.ros_distro = "kinetic"
        self.ros_version = "1"
        self.ubuntu_distro = "xenial"

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.plugins.v1.catkin._find_system_dependencies", return_value={}
        )
        self.dependencies_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.plugins.v1._ros.rosdep.Rosdep")
        self.rosdep_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.plugins.v1.catkin._Catkin")
        self.catkin_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.plugins.v1._ros.wstool.Wstool")
        self.wstool_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.plugins.v1._python.Pip")
        self.pip_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.pip_mock.return_value.list.return_value = {}

    def assert_rosdep_setup(
        self, rosdistro, ros_version, package_path, rosdep_path, ubuntu_distro
    ):
        self.rosdep_mock.assert_has_calls(
            [
                mock.call(
                    ros_distro=rosdistro,
                    ros_version=ros_version,
                    ros_package_path=package_path,
                    rosdep_path=rosdep_path,
                    ubuntu_distro=ubuntu_distro,
                    base="core",
                    target_arch=self.project._get_stage_packages_target_arch(),
                ),
                mock.call().setup(),
            ]
        )

    def assert_wstool_setup(self, package_path, wstool_path):
        self.wstool_mock.assert_has_calls(
            [
                mock.call(package_path, wstool_path, self.project, "core"),
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


class CatkinPluginTestCase(CatkinPluginBaseTest):
    def test_schema(self):
        schema = catkin.CatkinPlugin.schema()

        properties = schema["properties"]
        expected = (
            "catkin-packages",
            "source-space",
            "include-roscore",
            "catkin-cmake-args",
            "underlay",
            "rosinstall-files",
            "recursive-rosinstall",
            "catkin-ros-master-uri",
        )
        self.assertThat(properties, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(properties, Contains(prop))

    def test_schema_catkin_packages(self):
        schema = catkin.CatkinPlugin.schema()

        # Check catkin-packages property
        catkin_packages = schema["properties"]["catkin-packages"]
        expected = ("type", "minitems", "uniqueItems", "items")
        self.assertThat(catkin_packages, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(catkin_packages, Contains(prop))
        self.assertThat(catkin_packages["type"], Equals("array"))
        self.assertThat(catkin_packages["minitems"], Equals(1))
        self.assertTrue(catkin_packages["uniqueItems"])
        self.assertThat(catkin_packages["items"], Contains("type"))
        self.assertThat(catkin_packages["items"]["type"], Equals("string"))

    def test_schema_source_space(self):
        schema = catkin.CatkinPlugin.schema()

        # Check source-space property
        source_space = schema["properties"]["source-space"]
        expected = ("type", "default")
        self.assertThat(source_space, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(source_space, Contains(prop))
        self.assertThat(source_space["type"], Equals("string"))
        self.assertThat(source_space["default"], Equals("src"))

    def test_schema_include_roscore(self):
        schema = catkin.CatkinPlugin.schema()

        # Check include-roscore property
        include_roscore = schema["properties"]["include-roscore"]
        expected = ("type", "default")
        self.assertThat(include_roscore, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(include_roscore, Contains(prop))
        self.assertThat(include_roscore["type"], Equals("boolean"))
        self.assertThat(include_roscore["default"], Equals(True))

    def test_schema_catkin_catkin_cmake_args(self):
        schema = catkin.CatkinPlugin.schema()

        # Check catkin-cmake-args property
        catkin_cmake_args = schema["properties"]["catkin-cmake-args"]
        expected = ("type", "default", "minitems", "items")
        self.assertThat(catkin_cmake_args, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(catkin_cmake_args, Contains(prop))
        self.assertThat(catkin_cmake_args["type"], Equals("array"))
        self.assertThat(catkin_cmake_args["default"], Equals([]))
        self.assertThat(catkin_cmake_args["minitems"], Equals(1))
        self.assertThat(catkin_cmake_args["items"], Contains("type"))
        self.assertThat(catkin_cmake_args["items"]["type"], Equals("string"))

    def test_schema_underlay(self):
        schema = catkin.CatkinPlugin.schema()

        # Check underlay property
        underlay = schema["properties"]["underlay"]
        expected = ("type", "properties", "required")
        self.assertThat(underlay, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(underlay, Contains(prop))
        self.assertThat(underlay["type"], Equals("object"))

        underlay_required = underlay["required"]
        expected = ("build-path", "run-path")
        self.assertThat(underlay_required, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(underlay_required, Contains(prop))

        underlay_properties = underlay["properties"]
        expected = ("build-path", "run-path")
        self.assertThat(underlay_properties, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(underlay_properties, Contains(prop))
        underlay_build_path = underlay_properties["build-path"]
        self.assertThat(underlay_build_path, Contains("type"))
        self.assertThat(underlay_build_path["type"], Equals("string"))
        underlay_run_path = underlay_properties["run-path"]
        self.assertThat(underlay_run_path, Contains("type"))
        self.assertThat(underlay_run_path["type"], Equals("string"))

    def test_schema_rosinstall_files(self):
        schema = catkin.CatkinPlugin.schema()

        # Check rosinstall-files property
        rosinstall_files = schema["properties"]["rosinstall-files"]
        expected = ("type", "default", "minitems", "uniqueItems", "items")
        self.assertThat(rosinstall_files, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(rosinstall_files, Contains(prop))
        self.assertThat(rosinstall_files["type"], Equals("array"))
        self.assertThat(rosinstall_files["default"], Equals([]))
        self.assertThat(rosinstall_files["minitems"], Equals(1))
        self.assertTrue(rosinstall_files["uniqueItems"])
        self.assertThat(rosinstall_files["items"], Contains("type"))
        self.assertThat(rosinstall_files["items"]["type"], Equals("string"))

    def test_schema_recursive_rosinstall(self):
        schema = catkin.CatkinPlugin.schema()

        # Check recursive-rosinstall property
        recursive_rosinstall = schema["properties"]["recursive-rosinstall"]
        expected = ("type", "default")
        self.assertThat(recursive_rosinstall, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(recursive_rosinstall, Contains(prop))
        self.assertThat(recursive_rosinstall["type"], Equals("boolean"))
        self.assertThat(recursive_rosinstall["default"], Equals(False))

    def test_schema_catkin_ros_master_uri(self):
        schema = catkin.CatkinPlugin.schema()

        # Check ros-master-uri property
        catkin_ros_master_uri = schema["properties"]["catkin-ros-master-uri"]
        expected = ("type", "default")
        self.assertThat(catkin_ros_master_uri, HasLength(len(expected)))
        for prop in expected:
            self.assertThat(catkin_ros_master_uri, Contains(prop))
        self.assertThat(catkin_ros_master_uri["type"], Equals("string"))
        self.assertThat(
            catkin_ros_master_uri["default"], Equals("http://localhost:11311")
        )

    def test_get_pull_properties(self):
        expected_pull_properties = [
            "catkin-packages",
            "source-space",
            "include-roscore",
            "underlay",
            "rosinstall-files",
            "recursive-rosinstall",
        ]
        actual_pull_properties = catkin.CatkinPlugin.get_pull_properties()

        self.assertThat(
            actual_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, actual_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ["catkin-cmake-args"]
        actual_build_properties = catkin.CatkinPlugin.get_build_properties()

        self.assertThat(
            actual_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, actual_build_properties)

    def test_pull_invalid_dependency(self):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo"}}

        self.ubuntu_mock.fetch_stage_packages.side_effect = repo.errors.PackageNotFoundError(
            "foo"
        )

        raised = self.assertRaises(catkin.CatkinAptDependencyFetchError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to fetch apt dependencies: The package 'foo' was not found."
            ),
        )

    def test_pull_unable_to_resolve_roscore(self):
        self.properties.include_roscore = True
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies
        self.dependencies_mock.return_value = {}

        self.rosdep_mock.return_value.resolve_dependency.return_value = None

        self.assertRaises(catkin.CatkinCannotResolveRoscoreError, plugin.pull)

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_invalid_underlay(self, generate_setup_mock):
        self.properties.underlay = {
            "build-path": "test-build-path",
            "run-path": "test-run-path",
        }
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies
        self.dependencies_mock.return_value = {}

        raised = self.assertRaises(errors.SnapcraftEnvironmentError, plugin.pull)

        self.assertThat(
            str(raised),
            MatchesRegex(".*Requested underlay.*does not point to a valid directory"),
        )

    def test_clean_pull(self):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo", "bar", "baz"}}

        plugin.pull()
        os.makedirs(plugin._rosdep_path)

        plugin.clean_pull()
        self.assertFalse(os.path.exists(plugin._rosdep_path))

    def test_valid_catkin_workspace_src(self):
        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it contains a 'src' directory, this is a valid Catkin workspace.
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))
        # An exception will be raised if pull can't handle the valid workspace.
        plugin.pull()

    def test_invalid_catkin_workspace_no_src(self):
        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and `source-space` is 'src', this
        # should fail.
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        raised = self.assertRaises(catkin.CatkinPackagePathNotFoundError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to find package path: {!r}".format(
                    os.path.join(plugin.sourcedir, "src")
                )
            ),
        )

    def test_valid_catkin_workspace_source_space(self):
        self.properties.source_space = "foo"

        # sourcedir is expected to be the root of the Catkin workspace.
        # Normally this would mean it contained a `src` directory, but it can
        # be remapped via the `source-space` key.
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, self.properties.source_space))
        # An exception will be raised if pull can't handle the source space.
        plugin.pull()

    def test_invalid_catkin_workspace_invalid_source_space(self):
        self.properties.source_space = "foo"

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and source_space wasn't
        # specified, this should fail.
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        raised = self.assertRaises(catkin.CatkinPackagePathNotFoundError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to find package path: {!r}".format(
                    os.path.join(plugin.sourcedir, self.properties.source_space)
                )
            ),
        )

    def test_invalid_catkin_workspace_invalid_source_space_build_all(self):
        self.properties.source_space = "foo"
        self.properties.catkin_packages = None

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and source_space wasn't
        # specified, this should fail.
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        raised = self.assertRaises(catkin.CatkinPackagePathNotFoundError, plugin.pull)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to find package path: {!r}".format(
                    os.path.join(plugin.sourcedir, self.properties.source_space)
                )
            ),
        )

    def test_invalid_catkin_workspace_invalid_source_no_packages(self):
        """Test that an invalid source space is fine iff no packages."""
        self.properties.source_space = "foo"
        self.properties.catkin_packages = []

        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        # Normally pulling should fail, but since there are no packages to
        # build, even an invalid workspace should be okay.
        plugin.pull()

    def test_invalid_catkin_workspace_source_space_same_as_source(self):
        self.properties.source_space = "."

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # source_space was specified to be the same as the root, this should
        # fail.
        self.assertRaises(
            catkin.CatkinWorkspaceIsRootError,
            catkin.CatkinPlugin,
            "test-part",
            self.properties,
            self.project,
        )

    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_prepare_build")
    @mock.patch.object(catkin.CatkinPlugin, "_finish_build")
    def test_build_multiple(
        self, finish_build_mock, prepare_build_mock, run_output_mock, run_mock
    ):
        self.properties.catkin_packages.append("package_2")

        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        class check_pkg_arguments:
            def __init__(self, test):
                self.test = test

            def __eq__(self, args):
                index = args.index("--pkg")
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

    def test_use_in_snap_python_rewrites_shebangs(self):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.rosdir, "bin"))

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                "path": os.path.join(plugin.rosdir, "_setup_util.py"),
                "contents": "#!/foo/bar/baz/python",
                "expected": "#!/usr/bin/env python",
            },
            {
                "path": os.path.join(plugin.rosdir, "bin/catkin"),
                "contents": "#!/foo/baz/python",
                "expected": "#!/usr/bin/env python",
            },
            {
                "path": os.path.join(plugin.rosdir, "foo"),
                "contents": "foo",
                "expected": "foo",
            },
        ]

        for file_info in files:
            with open(file_info["path"], "w") as f:
                f.write(file_info["contents"])

        plugin._use_in_snap_python()

        for file_info in files:
            with open(os.path.join(plugin.rosdir, file_info["path"]), "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    def test_use_in_snap_python_skips_binarys(self, run_output_mock, run_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(plugin.rosdir)

        # Place a file to be discovered by _use_in_snap_python().
        open(os.path.join(plugin.rosdir, "foo"), "w").close()

        file_mock = mock.mock_open()
        with mock.patch.object(builtins, "open", file_mock):
            # Reading a binary file may throw a UnicodeDecodeError. Make sure
            # that's handled.
            file_mock.return_value.read.side_effect = UnicodeDecodeError(
                "foo", b"bar", 1, 2, "baz"
            )
            # An exception will be raised if the function can't handle the
            # binary file.
            plugin._use_in_snap_python()

    def test_use_in_snap_python_rewrites_10_ros_sh(self):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.rosdir, "etc", "catkin", "profile.d"))

        ros_profile = os.path.join(
            plugin.rosdir, "etc", "catkin", "profile.d", "10.ros.sh"
        )

        # Place 10.ros.sh with an absolute path to python
        with open(ros_profile, "w") as f:
            f.write("/usr/bin/python foo")

        plugin._use_in_snap_python()

        # Verify that the absolute path in 10.ros.sh was rewritten correctly
        with open(ros_profile, "r") as f:
            self.assertThat(
                f.read(),
                Equals("python foo"),
                "The absolute path to python was not replaced as expected",
            )

    def test_use_in_snap_python_rewrites_1_ros_package_path_sh(self):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.rosdir, "etc", "catkin", "profile.d"))

        ros_profile = os.path.join(
            plugin.rosdir, "etc", "catkin", "profile.d", "1.ros_package_path.sh"
        )

        # Place 1.ros_package_path.sh with an absolute path to python
        with open(ros_profile, "w") as f:
            f.write("/usr/bin/python foo")

        plugin._use_in_snap_python()

        # Verify that the absolute path in 1.ros_package_path.sh was rewritten
        # correctly
        with open(ros_profile, "r") as f:
            self.assertThat(
                f.read(),
                Equals("python foo"),
                "The absolute path to python was not replaced as expected",
            )

    def _verify_run_environment(self, plugin):
        python_path = os.path.join(
            plugin.installdir, "usr", "lib", "python2.7", "dist-packages"
        )
        os.makedirs(python_path)

        # Joining and re-splitting to get hacked script in there as well
        environment = "\n".join(plugin.env(plugin.installdir)).split("\n")

        self.assertThat(
            environment,
            Contains("PYTHONPATH={}${{PYTHONPATH:+:$PYTHONPATH}}".format(python_path)),
        )

        self.assertThat(
            environment,
            Contains("ROS_MASTER_URI={}".format(self.properties.catkin_ros_master_uri)),
        )

        self.assertThat(environment, Contains("ROS_HOME=${SNAP_USER_DATA:-/tmp}/ros"))

        self.assertThat(environment, Contains("LC_ALL=C.UTF-8"))

        # Verify that LD_LIBRARY_PATH was set before setup.sh is sourced
        ld_library_path_index = [
            i for i, line in enumerate(environment) if "LD_LIBRARY_PATH" in line
        ][0]
        source_setup_index = [
            i for i, line in enumerate(environment) if "setup.sh" in line
        ][0]
        self.assertThat(ld_library_path_index, LessThan(source_setup_index))

        return environment

    @mock.patch.object(
        catkin.CatkinPlugin, "_source_setup_sh", return_value="test-source-setup.sh"
    )
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="bar")
    def test_run_environment(self, run_mock, source_setup_sh_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        environment = self._verify_run_environment(plugin)

        source_setup_sh_mock.assert_called_with(plugin.installdir, None)
        self.assertThat(environment, Contains("test-source-setup.sh"))

    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="bar")
    def test_run_environment_with_underlay(self, run_mock):
        self.properties.underlay = {
            "build-path": "test-build-path",
            "run-path": "test-run-path",
        }
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        environment = self._verify_run_environment(plugin)

        setup_path = os.path.join(plugin.rosdir, "snapcraft-setup.sh")
        lines_of_interest = [
            "if [ -f {} ]; then".format(setup_path),
            ". {}".format(setup_path),
            "fi",
        ]
        actual_lines = []

        for line in environment:
            line = line.strip()
            if line in lines_of_interest:
                actual_lines.append(line)

        self.assertThat(
            actual_lines,
            Equals(lines_of_interest),
            "Expected snapcraft-setup.sh to be sourced after checking for its "
            "existence",
        )

    @mock.patch.object(
        catkin.CatkinPlugin, "_source_setup_sh", return_value="test-source-setup.sh"
    )
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="bar")
    def test_run_environment_with_catkin_ros_master_uri(
        self, run_mock, source_setup_sh_mock
    ):

        self.properties.catkin_ros_master_uri = "http://rosmaster:11311"
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        self._verify_run_environment(plugin)

    def _evaluate_environment(self, predefinition=""):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        python_path = os.path.join(
            plugin.installdir, "usr", "lib", "python2.7", "dist-packages"
        )
        os.makedirs(python_path)

        # Save plugin environment off into a file and read the evaluated
        # version back in, thus obtaining the real environment
        with tempfile.NamedTemporaryFile(mode="w+") as f:
            f.write(predefinition)
            f.write("\n".join(["export " + e for e in plugin.env(plugin.installdir)]))
            f.write('python3 -c "import os; print(dict(os.environ))"')
            f.flush()
            return ast.literal_eval(
                subprocess.check_output(["/bin/sh", f.name])
                .decode(sys.getfilesystemencoding())
                .strip()
            )

    def _pythonpath_segments(self, environment):
        # Verify that the environment contains PYTHONPATH, and return its
        # segments as a list.
        self.assertThat(environment, Contains("PYTHONPATH"))
        return environment["PYTHONPATH"].split(":")

    def _list_contains_empty_items(self, item_list):
        empty_items = [i for i in item_list if not i.strip()]
        return len(empty_items) > 0

    def test_pythonpath_if_not_defined(self):
        environment = self._evaluate_environment()
        segments = self._pythonpath_segments(environment)
        self.assertFalse(
            self._list_contains_empty_items(segments),
            "PYTHONPATH unexpectedly contains empty segments: {}".format(
                environment["PYTHONPATH"]
            ),
        )

    def test_pythonpath_if_null(self):
        environment = self._evaluate_environment(
            textwrap.dedent(
                """
            export PYTHONPATH=
        """
            )
        )

        segments = self._pythonpath_segments(environment)
        self.assertFalse(
            self._list_contains_empty_items(segments),
            "PYTHONPATH unexpectedly contains empty segments: {}".format(
                environment["PYTHONPATH"]
            ),
        )

    def test_pythonpath_if_not_empty(self):
        environment = self._evaluate_environment(
            textwrap.dedent(
                """
            export PYTHONPATH=foo
        """
            )
        )

        segments = self._pythonpath_segments(environment)
        self.assertFalse(
            self._list_contains_empty_items(segments),
            "PYTHONPATH unexpectedly contains empty segments: {}".format(
                environment["PYTHONPATH"]
            ),
        )
        self.assertThat(segments, Contains("foo"))

    @mock.patch.object(
        catkin.CatkinPlugin, "_source_setup_sh", return_value="test-source-setup"
    )
    def test_generate_snapcraft_sh(self, source_setup_sh_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        plugin._generate_snapcraft_setup_sh("test-root", None)
        source_setup_sh_mock.assert_called_with("test-root", None)
        with open(os.path.join(plugin.rosdir, "snapcraft-setup.sh"), "r") as f:
            self.assertThat(f.read(), Equals("test-source-setup"))

    def test_source_setup_sh(self):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        # Make sure $@ is zeroed, then setup.sh sourced, then $@ is restored
        rosdir = os.path.join("test-root", "opt", "ros", self.ros_distro)
        setup_path = os.path.join(rosdir, "setup.sh")
        lines_of_interest = [
            "set --",
            "if [ -f {} ]; then".format(setup_path),
            "set -- --local",
            "_CATKIN_SETUP_DIR={} . {}".format(rosdir, setup_path),
            "fi",
            'eval "set -- $BACKUP_ARGS"',
        ]
        actual_lines = []

        for line in plugin._source_setup_sh("test-root", None).split("\n"):
            line = line.strip()
            if line in lines_of_interest:
                actual_lines.append(line)

        self.assertThat(
            actual_lines,
            Equals(lines_of_interest),
            "Expected ROS's setup.sh to be sourced after args were zeroed, "
            "followed by the args being restored.",
        )

    def test_generate_snapcraft_sh_with_underlay(self):
        self.properties.underlay = {
            "build-path": "test-build-underlay",
            "run-path": "test-run-underlay",
        }
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        # Make sure $@ is zeroed, then setup.sh sourced, then $@ is restored
        underlay_setup_path = os.path.join("test-underlay", "setup.sh")
        rosdir = os.path.join("test-root", "opt", "ros", self.ros_distro)
        setup_path = os.path.join(rosdir, "setup.sh")
        lines_of_interest = [
            "set --",
            "if [ -f {} ]; then".format(underlay_setup_path),
            "set -- --local",
            "_CATKIN_SETUP_DIR={} . {}".format("test-underlay", underlay_setup_path),
            "if [ -f {} ]; then".format(setup_path),
            "set -- --local --extend",
            "_CATKIN_SETUP_DIR={} . {}".format(rosdir, setup_path),
            "fi",
            "fi",
            'eval "set -- $BACKUP_ARGS"',
        ]
        actual_lines = []

        plugin._generate_snapcraft_setup_sh("test-root", "test-underlay")
        with open(os.path.join(plugin.rosdir, "snapcraft-setup.sh"), "r") as f:
            for line in f:
                line = line.strip()
                if line in lines_of_interest:
                    actual_lines.append(line)

        self.assertThat(
            actual_lines,
            Equals(lines_of_interest),
            "Expected ROS's setup.sh to be sourced after args were zeroed, "
            "followed by the args being restored.",
        )

    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="bar")
    def test_run_environment_no_python(self, run_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

        python_path = os.path.join(
            plugin.installdir, "usr", "lib", "python2.7", "dist-packages"
        )

        environment = plugin.env(plugin.installdir)

        self.assertFalse(
            "PYTHONPATH={}".format(python_path) in environment, environment
        )

    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_prepare_build(self, use_python_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.rosdir, "test"))

        # Place a few .cmake files with incorrect paths, and some files that
        # shouldn't be changed.
        files = [
            {
                "path": "fooConfig.cmake",
                "contents": '"/usr/lib/foo"',
                "expected": '"{}/usr/lib/foo"'.format(plugin.installdir),
            },
            {
                "path": "anotherConfig.cmake",
                "contents": '"$ENV{SNAPCRAFT_STAGE}/usr/lib/another"',
                "expected": '"{}/usr/lib/another"'.format(plugin.installdir),
            },
            {
                "path": "bar.cmake",
                "contents": '"/usr/lib/bar"',
                "expected": '"/usr/lib/bar"',
            },
            {
                "path": "test/bazConfig.cmake",
                "contents": '"/test/baz;/usr/lib/baz"',
                "expected": '"{0}/test/baz;{0}/usr/lib/baz"'.format(plugin.installdir),
            },
            {"path": "test/quxConfig.cmake", "contents": "qux", "expected": "qux"},
            {
                "path": "test/installedConfig.cmake",
                "contents": '"{}/foo"'.format(plugin.installdir),
                "expected": '"{}/foo"'.format(plugin.installdir),
            },
        ]

        for file_info in files:
            path = os.path.join(plugin.rosdir, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        plugin._prepare_build()

        self.assertTrue(use_python_mock.called)

        for file_info in files:
            path = os.path.join(plugin.rosdir, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))


class PullNoUnderlayTestCase(CatkinPluginBaseTest):

    underlay = None
    expected_underlay_path = None

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_debian_dependencies(self, generate_setup_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo", "bar", "baz"}}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

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

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_local_dependencies(self, generate_setup_mock):
        self.properties.catkin_packages.append("package_2")

        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies (only local)
        self.dependencies_mock.return_value = {}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

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

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_with_roscore(self, generate_setup_mock):
        self.properties.include_roscore = True
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies
        self.dependencies_mock.return_value = {}

        def resolve(package_name):
            if package_name == "ros_core":
                return {"apt": {"ros-core-dependency"}}

        self.rosdep_mock.return_value.resolve_dependency = resolve

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        # Verify that roscore was installed
        self.assertThat(
            self.ubuntu_mock.fetch_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=plugin.stage_packages_path,
                        package_names={"ros-core-dependency"},
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

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_with_rosinstall_files(self, generate_setup_mock):
        self.properties.rosinstall_files = ["rosinstall-file"]
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies
        self.dependencies_mock.return_value = {}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.assert_wstool_setup(
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "wstool"),
        )

        self.wstool_mock.assert_has_calls(
            [
                mock.call().merge(os.path.join(plugin.sourcedir, "rosinstall-file")),
                mock.call().update(),
            ]
        )

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        # Verify that no .deb packages were installed
        self.assertTrue(
            mock.call().unpack(plugin.installdir) not in self.ubuntu_mock.mock_calls
        )

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_pip_dependencies(self, generate_setup_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"pip": {"foo", "bar", "baz"}}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        self.assert_pip_setup(
            "2", plugin.partdir, plugin.installdir, plugin.project.stage_dir
        )

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

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


class PullUnderlayTestCase(CatkinPluginBaseTest):

    underlay = {"build-path": "test-build-path", "run-path": "test-run-path"}
    expected_underlay_path = "test-build-path"

    def setUp(self):
        super().setUp()

        # Make the underlay a valid workspace
        if self.underlay:
            os.makedirs(self.underlay["build-path"])
            open(os.path.join(self.underlay["build-path"], "setup.sh"), "w").close()
            self.properties.underlay = self.underlay

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_debian_dependencies(self, generate_setup_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"apt": {"foo", "bar", "baz"}}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

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

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_local_dependencies(self, generate_setup_mock):
        self.properties.catkin_packages.append("package_2")

        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies (only local)
        self.dependencies_mock.return_value = {}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

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

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_with_roscore(self, generate_setup_mock):
        self.properties.include_roscore = True
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies
        self.dependencies_mock.return_value = {}

        def resolve(package_name):
            if package_name == "ros_core":
                return {"apt": {"ros-core-dependency"}}

        self.rosdep_mock.return_value.resolve_dependency = resolve

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        # Verify that roscore was installed
        self.assertThat(
            self.ubuntu_mock.fetch_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=plugin.stage_packages_path,
                        package_names={"ros-core-dependency"},
                        base=self.project._get_build_base(),
                        target_arch=self.project._get_stage_packages_target_arch(),
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

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_with_rosinstall_files(self, generate_setup_mock):
        self.properties.rosinstall_files = ["rosinstall-file"]
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        # No system dependencies
        self.dependencies_mock.return_value = {}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.assert_wstool_setup(
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "wstool"),
        )

        self.wstool_mock.assert_has_calls(
            [
                mock.call().merge(os.path.join(plugin.sourcedir, "rosinstall-file")),
                mock.call().update(),
            ]
        )

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        # Verify that no .deb packages were installed
        self.assertTrue(
            mock.call().unpack(plugin.installdir) not in self.ubuntu_mock.mock_calls
        )

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    def test_pull_pip_dependencies(self, generate_setup_mock):
        plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        self.dependencies_mock.return_value = {"pip": {"foo", "bar", "baz"}}

        plugin.pull()

        self.assert_rosdep_setup(
            self.ros_distro,
            self.ros_version,
            os.path.join(plugin.sourcedir, "src"),
            os.path.join(plugin.partdir, "rosdep"),
            self.ubuntu_distro,
        )

        self.wstool_mock.assert_not_called()

        self.assert_pip_setup(
            "2", plugin.partdir, plugin.installdir, plugin.project.stage_dir
        )

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

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
        catkin_packages = ["my_package"]
        source_space = "src"
        source_subdir = None
        include_roscore = False
        catkin_cmake_args = []
        underlay = None
        rosinstall_files = None
        recursive_rosinstall = False
        build_attributes = []
        catkin_ros_master_uri = "http://localhost:11311"
        disable_parallel = False

    return Options()


class TestBuildArgs:

    scenarios = [
        (
            "release without catkin-cmake-args",
            {
                "build_attributes": [],
                "catkin_cmake_args": [],
                "disable_parallel": False,
            },
        ),
        (
            "release with catkin-cmake-args",
            {
                "build_attributes": [],
                "catkin_cmake_args": ["-DFOO"],
                "disable_parallel": False,
            },
        ),
        (
            "debug without catkin-cmake-args",
            {
                "build_attributes": ["debug"],
                "catkin_cmake_args": [],
                "disable_parallel": False,
            },
        ),
        (
            "debug with catkin-cmake-args",
            {
                "build_attributes": ["debug"],
                "catkin_cmake_args": ["-DFOO"],
                "disable_parallel": False,
            },
        ),
        (
            "release without catkin-cmake-args single threaded",
            {"build_attributes": [], "catkin_cmake_args": [], "disable_parallel": True},
        ),
    ]

    @mock.patch("snapcraft.plugins.v1.catkin.CatkinPlugin.run", autospec=True)
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_prepare_build")
    @mock.patch.object(catkin.CatkinPlugin, "_finish_build")
    def test_build(
        self,
        finish_build_mock,
        prepare_build_mock,
        run_output_mock,
        run_mock,
        project_core18,
        options,
        build_attributes,
        catkin_cmake_args,
        disable_parallel,
    ):

        options.build_attributes += build_attributes
        options.catkin_cmake_args += catkin_cmake_args
        options.disable_parallel = disable_parallel

        plugin = catkin.CatkinPlugin("test-part", options, project_core18)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        prepare_build_mock.assert_called_once_with()

        if "debug" in build_attributes:
            build_type = "Debug"
        else:
            build_type = "Release"

        expected_command = [
            "catkin_make_isolated",
            "--install",
            "--pkg",
            "my_package",
            "--directory",
            plugin.builddir,
            "--source-space",
            f"{plugin.builddir}/src",
            "--install-space",
            f"{plugin.installdir}/opt/ros/melodic",
            "-j{}".format(1 if disable_parallel else 2),
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={build_type}",
        ]
        if catkin_cmake_args:
            expected_command += catkin_cmake_args
        run_mock.assert_called_once_with(plugin, expected_command)

        finish_build_mock.assert_called_once_with()

    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_prepare_build")
    @mock.patch.object(catkin.CatkinPlugin, "_finish_build")
    def test_build_all_packages(
        self,
        finish_build_mock,
        prepare_build_mock,
        run_output_mock,
        run_mock,
        project_core18,
        options,
        build_attributes,
        catkin_cmake_args,
        disable_parallel,
    ):
        options.catkin_packages = None
        options.build_attributes += build_attributes
        options.catkin_cmake_args += catkin_cmake_args
        options.disable_parallel = disable_parallel

        plugin = catkin.CatkinPlugin("test-part", options, project_core18)
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        prepare_build_mock.assert_called_once_with()

        if "debug" in build_attributes:
            build_type = "Debug"
        else:
            build_type = "Release"

        expected_command = [
            "catkin_make_isolated",
            "--install",
            "--directory",
            plugin.builddir,
            "--source-space",
            f"{plugin.builddir}/src",
            "--install-space",
            f"{plugin.installdir}/opt/ros/melodic",
            "-j{}".format(1 if disable_parallel else 2),
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={build_type}",
        ]
        if catkin_cmake_args:
            expected_command += catkin_cmake_args
        assert run_mock.mock_calls[0][1][0] == expected_command

        finish_build_mock.assert_called_once_with()


class FinishBuildNoUnderlayTestCase(CatkinPluginBaseTest):

    underlay = None
    expected_underlay_path = None

    def setUp(self):
        super().setUp()

        self.properties.underlay = self.underlay
        self.plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_finish_build_cmake_paths(self, use_python_mock, generate_setup_mock):
        os.makedirs(os.path.join(self.plugin.rosdir, "test"))

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
            path = os.path.join(self.plugin.rosdir, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._finish_build()

        self.assertTrue(use_python_mock.called)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                "$SNAP", self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        for file_info in files:
            path = os.path.join(self.plugin.rosdir, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

        # Verify that no sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            Not(FileExists()),
        )

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_finish_build_cmake_prefix_path(
        self, use_python_mock, run_output_mock, run_mock, generate_setup_mock
    ):
        setup_file = os.path.join(self.plugin.rosdir, "_setup_util.py")
        os.makedirs(os.path.dirname(setup_file))

        with open(setup_file, "w") as f:
            f.write(
                "CMAKE_PREFIX_PATH = '{0}/{1};{0}\n".format(
                    self.plugin.rosdir, self.ros_distro
                )
            )

        self.plugin._finish_build()

        self.assertTrue(use_python_mock.called)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                "$SNAP", self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        expected = "CMAKE_PREFIX_PATH = []\n"

        with open(setup_file, "r") as f:
            self.assertThat(
                f.read(),
                Equals(expected),
                "The absolute path to python or the CMAKE_PREFIX_PATH "
                "was not replaced as expected",
            )

        # Verify that no sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            Not(FileExists()),
        )

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_finish_build_python_sitecustomize(
        self, use_python_mock, run_output_mock, run_mock, generate_setup_mock
    ):
        self.pip_mock.return_value.list.return_value = {"test-package"}

        # Create site.py, indicating that python2 was a stage-package
        site_py_path = os.path.join(
            self.plugin.installdir, "usr", "lib", "python2.7", "site.py"
        )
        os.makedirs(os.path.dirname(site_py_path), exist_ok=True)
        open(site_py_path, "w").close()

        # Also create python2 site-packages, indicating that pip packages were
        # installed.
        os.makedirs(
            os.path.join(self.plugin.installdir, "lib", "python2.7", "site-packages"),
            exist_ok=True,
        )

        self.plugin._finish_build()

        # Verify that sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            FileExists(),
        )


class FinishBuildUnderlayTestCase(CatkinPluginBaseTest):

    underlay = {"build-path": "test-build-path", "run-path": "test-run-path"}
    expected_underlay_path = "test-run-path"

    def setUp(self):
        super().setUp()

        self.properties.underlay = self.underlay
        self.plugin = catkin.CatkinPlugin("test-part", self.properties, self.project)

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_finish_build_cmake_paths(self, use_python_mock, generate_setup_mock):
        os.makedirs(os.path.join(self.plugin.rosdir, "test"))

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
            path = os.path.join(self.plugin.rosdir, file_info["path"])
            with open(path, "w") as f:
                f.write(file_info["contents"])

        self.plugin._finish_build()

        self.assertTrue(use_python_mock.called)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                "$SNAP", self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        for file_info in files:
            path = os.path.join(self.plugin.rosdir, file_info["path"])
            with open(path, "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

        # Verify that no sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            Not(FileExists()),
        )

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_finish_build_cmake_prefix_path(
        self, use_python_mock, run_output_mock, run_mock, generate_setup_mock
    ):
        setup_file = os.path.join(self.plugin.rosdir, "_setup_util.py")
        os.makedirs(os.path.dirname(setup_file))

        with open(setup_file, "w") as f:
            f.write(
                "CMAKE_PREFIX_PATH = '{0}/{1};{0}\n".format(
                    self.plugin.rosdir, self.ros_distro
                )
            )

        self.plugin._finish_build()

        self.assertTrue(use_python_mock.called)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                "$SNAP", self.expected_underlay_path
            )
        else:
            generate_setup_mock.assert_not_called()

        expected = "CMAKE_PREFIX_PATH = []\n"

        with open(setup_file, "r") as f:
            self.assertThat(
                f.read(),
                Equals(expected),
                "The absolute path to python or the CMAKE_PREFIX_PATH "
                "was not replaced as expected",
            )

        # Verify that no sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            Not(FileExists()),
        )

    @mock.patch.object(catkin.CatkinPlugin, "_generate_snapcraft_setup_sh")
    @mock.patch.object(catkin.CatkinPlugin, "run")
    @mock.patch.object(catkin.CatkinPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin.CatkinPlugin, "_use_in_snap_python")
    def test_finish_build_python_sitecustomize(
        self, use_python_mock, run_output_mock, run_mock, generate_setup_mock
    ):
        self.pip_mock.return_value.list.return_value = {"test-package"}

        # Create site.py, indicating that python2 was a stage-package
        site_py_path = os.path.join(
            self.plugin.installdir, "usr", "lib", "python2.7", "site.py"
        )
        os.makedirs(os.path.dirname(site_py_path), exist_ok=True)
        open(site_py_path, "w").close()

        # Also create python2 site-packages, indicating that pip packages were
        # installed.
        os.makedirs(
            os.path.join(self.plugin.installdir, "lib", "python2.7", "site-packages"),
            exist_ok=True,
        )

        self.plugin._finish_build()

        # Verify that sitecustomize.py was generated
        self.assertThat(
            os.path.join(
                self.plugin.installdir, "usr", "lib", "python2.7", "sitecustomize.py"
            ),
            FileExists(),
        )


class FindSystemDependenciesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.rosdep_mock = mock.MagicMock()
        self.rosdep_mock.get_dependencies.return_value = {"bar"}

        self.catkin_mock = mock.MagicMock()
        exception = catkin.CatkinPackageNotFoundError("foo")
        self.catkin_mock.find.side_effect = exception

    def test_find_system_dependencies_system_only(self):
        self.rosdep_mock.resolve_dependency.return_value = {"apt": {"baz"}}

        self.assertThat(
            catkin._find_system_dependencies(
                {"foo"}, self.rosdep_mock, self.catkin_mock
            ),
            Equals({"apt": {"baz"}}),
        )

        self.rosdep_mock.get_dependencies.assert_called_once_with("foo")
        self.rosdep_mock.resolve_dependency.assert_called_once_with("bar")
        self.catkin_mock.find.assert_called_once_with("bar")

    def test_find_system_dependencies_system_only_no_packages(self):
        self.rosdep_mock.resolve_dependency.return_value = {"apt": {"baz"}}

        self.assertThat(
            catkin._find_system_dependencies(None, self.rosdep_mock, self.catkin_mock),
            Equals({"apt": {"baz"}}),
        )

        self.rosdep_mock.get_dependencies.assert_called_once_with()
        self.rosdep_mock.resolve_dependency.assert_called_once_with("bar")
        self.catkin_mock.find.assert_called_once_with("bar")

    def test_find_system_dependencies_local_only(self):
        self.assertThat(
            catkin._find_system_dependencies(
                {"foo", "bar"}, self.rosdep_mock, self.catkin_mock
            ),
            HasLength(0),
        )

        self.rosdep_mock.get_dependencies.assert_has_calls(
            [mock.call("foo"), mock.call("bar")], any_order=True
        )
        self.rosdep_mock.resolve_dependency.assert_not_called()
        self.catkin_mock.find.assert_not_called()

    def test_find_system_dependencies_satisfied_in_stage(self):
        self.catkin_mock.find.side_effect = None
        self.catkin_mock.find.return_value = "baz"

        self.assertThat(
            catkin._find_system_dependencies(
                {"foo"}, self.rosdep_mock, self.catkin_mock
            ),
            HasLength(0),
        )

        self.rosdep_mock.get_dependencies.assert_called_once_with("foo")
        self.catkin_mock.find.assert_called_once_with("bar")
        self.rosdep_mock.resolve_dependency.assert_not_called()

    def test_find_system_dependencies_mixed(self):
        self.rosdep_mock.get_dependencies.return_value = {"bar", "baz", "qux"}
        self.rosdep_mock.resolve_dependency.return_value = {"apt": {"quux"}}

        def _fake_find(package_name):
            if package_name == "qux":
                return package_name
            raise catkin.CatkinPackageNotFoundError(package_name)

        self.catkin_mock.find.side_effect = _fake_find
        self.assertThat(
            catkin._find_system_dependencies(
                {"foo", "bar"}, self.rosdep_mock, self.catkin_mock
            ),
            Equals({"apt": {"quux"}}),
        )

        self.rosdep_mock.get_dependencies.assert_has_calls(
            [mock.call("foo"), mock.call("bar")], any_order=True
        )
        self.rosdep_mock.resolve_dependency.assert_called_once_with("baz")
        self.catkin_mock.find.assert_has_calls(
            [mock.call("baz"), mock.call("qux")], any_order=True
        )

    def test_find_system_dependencies_missing_local_dependency(self):
        # Setup a dependency on a non-existing package, and it doesn't resolve
        # to a system dependency.'
        exception = _ros.rosdep.RosdepDependencyNotResolvedError("foo")
        self.rosdep_mock.resolve_dependency.side_effect = exception

        raised = self.assertRaises(
            catkin.CatkinInvalidSystemDependencyError,
            catkin._find_system_dependencies,
            {"foo"},
            self.rosdep_mock,
            self.catkin_mock,
        )

        self.assertThat(
            str(raised),
            Equals(
                "Package 'bar' isn't a valid system dependency. Did "
                "you forget to add it to catkin-packages? If not, "
                "add the Ubuntu package containing it to "
                "stage-packages until you can get it into the rosdep "
                "database."
            ),
        )

    def test_find_system_dependencies_raises_if_unsupported_type(self):
        self.rosdep_mock.resolve_dependency.return_value = {"unsupported-type": {"baz"}}

        raised = self.assertRaises(
            catkin.CatkinUnsupportedDependencyTypeError,
            catkin._find_system_dependencies,
            {"foo"},
            self.rosdep_mock,
            self.catkin_mock,
        )

        self.assertThat(
            str(raised),
            Equals(
                "Package 'bar' resolved to an unsupported type of dependency: "
                "'unsupported-type'."
            ),
        )


class HandleRosinstallFilesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.wstool_mock = mock.MagicMock()

    def test_single_rosinstall_file(self):
        rosinstall_file = os.path.join("source_path", "rosinstall_file")
        catkin._handle_rosinstall_files(self.wstool_mock, [rosinstall_file])
        self.wstool_mock.merge.assert_called_once_with(
            os.path.join("source_path", "rosinstall_file")
        )

    def test_multiple_rosinstall_files(self):
        rosinstall_files = [
            os.path.join("source_path", "file1"),
            os.path.join("source_path", "file2"),
        ]

        catkin._handle_rosinstall_files(self.wstool_mock, rosinstall_files)

        # The order matters here. It should be the same as how they were passed
        self.wstool_mock.merge.assert_has_calls(
            [
                mock.call(os.path.join("source_path", "file1")),
                mock.call(os.path.join("source_path", "file2")),
            ]
        )


class RecursivelyHandleRosinstallFilesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.wstool_mock = mock.MagicMock()

    def test_recursive_rosinstall(self):
        counter = 0

        # A fake update that plops a new rosinstall file down every time
        # it's called (up to two times).
        def _fake_wstool_update():
            nonlocal counter
            if counter < 2:
                counter += 1
                open(
                    os.path.join("source_path", "{}.rosinstall".format(counter)), "w"
                ).close()

        self.wstool_mock.update.side_effect = _fake_wstool_update

        os.mkdir("source_path")
        open(os.path.join("source_path", "0.rosinstall"), "w").close()

        catkin._recursively_handle_rosinstall_files(self.wstool_mock, "source_path")

        self.wstool_mock.merge.assert_has_calls(
            [
                mock.call(os.path.join("source_path", "0.rosinstall")),
                mock.call(os.path.join("source_path", "1.rosinstall")),
                mock.call(os.path.join("source_path", "2.rosinstall")),
            ]
        )


class CatkinFindTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.project = snapcraft.project.Project()
        self.project._snap_meta.build_base = "core"
        self.catkin = catkin._Catkin(
            "kinetic", "workspace_path", "catkin_path", self.project
        )

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_setup(self):
        # Return something other than a Mock to ease later assertions
        self.check_output_mock.return_value = b""

        self.catkin.setup()

        # Verify that only rospack was installed (no other .debs)
        self.assertThat(
            self.ubuntu_mock.fetch_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=self.catkin._catkin_stage_packages_path,
                        package_names=["ros-kinetic-catkin"],
                        base=self.project._get_build_base(),
                        target_arch=self.project._get_stage_packages_target_arch(),
                    )
                ]
            ),
        )
        self.assertThat(
            self.ubuntu_mock.unpack_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=self.catkin._catkin_stage_packages_path,
                        install_path=pathlib.Path("catkin_path/install"),
                    )
                ]
            ),
        )

    def test_setup_can_run_multiple_times(self):
        self.catkin.setup()

        # Make sure running setup() again doesn't have problems with the old
        # environment. An exception will be raised if setup() can't be called
        # twice.
        self.catkin.setup()

    def test_find(self):
        self.check_output_mock.return_value = b"bar"

        self.assertThat(self.catkin.find("foo"), Equals("bar"))

        self.assertTrue(self.check_output_mock.called)
        positional_args = self.check_output_mock.call_args[0][0]
        self.assertThat(
            " ".join(positional_args), Contains("catkin_find --first-only foo")
        )

    def test_find_only_in_catkin_workspace(self):
        self.check_output_mock.return_value = os.path.join(
            self.catkin._catkin_install_path, "bar"
        ).encode(sys.getfilesystemencoding())

        with testtools.ExpectedException(
            catkin.CatkinPackageNotFoundError, "Unable to find Catkin package 'foo'"
        ):
            self.catkin.find("foo")

    def test_find_non_existing_package(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(1, "foo")

        with testtools.ExpectedException(
            catkin.CatkinPackageNotFoundError, "Unable to find Catkin package 'foo'"
        ):
            self.catkin.find("foo")

        self.assertTrue(self.check_output_mock.called)
        positional_args = self.check_output_mock.call_args[0][0]
        self.assertThat(
            " ".join(positional_args), Contains("catkin_find --first-only foo")
        )


class TestCatkinCmakeArg:
    scenarios = [
        (
            "no quote",
            dict(input_string="-DCMAKE_FOO=BAR", expected_string="-DCMAKE_FOO=BAR"),
        ),
        (
            "no quote multiple",
            dict(
                input_string="-DCMAKE_C_FLAGS=-Wall -Werror",
                expected_string="-DCMAKE_C_FLAGS=-Wall -Werror",
            ),
        ),
        (
            "single quote",
            dict(input_string="-DCMAKE_FOO='BAR'", expected_string="-DCMAKE_FOO=BAR"),
        ),
        (
            "single quote multiple",
            dict(
                input_string="-DCMAKE_C_FLAGS='-Wall -Werror'",
                expected_string="-DCMAKE_C_FLAGS=-Wall -Werror",
            ),
        ),
        (
            "double quote",
            dict(input_string='-DCMAKE_FOO="BAR"', expected_string="-DCMAKE_FOO=BAR"),
        ),
        (
            "double quote multiple",
            dict(
                input_string='-DCMAKE_C_FLAGS="-Wall -Werror"',
                expected_string="-DCMAKE_C_FLAGS=-Wall -Werror",
            ),
        ),
        (
            "nested double quote",
            dict(
                input_string='-DCMAKE_C_FLAGS="-Wall -Werror -DTEXT=\\"Hello Friend\\""',
                expected_string='-DCMAKE_C_FLAGS=-Wall -Werror -DTEXT="Hello Friend"',
            ),
        ),
        (
            "nested single quote",
            dict(
                input_string="-DCMAKE_C_FLAGS=\"-Wall -Werror -DTEXT='Hello Friend'\"",
                expected_string="-DCMAKE_C_FLAGS=-Wall -Werror -DTEXT='Hello Friend'",
            ),
        ),
    ]

    def test_arg(self, input_string, expected_string):
        assert catkin._parse_cmake_arg(input_string) == expected_string
