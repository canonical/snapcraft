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

import os
from unittest import mock

import pytest

from snapcraft.plugins.v1 import catkin_tools

from . import PluginsV1BaseTestCase


class CatkinToolsPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class props:
            rosdistro = "indigo"
            ubuntu_distro = "trusty"
            catkin_packages = ["my_package"]
            source_space = "src"
            source_subdir = None
            include_roscore = False
            catkin_cmake_args = []
            underlay = None
            rosinstall_files = None
            build_attributes = []

        self.properties = props()

        patcher = mock.patch("snapcraft.plugins.v1._python.Pip")
        self.pip_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.pip_mock.return_value.list.return_value = {}


class CatkinToolsPluginTestCase(CatkinToolsPluginBaseTest):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_prepare_build")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_finish_build")
    def test_build_multiple(
        self, finish_build_mock, prepare_build_mock, run_output_mock, run_mock
    ):
        self.properties.catkin_packages.append("package_2")

        plugin = catkin_tools.CatkinToolsPlugin(
            "test-part", self.properties, self.project
        )
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        class check_pkg_arguments:
            def __init__(self, test):
                self.test = test

            def __eq__(self, args):
                index = args.index("build")
                packages = args[index + 1 :]
                self.test.assertIn("my_package", packages)
                self.test.assertIn("package_2", packages)
                return True

        run_mock.assert_called_with(check_pkg_arguments(self))

        finish_build_mock.assert_called_once_with()

    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_prepare_build")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_finish_build")
    def test_build(
        self, finish_build_mock, prepare_build_mock, run_output_mock, run_mock
    ):
        plugin = catkin_tools.CatkinToolsPlugin(
            "test-part", self.properties, self.project
        )
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        prepare_build_mock.assert_called_once_with()

        class check_build_command:
            def __eq__(self, args):
                command = " ".join(args)
                return (
                    args[0] == "catkin"
                    and "build" in command
                    and "my_package" in command
                )

        run_mock.assert_called_with(check_build_command())

        finish_build_mock.assert_called_once_with()


@pytest.fixture
def options():
    class Options:
        rosdistro = "indigo"
        ubuntu_distro = "trusty"
        catkin_packages = ["my_package"]
        source_space = "src"
        source_subdir = None
        include_roscore = False
        catkin_cmake_args = []
        underlay = None
        rosinstall_files = None
        build_attributes = []

    return Options()


class TestPrepareBuild:

    scenarios = [
        (
            "release without catkin-cmake-args",
            {"build_attributes": [], "catkin_cmake_args": []},
        ),
        (
            "release with catkin-cmake-args",
            {"build_attributes": [], "catkin_cmake_args": ["-DFOO"]},
        ),
        (
            "debug without catkin-cmake-args",
            {"build_attributes": ["debug"], "catkin_cmake_args": []},
        ),
        (
            "debug with catkin-cmake-args",
            {"build_attributes": ["debug"], "catkin_cmake_args": ["-DFOO"]},
        ),
    ]

    def test_prepare_build(
        self,
        monkeypatch,
        mock_run,
        project,
        options,
        build_attributes,
        catkin_cmake_args,
    ):
        monkeypatch.setattr(
            catkin_tools.CatkinToolsPlugin, "_use_in_snap_python", lambda x: True
        )

        options.build_attributes.extend(build_attributes)
        options.catkin_cmake_args = catkin_cmake_args

        plugin = catkin_tools.CatkinToolsPlugin("test-part", options, project)
        os.makedirs(os.path.join(plugin.rosdir, "test"))

        plugin._prepare_build()

        if "debug" in options.build_attributes:
            expected_cmake_args = ["-DCMAKE_BUILD_TYPE=Debug"] + catkin_cmake_args
        else:
            expected_cmake_args = ["-DCMAKE_BUILD_TYPE=Release"] + catkin_cmake_args

        mock_run.assert_has_calls(
            [
                mock.call(["catkin", "init"]),
                mock.call(["catkin", "clean", "-y"]),
                mock.call(["catkin", "profile", "add", "-f", "default"]),
                mock.call(
                    [
                        "catkin",
                        "config",
                        "--profile",
                        "default",
                        "--build-space",
                        plugin.builddir,
                        "--source-space",
                        os.path.join(plugin.builddir, plugin.options.source_space),
                        "--install",
                        "--install-space",
                        plugin.rosdir,
                        "--cmake-args",
                    ]
                    + expected_cmake_args
                ),
            ]
        )
