# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import os.path
import re

from unittest import mock
from testtools.matchers import Contains, Equals

import snapcraft
from snapcraft.plugins import catkin_tools
from tests import unit


class CatkinToolsPluginBaseTestCase(unit.TestCase):
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
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch("snapcraft.plugins._python.Pip")
        self.pip_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.pip_mock.return_value.list.return_value = {}


class CatkinToolsPluginTestCase(CatkinToolsPluginBaseTestCase):
    def setUp(self):
        super().setUp()

        self.project = snapcraft.ProjectOptions()
        self.compilers = catkin_tools.Compilers(
            "compilers_path", "sources", self.project
        )

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch("snapcraft.plugins.catkin_tools.Compilers")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_run_in_bash")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_prepare_build")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_finish_build")
    def test_build_multiple(
        self,
        finish_build_mock,
        prepare_build_mock,
        run_output_mock,
        bashrun_mock,
        run_mock,
        compilers_mock,
    ):
        self.properties.catkin_packages.append("package_2")

        plugin = catkin_tools.CatkinToolsPlugin(
            "test-part", self.properties, self.project_options
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

        bashrun_mock.assert_called_with(check_pkg_arguments(self), env=mock.ANY)

        finish_build_mock.assert_called_once_with()

    @mock.patch("snapcraft.plugins.catkin_tools.Compilers")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run_output", return_value="foo")
    def test_build_runs_in_bash(self, run_output_mock, run_mock, compilers_mock):
        plugin = catkin_tools.CatkinToolsPlugin(
            "test-part", self.properties, self.project_options
        )
        os.makedirs(os.path.join(plugin.sourcedir, "src"))

        plugin.build()

        run_mock.assert_has_calls(
            [mock.call(["/bin/bash", mock.ANY], cwd=mock.ANY, env=mock.ANY)]
        )

    @mock.patch("snapcraft.plugins.catkin.Compilers")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_run_in_bash")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "run_output", return_value="foo")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_prepare_build")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_finish_build")
    def test_build(
        self,
        finish_build_mock,
        prepare_build_mock,
        run_output_mock,
        bashrun_mock,
        run_mock,
        compilers_mock,
    ):
        plugin = catkin_tools.CatkinToolsPlugin(
            "test-part", self.properties, self.project_options
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

        bashrun_mock.assert_called_with(check_build_command(), env=mock.ANY)

        finish_build_mock.assert_called_once_with()


class PrepareBuildTestCase(CatkinToolsPluginBaseTestCase):

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

    def setUp(self):
        super().setUp()

        self.properties.build_attributes.extend(self.build_attributes)
        self.properties.catkin_cmake_args = self.catkin_cmake_args

    @mock.patch("snapcraft.plugins.catkin_tools.Compilers")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_run_in_bash")
    @mock.patch.object(catkin_tools.CatkinToolsPlugin, "_use_in_snap_python")
    def test_prepare_build(self, use_python_mock, bashrun_mock, compilers_mock):
        plugin = catkin_tools.CatkinToolsPlugin(
            "test-part", self.properties, self.project_options
        )
        os.makedirs(os.path.join(plugin.rosdir, "test"))

        plugin._prepare_build()

        build_attributes = self.build_attributes
        catkin_cmake_args = self.catkin_cmake_args

        self.assertTrue(use_python_mock.called)

        confArgs = bashrun_mock.mock_calls[0][1][0]
        command = " ".join(confArgs)
        self.assertThat(command, Contains("catkin init"))

        confArgs = bashrun_mock.mock_calls[1][1][0]
        command = " ".join(confArgs)
        self.assertThat(command, Contains("catkin clean -y"))

        confArgs = bashrun_mock.mock_calls[2][1][0]
        command = " ".join(confArgs)
        self.assertThat(command, Contains("catkin profile add -f default"))

        confArgs = bashrun_mock.mock_calls[3][1][0]
        self.assertThat(confArgs[0], Equals("catkin"))
        self.assertThat(confArgs[1], Equals("config"))

        command = " ".join(confArgs)
        self.assertThat(command, Contains("--profile default"))
        self.assertThat(command, Contains("--install"))
        self.assertThat(command, Contains("--build-space {}".format(plugin.builddir)))
        self.assertThat(
            command,
            Contains(
                "--source-space {}".format(
                    os.path.join(plugin.builddir, plugin.options.source_space)
                )
            ),
        )
        self.assertThat(command, Contains("--install-space {}".format(plugin.rosdir)))

        expected_args = " ".join(self.catkin_cmake_args)

        if catkin_cmake_args:
            self.assertRegexpMatches(
                command, ".*--cmake-args.*{}".format(re.escape(expected_args))
            )

        if "debug" in build_attributes:
            self.assertRegexpMatches(
                command, ".*--cmake-args.*-DCMAKE_BUILD_TYPE=Debug"
            )
        else:
            self.assertRegexpMatches(
                command, ".*--cmake-args.*-DCMAKE_BUILD_TYPE=Release"
            )
