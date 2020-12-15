# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017-2018, 2020 Canonical Ltd
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

from testtools.matchers import Equals, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import cmake
from tests import fixture_setup, unit

from . import PluginsV1BaseTestCase


class CMakeBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class Options:
            configflags = []
            source_subdir = None

            make_parameters = []
            disable_parallel = False
            build_snaps = []

        self.options = Options()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.useFixture(fixture_setup.CleanEnvironment())


class CMakeTest(CMakeBaseTest):
    def test_get_build_properties(self):
        expected_build_properties = ["configflags"]
        resulting_build_properties = cmake.CMakePlugin.get_build_properties()
        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_build_referencing_sourcedir_if_no_subdir(self):
        plugin = cmake.CMakePlugin("test-part", self.options, self.project)
        os.makedirs(plugin.builddir)
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["cmake", plugin.sourcedir, "-DCMAKE_INSTALL_PREFIX="],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
                mock.call(
                    ["cmake", "--build", ".", "--", "-j2"],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
                mock.call(
                    ["cmake", "--build", ".", "--target", "install"],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_referencing_sourcedir_with_subdir(self):
        self.options.source_subdir = "subdir"

        plugin = cmake.CMakePlugin("test-part", self.options, self.project)
        os.makedirs(plugin.builddir)
        plugin.build()

        sourcedir = os.path.join(plugin.sourcedir, plugin.options.source_subdir)
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["cmake", sourcedir, "-DCMAKE_INSTALL_PREFIX="],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
                mock.call(
                    ["cmake", "--build", ".", "--", "-j2"],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
                mock.call(
                    ["cmake", "--build", ".", "--target", "install"],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_disable_parallel(self):
        self.options.disable_parallel = True

        plugin = cmake.CMakePlugin("test-part", self.options, self.project)
        os.makedirs(plugin.builddir)
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["cmake", "--build", ".", "--", "-j1"],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                )
            ]
        )

    def test_build_environment(self):
        plugin = cmake.CMakePlugin("test-part", self.options, self.project)
        os.makedirs(plugin.builddir)
        plugin.build()

        expected = {}

        expected["DESTDIR"] = plugin.installdir
        expected["CMAKE_PREFIX_PATH"] = "$CMAKE_PREFIX_PATH:{}".format(self.stage_dir)
        expected["CMAKE_INCLUDE_PATH"] = "$CMAKE_INCLUDE_PATH:" + ":".join(
            ["{0}/include", "{0}/usr/include", "{0}/include/{1}", "{0}/usr/include/{1}"]
        ).format(self.stage_dir, plugin.project.arch_triplet)
        expected["CMAKE_LIBRARY_PATH"] = "$CMAKE_LIBRARY_PATH:" + ":".join(
            ["{0}/lib", "{0}/usr/lib", "{0}/lib/{1}", "{0}/usr/lib/{1}"]
        ).format(self.stage_dir, plugin.project.arch_triplet)

        self.assertThat(self.run_mock.call_count, Equals(3))
        for call_args in self.run_mock.call_args_list:
            environment = call_args[1]["env"]
            for variable, value in expected.items():
                self.assertTrue(
                    variable in environment,
                    'Expected variable "{}" to be in environment'.format(variable),
                )

                self.assertThat(
                    environment[variable],
                    Equals(value),
                    "Expected ${}={}, but it was {}".format(
                        variable, value, environment[variable]
                    ),
                )

    def test_unsupported_base(self):
        self.project._snap_meta.base = "unsupported-base"

        raised = self.assertRaises(
            errors.PluginBaseError,
            cmake.CMakePlugin,
            "test-part",
            self.options,
            self.project,
        )

        self.assertThat(raised.part_name, Equals("test-part"))
        self.assertThat(raised.base, Equals("unsupported-base"))


class TestSnapCMakeBuild:

    scenarios = [
        ("no snaps", dict(build_snaps=[], expected_config_flags=[])),
        (
            "one build snap",
            dict(
                snaps=["kde-plasma-sdk"],
                expected_configflags=[
                    "-DCMAKE_FIND_ROOT_PATH=/snap/kde-plasma-sdk/current"
                ],
            ),
        ),
        (
            "one build snap, preexisting root find path",
            dict(
                snaps=["kde-plasma-sdk"],
                expected_configflags=[
                    "-DCMAKE_FIND_ROOT_PATH=/snap/kde-plasma-sdk/current"
                ],
            ),
            (
                "one build snap with channel",
                dict(
                    snaps=["kde-plasma-sdk/latest/edge"],
                    expected_configflags=[
                        "-DCMAKE_FIND_ROOT_PATH=/snap/kde-plasma-sdk/current"
                    ],
                ),
            ),
        ),
        (
            "two build snap with channel",
            dict(
                snaps=["gnome-sdk", "kde-plasma-sdk/latest/edge"],
                expected_configflags=[
                    "-DCMAKE_FIND_ROOT_PATH=/snap/gnome-sdk/current;/snap/kde-plasma-sdk/current"
                ],
            ),
        ),
    ]

    def test_build(self, project, mock_run, snaps, expected_configflags):
        class Options:
            configflags = []
            source_subdir = None

            make_parameters = []
            disable_parallel = False
            build_snaps = snaps

        plugin = cmake.CMakePlugin("test-part", Options(), project)
        os.makedirs(plugin.builddir)
        plugin.build()

        mock_run.assert_has_calls(
            [
                mock.call(
                    ["cmake", plugin.sourcedir, "-DCMAKE_INSTALL_PREFIX="]
                    + expected_configflags,
                    env=mock.ANY,
                ),
                mock.call(["cmake", "--build", ".", "--", "-j2"], env=mock.ANY),
                mock.call(
                    ["cmake", "--build", ".", "--target", "install"], env=mock.ANY
                ),
            ]
        )


class FlagsTest(unit.TestCase):
    def test_simple_flag(self):
        flag = cmake._Flag("-DVERBOSE")

        self.assertThat(flag.name, Equals("-DVERBOSE"))
        self.assertThat(flag.value, Equals(None))
        self.assertThat(str(flag), Equals("-DVERBOSE"))

    def test_flag(self):
        flag = cmake._Flag("-DCMAKE_PREFIX_PATH=foo")

        self.assertThat(flag.name, Equals("-DCMAKE_PREFIX_PATH"))
        self.assertThat(flag.value, Equals("foo"))
        self.assertThat(str(flag), Equals("-DCMAKE_PREFIX_PATH=foo"))

    def test_flag_value_change(self):
        flag = cmake._Flag("-DCMAKE_PREFIX_PATH=foo")
        flag.value = "bar"

        self.assertThat(flag.name, Equals("-DCMAKE_PREFIX_PATH"))
        self.assertThat(flag.value, Equals("bar"))
        self.assertThat(str(flag), Equals("-DCMAKE_PREFIX_PATH=bar"))
