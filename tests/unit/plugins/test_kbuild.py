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

import logging
import os
from unittest import mock

import fixtures
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import kbuild
from tests import unit


class KBuildPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            build_parameters = []
            kconfigfile = None
            kconfigflavour = None
            kdefconfig = []
            kconfigs = []
            build_attributes = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        schema = kbuild.KBuildPlugin.schema()

        properties = schema["properties"]
        self.assertThat(properties["kdefconfig"]["type"], Equals("array"))
        self.assertThat(properties["kdefconfig"]["default"], Equals(["defconfig"]))

        self.assertThat(properties["kconfigfile"]["type"], Equals("string"))
        self.assertThat(properties["kconfigfile"]["default"], Equals(None))

        self.assertThat(properties["kconfigflavour"]["type"], Equals("string"))
        self.assertThat(properties["kconfigflavour"]["default"], Equals(None))

        self.assertThat(properties["kconfigs"]["type"], Equals("array"))
        self.assertThat(properties["kconfigs"]["default"], Equals([]))
        self.assertThat(properties["kconfigs"]["minitems"], Equals(1))
        self.assertThat(properties["kconfigs"]["items"]["type"], Equals("string"))
        self.assertTrue(properties["kconfigs"]["uniqueItems"])

    def test_get_build_properties(self):
        expected_build_properties = [
            "kdefconfig",
            "kconfigfile",
            "kconfigflavour",
            "kconfigs",
        ]
        resulting_build_properties = kbuild.KBuildPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    @mock.patch("subprocess.check_call")
    @mock.patch.object(kbuild.KBuildPlugin, "run")
    def test_build_with_kconfigfile(self, run_mock, check_call_mock):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project_options)

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertThat(check_call_mock.call_count, Equals(1))
        check_call_mock.assert_has_calls(
            [mock.call('yes "" | make -j2 oldconfig', shell=True, cwd=plugin.builddir)]
        )

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "install",
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))

    @mock.patch("subprocess.check_call")
    @mock.patch.object(kbuild.KBuildPlugin, "run")
    def test_build_verbose_with_kconfigfile(self, run_mock, check_call_mock):
        fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(fake_logger)

        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project_options)

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertThat(check_call_mock.call_count, Equals(1))
        check_call_mock.assert_has_calls(
            [
                mock.call(
                    'yes "" | make -j2 V=1 oldconfig', shell=True, cwd=plugin.builddir
                )
            ]
        )

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "V=1"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "V=1",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "install",
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))

    @mock.patch("subprocess.check_call")
    @mock.patch.object(kbuild.KBuildPlugin, "run")
    def test_build_with_kconfigfile_and_kconfigs(self, run_mock, check_call_mock):
        self.options.kconfigfile = "config"
        self.options.kconfigs = ["SOMETHING=y", "ACCEPT=n"]

        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project_options)

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertThat(check_call_mock.call_count, Equals(1))
        check_call_mock.assert_has_calls(
            [mock.call('yes "" | make -j2 oldconfig', shell=True, cwd=plugin.builddir)]
        )

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "install",
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        expected_config = """SOMETHING=y
ACCEPT=n

ACCEPT=y

SOMETHING=y
ACCEPT=n
"""
        self.assertThat(config_contents, Equals(expected_config))

        # Finally, ensure that the original kconfigfile was not modified (it's back in
        # the source tree)
        with open(self.options.kconfigfile, "r") as f:
            self.assertThat(f.read(), Equals("ACCEPT=y\n"))

    @mock.patch("subprocess.check_call")
    @mock.patch.object(kbuild.KBuildPlugin, "run")
    def test_build_with_defconfig_and_kconfigs(self, run_mock, check_call_mock):
        self.options.kdefconfig = ["defconfig"]
        self.options.kconfigs = ["SOMETHING=y", "ACCEPT=n"]

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project_options)

        config_file = os.path.join(plugin.builddir, ".config")

        def fake_defconfig(*args, **kwargs):
            if os.path.exists(config_file):
                return
            with open(config_file, "w") as f:
                f.write("ACCEPT=y\n")

        run_mock.side_effect = fake_defconfig

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertThat(check_call_mock.call_count, Equals(1))
        check_call_mock.assert_has_calls(
            [mock.call('yes "" | make -j2 oldconfig', shell=True, cwd=plugin.builddir)]
        )

        self.assertThat(run_mock.call_count, Equals(3))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "install",
                    ]
                ),
            ]
        )

        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        expected_config = """SOMETHING=y
ACCEPT=n

ACCEPT=y

SOMETHING=y
ACCEPT=n
"""
        self.assertThat(config_contents, Equals(expected_config))


class KBuildCrossCompilePluginTestCase(unit.TestCase):

    scenarios = [
        ("armv7l", dict(deb_arch="armhf")),
        ("aarch64", dict(deb_arch="arm64")),
        ("ppc64le", dict(deb_arch="ppc64el")),
    ]

    def setUp(self):
        super().setUp()

        class Options:
            build_parameters = []
            kconfigfile = None
            kconfigflavour = None
            kdefconfig = []
            kconfigs = []
            build_attributes = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions(target_deb_arch=self.deb_arch)

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.ProjectOptions.is_cross_compiling")
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.dict(os.environ, {})
        self.env_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch("subprocess.check_call")
    @mock.patch.object(kbuild.KBuildPlugin, "run")
    def test_cross_compile(self, run_mock, check_call_mock):
        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project_options)
        plugin.enable_cross_compilation()

        plugin.build()
        run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "make",
                        "-j1",
                        "ARCH={}".format(self.project_options.kernel_arch),
                        "CROSS_COMPILE={}".format(
                            self.project_options.cross_compiler_prefix
                        ),
                        "PATH={}:/usr/{}/bin".format(
                            os.environ.copy().get("PATH", ""),
                            self.project_options.arch_triplet,
                        ),
                    ]
                )
            ]
        )
