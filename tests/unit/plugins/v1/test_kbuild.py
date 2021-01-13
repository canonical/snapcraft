# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018, 2020 Canonical Ltd
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
import pytest
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.internal import errors, meta
from snapcraft.plugins.v1 import kbuild

from . import PluginsV1BaseTestCase


class KBuildPluginTestCase(PluginsV1BaseTestCase):
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

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project)

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

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project)

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

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project)

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

        plugin = kbuild.KBuildPlugin("test-part", self.options, self.project)

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

    def test_unsupported_base(self):
        self.project._snap_meta.base = "unsupported-base"

        raised = self.assertRaises(
            errors.PluginBaseError,
            kbuild.KBuildPlugin,
            "test-part",
            self.options,
            self.project,
        )

        self.assertThat(raised.part_name, Equals("test-part"))
        self.assertThat(raised.base, Equals("unsupported-base"))


@pytest.mark.parametrize("deb_arch", ["armhf", "arm64", "i386", "ppc64el"])
@mock.patch("subprocess.check_call")
def test_cross_compile(mock_check_call, monkeypatch, mock_run, deb_arch):
    monkeypatch.setattr(snapcraft.project.Project, "is_cross_compiling", True)

    class Options:
        build_parameters = []
        kconfigfile = None
        kconfigflavour = None
        kdefconfig = []
        kconfigs = []
        build_attributes = []

    project = snapcraft.project.Project(target_deb_arch=deb_arch)
    project._snap_meta = meta.snap.Snap(name="test-snap", base="core18")

    plugin = kbuild.KBuildPlugin("test-part", Options(), project)
    plugin.enable_cross_compilation()

    plugin.build()
    mock_run.assert_has_calls(
        [
            mock.call(
                [
                    "make",
                    "-j1",
                    f"ARCH={project.kernel_arch}",
                    f"CROSS_COMPILE={project.cross_compiler_prefix}",
                    mock.ANY,
                ]
            )
        ]
    )
