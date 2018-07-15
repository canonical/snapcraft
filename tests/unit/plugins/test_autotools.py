# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
# Copyright (C) 2016 Harald Sitter <sitter@kde.org>
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
import stat

from unittest import mock
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import autotools
from snapcraft.plugins import make
from tests import unit


class AutotoolsPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            configflags = []
            install_via = "destdir"
            disable_parallel = False
            makefile = None
            make_parameters = []
            make_install_var = "DESTDIR"
            artifacts = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        schema = autotools.AutotoolsPlugin.schema()

        # Verify the presence of all properties
        properties = schema["properties"]
        self.assertTrue(
            "configflags" in properties,
            'Expected "configflags" to be included in properties',
        )
        self.assertTrue(
            "install-via" in properties,
            'Expected "install-via" to be included in properties',
        )

        # Check configflags property
        configflags = properties["configflags"]
        for item in ["type", "minitems", "items", "default"]:
            self.assertTrue(
                item in configflags,
                'Expected "{}" to be included in "configflags"'.format(item),
            )

        configflags_type = configflags["type"]
        self.assertThat(
            configflags_type,
            Equals("array"),
            'Expected "configflags" "type" to be "array", but it '
            'was "{}"'.format(configflags_type),
        )

        configflags_minitems = configflags["minitems"]
        self.assertThat(
            configflags_minitems,
            Equals(1),
            'Expected "configflags" "minitems" to be 1, but '
            "it was {}".format(configflags_minitems),
        )

        configflags_default = configflags["default"]
        self.assertThat(
            configflags_default,
            Equals([]),
            'Expected "configflags" "default" to be [], but '
            "it was {}".format(configflags_default),
        )

        configflags_items = configflags["items"]
        self.assertTrue(
            "type" in configflags_items,
            'Expected "type" to be included in "configflags" ' '"items"',
        )

        configflags_items_type = configflags_items["type"]
        self.assertThat(
            configflags_items_type,
            Equals("string"),
            'Expected "configflags" "items" "type" to be '
            '"string", but it was "{}"'.format(configflags_items_type),
        )

        # Check install-via property
        installvia = properties["install-via"]
        self.assertTrue(
            "enum" in installvia, 'Expected "enum" to be included in "install-via"'
        )
        self.assertTrue(
            "default" in installvia,
            'Expected "default" to be included in "install-via"',
        )

        installvia_enum = installvia["enum"]
        # Using sets for order independence in the comparison
        self.assertThat(set(installvia_enum), Equals(set(["destdir", "prefix"])))

        installvia_default = installvia["default"]
        self.assertThat(
            installvia_default,
            Equals("destdir"),
            'Expected "install-via" "default" to be "destdir", '
            'but it was "{}"'.format(installvia_default),
        )

    def test_get_build_properties(self):
        expected_build_properties = make.MakePlugin.get_build_properties() + (
            ["configflags", "install-via"]
        )
        resulting_build_properties = autotools.AutotoolsPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_install_via_invalid_enum(self):
        self.options.install_via = "invalid"
        raised = self.assertRaises(
            RuntimeError,
            autotools.AutotoolsPlugin,
            "test-part",
            self.options,
            self.project_options,
        )

        self.assertThat(
            str(raised), Equals('Unsupported installation method: "invalid"')
        )

    def build_with_configure(self):
        plugin = autotools.AutotoolsPlugin(
            "test-part", self.options, self.project_options
        )
        os.makedirs(plugin.builddir)

        # Create both configure and autogen.sh.
        # Configure should take precedence.
        open(os.path.join(plugin.builddir, "configure"), "w").close()
        open(os.path.join(plugin.builddir, "autogen.sh"), "w").close()

        plugin.build()

        return plugin

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_configure_with_destdir(self, run_mock):
        plugin = self.build_with_configure()

        self.assertThat(run_mock.call_count, Equals(3))
        run_mock.assert_has_calls(
            [
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_configure_with_prefix(self, run_mock):
        self.options.install_via = "prefix"
        plugin = self.build_with_configure()

        self.assertThat(run_mock.call_count, Equals(3))
        run_mock.assert_has_calls(
            [
                mock.call(["./configure", "--prefix={}".format(plugin.installdir)]),
                mock.call(["make", "-j2"], env=None),
                mock.call(["make", "install"], env=None),
            ]
        )

    def build_with_autogen(self, files=None, dirs=None):
        plugin = autotools.AutotoolsPlugin(
            "test-part", self.options, self.project_options
        )
        os.makedirs(plugin.builddir)

        if not files:
            files = ["autogen.sh"]
        if not dirs:
            dirs = []

        # No configure-- only autogen.sh. Make sure it's executable.
        for filename in files:
            open(os.path.join(plugin.builddir, filename), "w").close()
            os.chmod(
                os.path.join(plugin.builddir, filename),
                stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR,
            )
        for directory in dirs:
            os.makedirs(os.path.join(plugin.builddir, directory))

        plugin.build()

        return plugin

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_autogen_with_bootstrap_dir(self, run_mock):
        plugin = self.build_with_autogen(files=["README"], dirs=["bootstrap"])

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["autoreconf", "-i"]),
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_autogen_with_destdir(self, run_mock):
        plugin = self.build_with_autogen()

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["env", "NOCONFIGURE=1", "./autogen.sh"]),
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_bootstrap_with_destdir(self, run_mock):
        plugin = self.build_with_autogen(files=["bootstrap"])

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["env", "NOCONFIGURE=1", "./bootstrap"]),
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_bootstrap_and_autogen_with_destdir(self, run_mock):
        plugin = self.build_with_autogen(files=["bootstrap", "autogen.sh"])

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["env", "NOCONFIGURE=1", "./autogen.sh"]),
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_autogen_with_prefix(self, run_mock):
        self.options.install_via = "prefix"
        plugin = self.build_with_autogen()

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["env", "NOCONFIGURE=1", "./autogen.sh"]),
                mock.call(["./configure", "--prefix={}".format(plugin.installdir)]),
                mock.call(["make", "-j2"], env=None),
                mock.call(["make", "install"], env=None),
            ]
        )

    def build_with_autoreconf(self):
        plugin = autotools.AutotoolsPlugin(
            "test-part", self.options, self.project_options
        )
        os.makedirs(plugin.sourcedir)

        # No configure or autogen.sh.

        plugin.build()

        return plugin

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_autoreconf_with_destdir(self, run_mock):
        plugin = self.build_with_autoreconf()

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["autoreconf", "-i"]),
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_autoreconf_with_prefix(self, run_mock):
        self.options.install_via = "prefix"
        plugin = self.build_with_autoreconf()

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["autoreconf", "-i"]),
                mock.call(["./configure", "--prefix={}".format(plugin.installdir)]),
                mock.call(["make", "-j2"], env=None),
                mock.call(["make", "install"], env=None),
            ]
        )

    @mock.patch.object(autotools.AutotoolsPlugin, "run")
    def test_build_autoreconf_with_disable_parallel(self, run_mock):
        self.options.disable_parallel = True
        plugin = self.build_with_autoreconf()

        self.assertThat(run_mock.call_count, Equals(4))
        run_mock.assert_has_calls(
            [
                mock.call(["autoreconf", "-i"]),
                mock.call(["./configure", "--prefix="]),
                mock.call(["make", "-j1"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch("sys.stdout")
    def test_build_nonexecutable_autogen(self, stdout_mock):
        plugin = autotools.AutotoolsPlugin(
            "test-part", self.options, self.project_options
        )
        os.makedirs(plugin.sourcedir)

        # Make a non-executable autogen.sh
        with open(os.path.join(plugin.sourcedir, "autogen.sh"), "w") as f:
            f.write("#!/bin/sh")

        patcher = mock.patch.object(autotools.AutotoolsPlugin, "run")
        run_mock = patcher.start()

        # We want to mock out every run() call except the one to autogen
        def _run(cmd, env=None):
            if "./autogen.sh" in cmd:
                patcher.stop()
                output = plugin.run(cmd, env=env)
                patcher.start()
                return output

        run_mock.side_effect = _run

        # An exception will be raised if build can't handle the non-executable
        # autogen.
        plugin.build()

    def test_fileset_ignores(self):
        plugin = autotools.AutotoolsPlugin(
            "test-part", self.options, self.project_options
        )
        expected_fileset = ["-**/*.la"]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)


class AutotoolsCrossCompilePluginTestCase(unit.TestCase):

    scenarios = [
        ("armv7l", dict(deb_arch="armhf", triplet="arm-linux-gnueabihf")),
        ("aarch64", dict(deb_arch="arm64", triplet="aarch64-linux-gnu")),
        ("i386", dict(deb_arch="i386", triplet="i386-linux-gnu")),
        ("x86_64", dict(deb_arch="amd64", triplet="x86_64-linux-gnu")),
        ("ppc64le", dict(deb_arch="ppc64el", triplet="powerpc64le-linux-gnu")),
    ]

    def setUp(self):
        super().setUp()

        class Options:
            configflags = []
            install_via = "destdir"
            disable_parallel = False
            makefile = None
            make_parameters = []
            make_install_var = "DESTDIR"
            artifacts = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions(target_deb_arch=self.deb_arch)

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.ProjectOptions.is_cross_compiling", return_value=True
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_cross_compile(self):
        plugin = autotools.AutotoolsPlugin(
            "test-part", self.options, self.project_options
        )
        plugin.enable_cross_compilation()
        plugin.build()
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["./configure", "--prefix=", "--host={}".format(self.triplet)],
                    cwd=mock.ANY,
                )
            ]
        )
