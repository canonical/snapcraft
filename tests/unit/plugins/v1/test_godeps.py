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

import os
from unittest import mock

from testtools.matchers import Contains, Equals, HasLength, Not

from snapcraft.internal import errors
from snapcraft.plugins.v1 import godeps
from tests import unit

from . import PluginsV1BaseTestCase


class GodepsPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        class Options:
            source = "src"
            go_channel = "1.15/stable"
            go_importpath = "github.com/foo/bar"
            godeps_file = "dependencies.tsv"
            go_packages = []

        self.options = Options()


class GoPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = godeps.GodepsPlugin.schema()

        properties = schema["properties"]
        for expected in ["godeps-file", "go-importpath"]:
            self.assertTrue(
                expected in properties,
                "Expected {!r} to be included in properties".format(expected),
            )

        # Check go-channel
        go_channel = properties["go-channel"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in go_channel,
                "Expected {!r} to be included in 'go-channel'".format(expected),
            )

        go_channel_type = go_channel["type"]
        self.assertThat(
            go_channel_type,
            Equals("string"),
            'Expected "go-channel" "type" to be "string", but '
            'it was "{}"'.format(go_channel_type),
        )

        go_channel_default = go_channel["default"]
        self.assertThat(
            go_channel_default,
            Equals("1.15/stable"),
            'Expected "go-channel" "default" to be '
            '"1.15/stable", but it was "{}"'.format(go_channel_default),
        )

        # Check godeps-file
        godeps_file = properties["godeps-file"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in godeps_file,
                "Expected {!r} to be included in 'godeps-file'".format(expected),
            )

        godeps_file_type = godeps_file["type"]
        self.assertThat(
            godeps_file_type,
            Equals("string"),
            'Expected "godeps-file" "type" to be "string", but '
            'it was "{}"'.format(godeps_file_type),
        )

        godeps_file_default = godeps_file["default"]
        self.assertThat(
            godeps_file_default,
            Equals("dependencies.tsv"),
            'Expected "godeps-file" "default" to be '
            '"dependencies.tsv", but it was "{}"'.format(godeps_file_default),
        )

        # Check go-importpath
        go_importpath = properties["go-importpath"]
        for expected in ["type"]:
            self.assertTrue(
                expected in go_importpath,
                "Expected {!r} to be included in 'go-importpath'".format(expected),
            )

        go_importpath_type = go_importpath["type"]
        self.assertThat(
            go_importpath_type,
            Equals("string"),
            'Expected "go-importpath" "type" to be "string", but '
            'it was "{}"'.format(go_importpath_type),
        )

        # go-importpath should be required
        self.assertTrue(
            "go-importpath" in schema["required"],
            'Expeced "go-importpath" to be required',
        )

        # Check go-packages
        go_packages = properties["go-packages"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in go_packages,
                "Expected {!r} to be included in 'go-packages'".format(expected),
            )

        go_packages_type = go_packages["type"]
        self.assertThat(
            go_packages_type,
            Equals("array"),
            'Expected "go-packages" "type" to be "array", but '
            'it was "{}"'.format(go_packages_type),
        )

        go_packages_default = go_packages["default"]
        self.assertThat(
            go_packages_default,
            Equals([]),
            'Expected "go-packages" "default" to be "[]", but '
            'it was "{}"'.format(go_packages_default),
        )

    def test_get_pull_properties(self):
        expected_pull_properties = ["go-channel", "godeps-file", "go-importpath"]
        resulting_pull_properties = godeps.GodepsPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        self.assertThat(resulting_pull_properties, Equals(expected_pull_properties))

    def test_get_build_properties(self):
        expected_build_properties = ["go-packages"]
        resulting_build_properties = godeps.GodepsPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        self.assertThat(resulting_build_properties, Equals(expected_build_properties))


class GodepsPluginTest(GodepsPluginBaseTest):
    def test_build_environment(self):
        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        os.makedirs(plugin.options.source)
        os.makedirs(os.path.join(plugin.installdir, "lib"))
        os.makedirs(os.path.join(plugin.installdir, "usr", "lib"))
        os.makedirs(os.path.join(plugin.project.stage_dir, "lib"))
        os.makedirs(os.path.join(plugin.project.stage_dir, "usr", "lib"))
        plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(6))
        for call_args in self.run_mock.call_args_list:
            env = call_args[1]["env"]
            self.assertTrue("GOPATH" in env, "Expected environment to include GOPATH")
            self.assertThat(env["GOPATH"], Equals(plugin._gopath))

            self.assertTrue("PATH" in env, "Expected environment to include PATH")
            self.assertTrue(
                os.path.join(plugin._gopath, "bin") in env["PATH"],
                "Expected $PATH to include $GOPATH/bin",
            )

            self.assertTrue(
                "CGO_LDFLAGS" in env, "Expected environment to include CGO_LDFLAGS"
            )
            expected_flags = [
                "-L{}/lib".format(plugin.installdir),
                "-L{}/usr/lib".format(plugin.installdir),
                "-L{}/lib".format(plugin.project.stage_dir),
                "-L{}/usr/lib".format(plugin.project.stage_dir),
            ]
            for flag in expected_flags:
                self.assertTrue(
                    flag in env["CGO_LDFLAGS"],
                    "Expected $CGO_LDFLAGS to include {!r}, but it was "
                    '"{}"'.format(flag, env["CGO_LDFLAGS"]),
                )

    def test_pull(self):
        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        os.makedirs(plugin.options.source)

        plugin.pull()

        assert self.run_mock.mock_calls == [
            mock.call(
                ["go", "get", "-d", "github.com/rogpeppe/godeps"],
                cwd=plugin._gopath_src,
                env=mock.ANY,
            ),
            mock.call(
                ["git", "checkout", "4e9e0ee19b60b13eb79915933f44d8ed5f268bdd"],
                cwd=plugin._gopath_src + "/github.com/pelletier/go-toml",
                env=mock.ANY,
            ),
            mock.call(
                ["git", "checkout", "d6ce6262d87e3a4e153e86023ff56ae771554a41"],
                cwd=plugin._gopath_src + "/github.com/kisielk/gotool",
                env=mock.ANY,
            ),
            mock.call(
                ["git", "checkout", "1937f90a1bb43667aff4059b1bab13eb15121e8e"],
                cwd=plugin._gopath_src + "/golang.org/x/tools",
                env=mock.ANY,
            ),
            mock.call(
                ["go", "install", "github.com/rogpeppe/godeps"],
                cwd=plugin._gopath_src,
                env=mock.ANY,
            ),
            mock.call(
                [
                    "godeps",
                    "-t",
                    "-u",
                    os.path.join(plugin.sourcedir, self.options.godeps_file),
                ],
                cwd=plugin._gopath_src,
                env=mock.ANY,
            ),
        ]

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_pkg))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

        sourcedir = os.path.join(plugin._gopath_src, "github.com", "foo", "bar")
        self.assertTrue(os.path.islink(sourcedir))
        self.assertThat(os.readlink(sourcedir), Equals(plugin.sourcedir))

    def test_clean_pull(self):
        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        os.makedirs(plugin.options.source)

        plugin.pull()

        self.assertTrue(os.path.exists(plugin._gopath))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin._gopath))

    def test_build(self):
        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        os.makedirs(plugin.options.source)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()

        open(os.path.join(plugin._gopath_bin, "test-binary"), "w").close()
        open(os.path.join(plugin._gopath_bin, "godeps"), "w").close()
        plugin.build()

        self.assertThat(self.run_mock.call_count, Equals(1))
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["go", "install", "./github.com/foo/bar/..."],
                    cwd=plugin._gopath_src,
                    env=mock.ANY,
                )
            ]
        )

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
        self.assertTrue(
            os.path.exists(os.path.join(plugin.installdir, "bin", "test-binary"))
        )

        # Assert that the godeps binary was NOT copied
        self.assertFalse(
            os.path.exists(os.path.join(plugin.installdir, "bin", "godeps"))
        )

    def test_build_with_go_packages(self):
        self.options.go_packages = ["github.com/foo/bar/cmd/my-command"]

        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        os.makedirs(plugin.options.source)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()

        open(os.path.join(plugin._gopath_bin, "test-binary"), "w").close()
        open(os.path.join(plugin._gopath_bin, "godeps"), "w").close()
        plugin.build()

        self.assertThat(self.run_mock.call_count, Equals(1))
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["go", "install", "github.com/foo/bar/cmd/my-command"],
                    cwd=plugin._gopath_src,
                    env=mock.ANY,
                )
            ]
        )

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
        self.assertTrue(
            os.path.exists(os.path.join(plugin.installdir, "bin", "test-binary"))
        )

        # Assert that the godeps binary was NOT copied
        self.assertFalse(
            os.path.exists(os.path.join(plugin.installdir, "bin", "godeps"))
        )

    def test_clean_build(self):
        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        os.makedirs(plugin.options.source)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin._gopath_pkg)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_pkg))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

        plugin.clean_build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_pkg))
        self.assertFalse(os.path.exists(plugin._gopath_bin))


class GodepsPluginToolSetupTest(GodepsPluginBaseTest):
    def test_snap(self):
        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        self.assertThat(plugin.build_packages, Not(Contains("golang-go")))
        self.assertThat(plugin.build_snaps, Contains("go/1.15/stable"))

    def test_build_packages(self):
        self.options.go_channel = ""

        plugin = godeps.GodepsPlugin("test-part", self.options, self.project)

        self.assertThat(plugin.build_packages, Contains("golang-go"))
        self.assertThat(plugin.build_snaps, Not(Contains("go/1.15/stable")))


class GodepsPluginUnsupportedBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            source = "dir"
            go_channel = "1.15/stable"

        self.options = Options()

    def test_unsupported_base_using_snap(self):
        self.assertRaises(
            errors.PluginBaseError,
            godeps.GodepsPlugin,
            "test-part",
            self.options,
            self.project,
        )

    def test_unsupported_base_using_without_snap_raises(self):
        self.options.go_channel = ""

        self.assertRaises(
            errors.PluginBaseError,
            godeps.GodepsPlugin,
            "test-part",
            self.options,
            self.project,
        )
