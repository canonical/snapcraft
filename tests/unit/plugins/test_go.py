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

from unittest import mock
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import go
from tests import fixture_setup, unit


class GoPluginCrossCompileTestCase(unit.TestCase):

    scenarios = [
        ("armv7l", dict(deb_arch="armhf", go_arch="arm")),
        ("aarch64", dict(deb_arch="arm64", go_arch="arm64")),
        ("i386", dict(deb_arch="i386", go_arch="386")),
        ("x86_64", dict(deb_arch="amd64", go_arch="amd64")),
        ("ppc64le", dict(deb_arch="ppc64el", go_arch="ppc64le")),
    ]

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions(target_deb_arch=self.deb_arch)

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.ProjectOptions.is_cross_compiling")
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_cross_compile(self):
        class Options:
            source = ""
            go_packages = ["github.com/gotools/vet"]
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(1))
        for call_args in self.run_mock.call_args_list:
            env = call_args[1]["env"]
            self.assertIn("CC", env)
            self.assertThat(
                env["CC"], Equals("{}-gcc".format(self.project_options.arch_triplet))
            )
            self.assertIn("CXX", env)
            self.assertThat(
                env["CXX"], Equals("{}-g++".format(self.project_options.arch_triplet))
            )
            self.assertIn("CGO_ENABLED", env)
            self.assertThat(env["CGO_ENABLED"], Equals("1"))
            self.assertIn("GOARCH", env)
            self.assertThat(env["GOARCH"], Equals(self.go_arch))
            if self.deb_arch == "armhf":
                self.assertIn("GOARM", env)
                self.assertThat(env["GOARM"], Equals("7"))


class GoPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.CleanEnvironment())

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.internal.common.run_output")
        self.run_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("sys.stdout")
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = go.GoPlugin.schema()

        properties = schema["properties"]
        for expected in ["go-packages", "go-importpath", "go-buildtags"]:
            self.assertTrue(
                expected in properties,
                "Expected {!r} to be included in properties".format(expected),
            )

        # Check go-packages
        go_packages = properties["go-packages"]
        for expected in ["type", "default", "minitems", "uniqueItems", "items"]:
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
            'Expected "go-packages" "default" to be '
            '"d[]", but it was "{}"'.format(go_packages_default),
        )

        go_packages_minitems = go_packages["minitems"]
        self.assertThat(
            go_packages_minitems,
            Equals(1),
            'Expected "go-packages" "minitems" to be 1, but '
            "it was {}".format(go_packages_minitems),
        )

        self.assertTrue(go_packages["uniqueItems"])

        go_packages_items = go_packages["items"]
        self.assertTrue(
            "type" in go_packages_items,
            'Expected "type" to be included in "go-packages" ' '"items"',
        )

        go_packages_items_type = go_packages_items["type"]
        self.assertThat(
            go_packages_items_type,
            Equals("string"),
            'Expected "go-packages" "item" "type" to be '
            '"string", but it was "{}"'.format(go_packages_items_type),
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

        go_importpath_default = go_importpath["default"]
        self.assertThat(
            go_importpath_default,
            Equals(""),
            'Expected "go-default" "default" to be "'
            '", but '
            'it was "{}"'.format(go_importpath_default),
        )

        # Check go-buildtags
        go_buildtags = properties["go-buildtags"]
        for expected in ["type", "default", "minitems", "uniqueItems", "items"]:
            self.assertTrue(
                expected in go_buildtags,
                "Expected {!r} to be included in 'go-buildtags'".format(expected),
            )

        go_buildtags_type = go_buildtags["type"]
        self.assertThat(
            go_buildtags_type,
            Equals("array"),
            'Expected "go-buildtags" "type" to be "array", but '
            'it was "{}"'.format(go_buildtags_type),
        )

        go_buildtags_default = go_buildtags["default"]
        self.assertThat(
            go_buildtags_default,
            Equals([]),
            'Expected "go-buildtags" "default" to be "[]", but '
            'it was "{}"'.format(go_buildtags_type),
        )

        go_buildtags_minitems = go_buildtags["minitems"]
        self.assertThat(
            go_buildtags_minitems,
            Equals(1),
            'Expected "go-buildtags" "minitems" to be 1, but '
            "it was {}".format(go_buildtags_minitems),
        )

        self.assertTrue(go_buildtags["uniqueItems"])

        go_buildtags_items = go_buildtags["items"]
        self.assertTrue(
            "type" in go_buildtags_items,
            'Expected "type" to be included in "go-buildtags" ' '"items"',
        )

        go_buildtags_items_type = go_buildtags_items["type"]
        self.assertThat(
            go_buildtags_items_type,
            Equals("string"),
            'Expected "go-buildtags" "item" "type" to be '
            '"string", but it was "{}"'.format(go_packages_items_type),
        )

        # Check required properties
        self.assertNotIn("required", schema)

    def test_get_pull_properties(self):
        expected_pull_properties = ["go-packages"]
        resulting_pull_properties = go.GoPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ["go-packages", "go-buildtags"]
        resulting_build_properties = go.GoPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_pull_local_sources(self):
        class Options:
            source = "dir"
            go_packages = []
            go_importpath = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, "main.go"), "w").close()

        plugin.pull()

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["go", "get", "-t", "-d", "./dir/..."],
                    cwd=plugin._gopath_src,
                    env=mock.ANY,
                )
            ]
        )

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_no_local_source_with_go_packages(self):
        class Options:
            source = None
            go_packages = ["github.com/gotools/vet"]
            go_importpath = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["go", "get", "-t", "-d", plugin.options.go_packages[0]],
                    env=mock.ANY,
                    cwd=plugin._gopath_src,
                )
            ]
        )

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_with_local_sources_or_go_packages(self):
        class Options:
            source = None
            go_packages = []
            go_importpath = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)
        plugin.pull()

        self.run_mock.assert_has_calls([])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_build_with_local_sources(self):
        class Options:
            source = "dir"
            go_packages = []
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, "main.go"), "w").close()

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()
        self.run_output_mock.reset_mock()
        self.run_output_mock.return_value = "dir/pkg/main main"

        plugin.build()

        self.run_output_mock.assert_called_once_with(
            ["go", "list", "-f", "{{.ImportPath}} {{.Name}}", "./dir/..."],
            cwd=plugin._gopath_src,
            env=mock.ANY,
        )

        binary = os.path.join(plugin._gopath_bin, "main")
        self.run_mock.assert_called_once_with(
            ["go", "build", "-o", binary, "dir/pkg/main"],
            cwd=plugin._gopath_src,
            env=mock.ANY,
        )

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

    def test_build_go_packages(self):
        class Options:
            source = ""
            go_packages = ["github.com/gotools/vet"]
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)
        # fake some binaries
        binary = os.path.join(plugin._gopath_bin, "vet")
        open(binary, "w").close()

        self.run_mock.reset_mock()
        plugin.build()

        self.run_mock.assert_called_once_with(
            ["go", "build", "-o", binary, plugin.options.go_packages[0]],
            cwd=plugin._gopath_src,
            env=mock.ANY,
        )

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
        vet_binary = os.path.join(plugin.installdir, "bin", "vet")
        self.assertTrue(os.path.exists(vet_binary))

    def test_build_with_no_local_sources_or_go_packages(self):
        class Options:
            source = ""
            go_packages = []
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.run_mock.assert_has_calls([])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

    def test_clean_build(self):
        class Options:
            source = "dir"
            go_packages = []
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin._gopath_pkg)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

        plugin.clean_build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))
        self.assertFalse(os.path.exists(plugin._gopath_pkg))

    def test_clean_pull(self):
        class Options:
            source = "dir"
            go_packages = []
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, "main.go"), "w").close()

        plugin.pull()

        self.assertTrue(os.path.exists(plugin._gopath))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin._gopath))

    def test_build_with_local_sources_and_go_importpath(self):
        class Options:
            source = "dir"
            go_packages = []
            go_importpath = "github.com/snapcore/launcher"
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, "main.go"), "w").close()

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)
        self.run_output_mock.return_value = "github.com/snapcore/launcher main"

        plugin.build()

        self.run_output_mock.assert_called_once_with(
            [
                "go",
                "list",
                "-f",
                "{{.ImportPath}} {{.Name}}",
                "./github.com/snapcore/launcher/...",
            ],
            cwd=plugin._gopath_src,
            env=mock.ANY,
        )

        binary = os.path.join(plugin._gopath_bin, "launcher")
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["go", "get", "-t", "-d", "./github.com/snapcore/launcher/..."],
                    cwd=plugin._gopath_src,
                    env=mock.ANY,
                ),
                mock.call(
                    ["go", "build", "-o", binary, "github.com/snapcore/launcher"],
                    cwd=plugin._gopath_src,
                    env=mock.ANY,
                ),
            ]
        )

        self.assertTrue(
            os.path.exists(
                os.path.join(plugin._gopath_src, plugin.options.go_importpath)
            )
        )

    def test_build_environment(self):
        class Options:
            source = "dir"
            go_packages = []
            go_importpath = ""
            go_buildtags = ""

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, "main.go"), "w").close()
        os.makedirs(os.path.join(plugin.installdir, "lib"))
        os.makedirs(os.path.join(plugin.installdir, "usr", "lib"))
        os.makedirs(os.path.join(plugin.project.stage_dir, "lib"))
        os.makedirs(os.path.join(plugin.project.stage_dir, "usr", "lib"))
        plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(1))
        for call_args in self.run_mock.call_args_list:
            env = call_args[1]["env"]
            self.assertTrue("GOPATH" in env, "Expected environment to include GOPATH")
            self.assertThat(env["GOPATH"], Equals(plugin._gopath))

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

    def test_build_with_buildtag(self):
        class Options:
            source = "dir"
            go_importpath = ""
            go_packages = []
            go_buildtags = ["testbuildtag1", "testbuildtag2"]

        plugin = go.GoPlugin("test-part", Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()
        self.run_output_mock.return_value = "dir/pkg/main main"

        plugin.build()

        self.run_output_mock.assert_called_once_with(
            ["go", "list", "-f", "{{.ImportPath}} {{.Name}}", "./dir/..."],
            cwd=plugin._gopath_src,
            env=mock.ANY,
        )

        binary = os.path.join(plugin._gopath_bin, "main")
        self.run_mock.assert_called_once_with(
            [
                "go",
                "build",
                "-o",
                binary,
                "-tags=testbuildtag1,testbuildtag2",
                "dir/pkg/main",
            ],
            cwd=plugin._gopath_src,
            env=mock.ANY,
        )

    def test_build_stage_packages_default(self):
        class Options:
            source = "dir"

        plugin = go.GoPlugin("test-part", Options(), self.project_options)
        self.assertIn("golang-go", plugin.build_packages)

    def test_build_stage_packages_with_go_build_snap(self):
        class Options:
            source = "dir"
            go_importpath = ""
            build_snaps = ["go"]

        plugin = go.GoPlugin("test-part", Options(), self.project_options)
        self.assertNotIn("golang-go", plugin.build_packages)

    def test_build_stage_packages_with_go_build_snap_channel(self):
        class Options:
            source = "dir"
            go_importpath = ""
            build_snaps = ["go/foo"]

        plugin = go.GoPlugin("test-part", Options(), self.project_options)
        self.assertNotIn("golang-go", plugin.build_packages)
