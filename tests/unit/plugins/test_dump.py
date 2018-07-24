# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017-2018 Canonical Ltd
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

from testtools.matchers import Equals

import snapcraft
from snapcraft.plugins.dump import DumpInvalidSymlinkError, DumpPlugin
from tests import unit


class DumpPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.project_options = snapcraft.ProjectOptions()

        class Options:
            source = "."

        self.options = Options()

    def test_dumping_nothing(self):
        plugin = DumpPlugin("dump", self.options, self.project_options)
        os.makedirs(plugin.builddir)
        plugin.build()

        self.assertThat(os.listdir(plugin.installdir), Equals([]))

    def test_dumping_with_contents(self):
        plugin = DumpPlugin("dump", self.options, self.project_options)

        os.makedirs(plugin.builddir)
        open(os.path.join(plugin.builddir, "file1"), "w").close()
        open(os.path.join(plugin.builddir, "file2"), "w").close()
        os.mkdir(os.path.join(plugin.builddir, "dir1"))
        open(os.path.join(plugin.builddir, "dir1", "subfile1"), "w").close()

        plugin.build()

        contents = os.listdir(plugin.installdir)
        contents.sort()
        self.assertThat(contents, Equals(["dir1", "file1", "file2"]))
        self.assertThat(
            os.listdir(os.path.join(plugin.installdir, "dir1")), Equals(["subfile1"])
        )

    def test_dump_symlinks(self):
        plugin = DumpPlugin("dump", self.options, self.project_options)

        os.makedirs(os.path.join(plugin.builddir, "subdir"))
        with open(os.path.join(plugin.builddir, "file"), "w") as f:
            f.write("foo")

        symlinks = [
            {
                "source": "file",
                "link_name": os.path.join(plugin.builddir, "relative1"),
                "destination": os.path.join(plugin.installdir, "relative1"),
                "expected_realpath": os.path.join(plugin.installdir, "file"),
                "expected_contents": "foo",
            },
            {
                "source": os.path.join("..", "file"),
                "link_name": os.path.join(plugin.builddir, "subdir", "relative2"),
                "destination": os.path.join(plugin.installdir, "subdir", "relative2"),
                "expected_realpath": os.path.join(plugin.installdir, "file"),
                "expected_contents": "foo",
            },
            {
                "source": os.path.join("..", "..", "install", "file"),
                "link_name": os.path.join(plugin.builddir, "subdir", "relative3"),
                "destination": os.path.join(plugin.installdir, "subdir", "relative3"),
                "expected_realpath": os.path.join(plugin.installdir, "file"),
                "expected_contents": "foo",
            },
        ]

        for symlink in symlinks:
            os.symlink(symlink["source"], symlink["link_name"])

        plugin.build()

        with open(os.path.join(plugin.installdir, "file"), "r") as f:
            self.assertThat(f.read(), Equals("foo"))

        for symlink in symlinks:
            destination = symlink["destination"]
            self.assertTrue(
                os.path.islink(destination),
                "Expected {!r} to be a symlink".format(destination),
            )

            self.assertThat(
                os.path.realpath(destination),
                Equals(symlink["expected_realpath"]),
                "Expected {!r} to be a relative path to {!r}".format(
                    destination, symlink["expected_realpath"]
                ),
            )

            with open(destination, "r") as f:
                self.assertThat(f.read(), Equals(symlink["expected_contents"]))

    def test_dump_symlinks_that_should_be_followed(self):
        # TODO: Move to an integration test
        plugin = DumpPlugin("dump", self.options, self.project_options)

        os.makedirs(os.path.join(plugin.builddir, "src"))
        with open(os.path.join(plugin.builddir, "src", "file"), "w") as f:
            f.write("foo")

        with open("unsnapped", "w") as f:
            f.write("bar")

        symlinks = [
            # Links with an absolute path should be followed
            {
                "source": os.path.abspath(os.path.join(plugin.builddir, "src", "file")),
                "link_name": os.path.join(plugin.builddir, "src", "absolute"),
                "destination": os.path.join(plugin.installdir, "src", "absolute"),
                "expected_contents": "foo",
            },
            # Links with a relative path that points outside of the snap
            # should also be followed
            {
                "source": "../../../../unsnapped",
                "link_name": os.path.join(plugin.builddir, "src", "bad_relative"),
                "destination": os.path.join(plugin.installdir, "src", "bad_relative"),
                "expected_contents": "bar",
            },
        ]

        for symlink in symlinks:
            os.symlink(symlink["source"], symlink["link_name"])

        plugin.build()

        with open(os.path.join(plugin.installdir, "src", "file"), "r") as f:
            self.assertThat(f.read(), Equals("foo"))

        for symlink in symlinks:
            destination = symlink["destination"]
            self.assertFalse(
                os.path.islink(destination),
                "Expected {!r} to be a copy rather than a "
                "symlink".format(destination),
            )

            with open(destination, "r") as f:
                self.assertThat(f.read(), Equals(symlink["expected_contents"]))

    def test_dump_symlinks_to_libc(self):
        plugin = DumpPlugin("dump", self.options, self.project_options)
        os.makedirs(plugin.builddir)

        # Even though this symlink is absolute, since it's to libc the copy
        # plugin shouldn't try to follow it or modify it.
        libc_libs = snapcraft.repo.Repo.get_package_libraries("libc6")

        # We don't care which lib we're testing with, as long as it's a .so.
        libc_library_path = [lib for lib in libc_libs if ".so" in lib][0]
        os.symlink(libc_library_path, os.path.join(plugin.builddir, "libc-link"))

        plugin.build()

        self.assertThat(
            os.path.join(plugin.installdir, "libc-link"),
            unit.LinkExists(libc_library_path),
        )

    def test_dump_broken_symlink(self):
        self.options.source = "src"
        plugin = DumpPlugin("dump", self.options, self.project_options)

        os.makedirs(os.path.join(plugin.builddir, "src"))
        with open(os.path.join(plugin.builddir, "src", "file"), "w") as f:
            f.write("foo")

        with open("unsnapped", "w") as f:
            f.write("bar")

        symlinks = [
            # This symlink is valid in source, but broken when snapped.
            {
                "source": "../../../unsnapped",
                "link_name": os.path.join(plugin.builddir, "src", "bad_relative"),
                "destination": os.path.join(plugin.installdir, "src", "bad_relative"),
                "expected_contents": "bar",
            }
        ]

        for symlink in symlinks:
            os.symlink(symlink["source"], symlink["link_name"])

        plugin.pull()

        raised = self.assertRaises(DumpInvalidSymlinkError, plugin.build)

        self.assertThat(
            raised.path, Equals(os.path.join(plugin.builddir, "src", "bad_relative"))
        )

    def test_dump_enable_cross_compilation(self):
        plugin = DumpPlugin("dump", self.options, self.project_options)
        plugin.enable_cross_compilation()
