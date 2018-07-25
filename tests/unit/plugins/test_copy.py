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

import os.path
import jsonschema

from unittest.mock import Mock, patch
import testtools
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins.copy import CopyPlugin, _recursively_link
from snapcraft.internal import errors
from tests import unit


class TestCopyPlugin(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.mock_options = Mock()
        self.mock_options.source = "."
        self.mock_options.source_subdir = None
        self.mock_options.files = {}
        # setup the expected target dir in our tempdir
        self.dst_prefix = "parts/copy/install/"
        os.makedirs(self.dst_prefix)
        self.project_options = snapcraft.ProjectOptions()

    def test_get_build_properties(self):
        expected_build_properties = ["files"]
        resulting_build_properties = CopyPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_copy_plugin_any_missing_src_raises_exception(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {"src": "dst", "zzz": "zzz"}
        open("zzz", "w").close()
        c = CopyPlugin("copy", self.mock_options, self.project_options)
        c.pull()

        raised = self.assertRaises(errors.SnapcraftCopyFileNotFoundError, c.build)
        self.assertThat(raised.path, Equals(os.path.join(c.builddir, "src")))

    def test_copy_glob_does_not_match_anything(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {"src*": "dst"}
        c = CopyPlugin("copy", self.mock_options, self.project_options)
        c.pull()

        raised = self.assertRaises(errors.SnapcraftEnvironmentError, c.build)

        self.assertThat(raised.__str__(), Equals("no matches for 'src*'"))

    def test_copy_plugin_copies(self):
        self.mock_options.files = {"src": "dst"}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        open(os.path.join(c.builddir, "src"), "w").close()

        c.build()

        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, "dst")))

    def test_copy_plugin_handles_leading_slash(self):
        self.mock_options.files = {"src": "/dst"}
        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        open(os.path.join(c.builddir, "src"), "w").close()

        c.build()

        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, "dst")))

    def test_copy_plugin_handles_dot(self):
        self.mock_options.files = {"src": "."}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        open(os.path.join(c.builddir, "src"), "w").close()

        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, "src")))

    def test_copy_plugin_creates_prefixes(self):
        self.mock_options.files = {"src": "dir/dst"}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        open(os.path.join(c.builddir, "src"), "w").close()

        c.build()

        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, "dir/dst")))

    def test_copy_directories(self):
        self.mock_options.files = {"dirs1": "dir/dst"}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        os.mkdir(os.path.join(c.builddir, "dirs1"))
        open(os.path.join(c.builddir, "dirs1", "f"), "w").close()
        os.makedirs(os.path.join(c.builddir, "foo", "bar"))

        c.pull()
        c.build()
        self.assertTrue(
            os.path.exists(os.path.join(self.dst_prefix, "dir", "dst", "f"))
        )

    def test_copy_plugin_glob(self):
        self.mock_options.files = {"*.txt": "."}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        for filename in ("file-a.txt", "file-b.txt", "file-c.notxt"):
            with open(os.path.join(c.builddir, filename), "w") as datafile:
                datafile.write(filename)

        c.build()

        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, "file-a.txt")))
        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, "file-b.txt")))
        self.assertFalse(os.path.exists(os.path.join(self.dst_prefix, "file-c.notxt")))

    def test_copy_plugin_glob_with_folders(self):
        self.mock_options.files = {"foo/*": "."}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        os.makedirs(os.path.join(c.builddir, "foo", "directory"))
        open(os.path.join(c.builddir, "foo", "file1"), "w").close()
        open(os.path.join(c.builddir, "foo", "directory", "file2"), "w").close()

        c.build()

        self.assertTrue(os.path.isfile(os.path.join(c.installdir, "file1")))
        self.assertTrue(os.path.isdir(os.path.join(c.installdir, "directory")))
        self.assertTrue(
            os.path.isfile(os.path.join(c.installdir, "directory", "file2"))
        )

    def test_copy_symlinks(self):
        self.mock_options.files = {"foo/*": "baz/"}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        os.makedirs(os.path.join(c.builddir, "foo", "bar"))
        with open(os.path.join(c.builddir, "foo", "file"), "w") as f:
            f.write("foo")

        destination = os.path.join(c.installdir, "baz")

        symlinks = [
            {
                "source": "file",
                "link_name": os.path.join(c.builddir, "foo", "relative1"),
                "destination": os.path.join(destination, "relative1"),
                "expected_realpath": os.path.join(destination, "file"),
                "expected_contents": "foo",
            },
            {
                "source": "../file",
                "link_name": os.path.join(c.builddir, "foo", "bar", "relative2"),
                "destination": os.path.join(destination, "bar", "relative2"),
                "expected_realpath": os.path.join(destination, "file"),
                "expected_contents": "foo",
            },
            {
                "source": "../../baz/file",
                "link_name": os.path.join(c.builddir, "foo", "bar", "relative3"),
                "destination": os.path.join(destination, "bar", "relative3"),
                "expected_realpath": os.path.join(destination, "file"),
                "expected_contents": "foo",
            },
        ]

        for symlink in symlinks:
            os.symlink(symlink["source"], symlink["link_name"])

        c.build()

        self.assertTrue(
            os.path.isdir(destination), "Expected foo's contents to be copied into baz/"
        )
        with open(os.path.join(destination, "file"), "r") as f:
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

    def test_copy_symlinks_that_should_be_followed(self):
        self.mock_options.files = {"foo/*": "."}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        os.makedirs(os.path.join(c.builddir, "foo", "bar"))
        with open(os.path.join(c.builddir, "foo", "file"), "w") as f:
            f.write("foo")

        with open("unsnapped", "w") as f:
            f.write("bar")

        symlinks = [
            # Links with an absolute path should be followed
            {
                "source": os.path.join(c.builddir, "foo", "file"),
                "link_name": os.path.join(c.builddir, "foo", "absolute"),
                "destination": os.path.join(c.installdir, "absolute"),
                "expected_contents": "foo",
            },
            # Links with a relative path that points outside of the snap
            # should also be followed
            {
                "source": "../../../../unsnapped",
                "link_name": os.path.join(c.builddir, "foo", "bad_relative"),
                "destination": os.path.join(c.installdir, "bad_relative"),
                "expected_contents": "bar",
            },
        ]

        for symlink in symlinks:
            os.symlink(symlink["source"], symlink["link_name"])

        c.build()

        with open(os.path.join(c.installdir, "file"), "r") as f:
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

    def test_copy_symlinks_to_libc(self):
        self.mock_options.files = {"*": "."}

        c = CopyPlugin("copy", self.mock_options, self.project_options)

        # These directories are created by the pluginhandler
        os.makedirs(c.builddir)

        # Even though this symlink is absolute, since it's to libc the copy
        # plugin shouldn't try to follow it or modify it.
        libc_libs = snapcraft.repo.Repo.get_package_libraries("libc6")

        # We don't care which lib we're testing with, as long as it's a .so.
        libc_library_path = [lib for lib in libc_libs if ".so" in lib][0]
        os.symlink(libc_library_path, os.path.join(c.builddir, "libc-link"))

        c.build()

        self.assertThat(
            os.path.join(c.installdir, "libc-link"), unit.LinkExists(libc_library_path)
        )

    def test_copy_enable_cross_compilation(self):
        c = CopyPlugin("copy", self.mock_options, self.project_options)
        c.enable_cross_compilation()

    def test_schema_catches_missing_destination(self):
        with testtools.ExpectedException(
            jsonschema.exceptions.ValidationError, ".*None is not of type 'string'.*"
        ):
            jsonschema.validate({"files": {"foo": None}}, CopyPlugin.schema())

        with testtools.ExpectedException(
            jsonschema.exceptions.ValidationError, ".*'' is too short.*"
        ):
            jsonschema.validate({"files": {"foo": ""}}, CopyPlugin.schema())


class TestRecursivelyLink(unit.TestCase):
    def setUp(self):
        super().setUp()

        os.makedirs("foo/bar/baz")
        open("1", "w").close()
        open(os.path.join("foo", "2"), "w").close()
        open(os.path.join("foo", "bar", "3"), "w").close()
        open(os.path.join("foo", "bar", "baz", "4"), "w").close()

    def test_recursively_link_file_to_file(self):
        _recursively_link("1", "qux", os.getcwd())
        self.assertTrue(os.path.isfile("qux"))

    def test_recursively_link_file_into_directory(self):
        os.mkdir("qux")
        _recursively_link("1", "qux", os.getcwd())
        self.assertTrue(os.path.isfile(os.path.join("qux", "1")))

    @patch("os.chown")
    def test_recursively_link_file_into_dir_chown_permissions(self, chown_mock):
        chown_mock.side_effect = PermissionError("Nope")
        os.mkdir("qux")
        _recursively_link("1", "qux", os.getcwd())
        self.assertTrue(os.path.isfile(os.path.join("qux", "1")))

        self.assertTrue(chown_mock.called)

    def test_recursively_link_directory_to_directory(self):
        _recursively_link("foo", "qux", os.getcwd())
        self.assertTrue(os.path.isfile(os.path.join("qux", "2")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "bar", "3")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "bar", "baz", "4")))

    def test_recursively_link_directory_into_directory(self):
        os.mkdir("qux")
        _recursively_link("foo", "qux", os.getcwd())
        self.assertTrue(os.path.isfile(os.path.join("qux", "foo", "2")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "foo", "bar", "3")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "foo", "bar", "baz", "4")))

    def test_recursively_link_directory_overwrite_file_raises(self):
        open("qux", "w").close()
        raised = self.assertRaises(
            NotADirectoryError, _recursively_link, "foo", "qux", os.getcwd()
        )

        self.assertThat(
            str(raised),
            Equals("Cannot overwrite non-directory 'qux' with directory 'foo'"),
        )

    def test_recursively_link_subtree(self):
        _recursively_link("foo/bar", "qux", os.getcwd())
        self.assertTrue(os.path.isfile(os.path.join("qux", "3")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "baz", "4")))
