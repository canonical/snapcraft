# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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
import pathlib
import re
import shutil
import subprocess
from unittest import mock

import pytest
import testtools
from testtools.matchers import Equals

from snapcraft import file_utils
from snapcraft.internal import common, errors
from tests import fixture_setup, unit


class TestReplaceInFile:

    scenarios = [
        (
            "2to3",
            {
                "file_path": os.path.join("bin", "2to3"),
                "contents": "#!/foo/bar/baz/python",
                "expected": "#!/usr/bin/env python",
            },
        ),
        (
            "snapcraft",
            {
                "file_path": os.path.join("bin", "snapcraft"),
                "contents": "#!/foo/baz/python",
                "expected": "#!/usr/bin/env python",
            },
        ),
        (
            "foo",
            {
                "file_path": os.path.join("bin", "foo"),
                "contents": "foo",
                "expected": "foo",
            },
        ),
    ]

    def test_replace_in_file(self, tmp_work_path, file_path, contents, expected):
        (tmp_work_path / "bin").mkdir()

        with open(file_path, "w") as f:
            f.write(contents)

        file_utils.replace_in_file(
            "bin", re.compile(r""), re.compile(r"#!.*python"), r"#!/usr/bin/env python"
        )

        with open(file_path, "r") as f:
            assert f.read() == expected


class TestLinkOrCopyTree(unit.TestCase):
    def setUp(self):
        super().setUp()

        os.makedirs("foo/bar/baz")
        open("1", "w").close()
        open(os.path.join("foo", "2"), "w").close()
        open(os.path.join("foo", "bar", "3"), "w").close()
        open(os.path.join("foo", "bar", "baz", "4"), "w").close()

    def test_link_file_to_file_raises(self):
        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError, file_utils.link_or_copy_tree, "1", "qux"
        )

        self.assertThat(str(raised), Equals("'1' is not a directory"))

    def test_link_file_into_directory(self):
        os.mkdir("qux")
        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError, file_utils.link_or_copy_tree, "1", "qux"
        )

        self.assertThat(str(raised), Equals("'1' is not a directory"))

    def test_link_directory_to_directory(self):
        file_utils.link_or_copy_tree("foo", "qux")
        self.assertTrue(os.path.isfile(os.path.join("qux", "2")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "bar", "3")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "bar", "baz", "4")))

    def test_link_directory_overwrite_file_raises(self):
        open("qux", "w").close()
        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError, file_utils.link_or_copy_tree, "foo", "qux"
        )

        self.assertThat(
            str(raised),
            Equals("Cannot overwrite non-directory 'qux' with directory 'foo'"),
        )

    def test_link_subtree(self):
        file_utils.link_or_copy_tree("foo/bar", "qux")
        self.assertTrue(os.path.isfile(os.path.join("qux", "3")))
        self.assertTrue(os.path.isfile(os.path.join("qux", "baz", "4")))

    def test_link_symlink_to_file(self):
        # Create a symlink to a file
        os.symlink("2", os.path.join("foo", "2-link"))
        file_utils.link_or_copy_tree("foo", "qux")

        # Verify that the symlink remains a symlink
        self.assertThat(os.path.join("qux", "2-link"), unit.LinkExists("2"))

    def test_link_symlink_to_dir(self):
        os.symlink("bar", os.path.join("foo", "bar-link"))
        file_utils.link_or_copy_tree("foo", "qux")

        # Verify that the symlink remains a symlink
        self.assertThat(os.path.join("qux", "bar-link"), unit.LinkExists("bar"))


class TestLinkOrCopy(unit.TestCase):
    def setUp(self):
        super().setUp()

        os.makedirs("foo/bar/baz")
        open("1", "w").close()
        open(os.path.join("foo", "2"), "w").close()
        open(os.path.join("foo", "bar", "3"), "w").close()
        open(os.path.join("foo", "bar", "baz", "4"), "w").close()

    def test_link_file_ioerror(self):
        orig_link = os.link

        def link_and_ioerror(a, b, **kwargs):
            orig_link(a, b)
            raise IOError()

        with mock.patch("os.link") as mock_link:
            mock_link.side_effect = link_and_ioerror
            file_utils.link_or_copy("1", "foo/1")

    def test_copy_nested_file(self):
        file_utils.link_or_copy("foo/bar/baz/4", "foo2/bar/baz/4")
        self.assertTrue(os.path.isfile("foo2/bar/baz/4"))


class RequiresCommandSuccessTestCase(unit.TestCase):
    @mock.patch("subprocess.check_call")
    def test_requires_command_works(self, mock_check_call):
        mock_check_call.side_effect = [None]
        file_utils.requires_command_success("foo").__enter__()
        mock_check_call.assert_called_once_with(
            ["foo"], stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )

    @mock.patch("subprocess.check_call")
    def test_requires_command_multipe_args(self, mock_check_call):
        mock_check_call.side_effect = [None]
        file_utils.requires_command_success("foo bar baz").__enter__()
        mock_check_call.assert_called_once_with(
            ["foo", "bar", "baz"], stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )

    @mock.patch("subprocess.check_call")
    def test_requires_command_success_not_found(self, mock_check_call):
        mock_check_call.side_effect = [FileNotFoundError()]

        raised = self.assertRaises(
            errors.RequiredCommandNotFound,
            file_utils.requires_command_success("foo").__enter__,
        )

        self.assertIsInstance(raised, errors.SnapcraftError)
        self.assertThat(str(raised), Equals("'foo' not found."))

    @mock.patch("subprocess.check_call")
    def test_requires_command_success_error(self, mock_check_call):
        mock_check_call.side_effect = [subprocess.CalledProcessError(1, "x")]

        raised = self.assertRaises(
            errors.RequiredCommandFailure,
            file_utils.requires_command_success("foo").__enter__,
        )

        self.assertIsInstance(raised, errors.SnapcraftError)
        self.assertThat(str(raised), Equals("'foo' failed."))

    def test_requires_command_success_broken(self):
        raised = self.assertRaises(
            TypeError, file_utils.requires_command_success(1).__enter__
        )

        self.assertThat(str(raised), Equals("command must be a string."))

    @mock.patch("subprocess.check_call")
    def test_requires_command_success_custom_error(self, mock_check_call):
        mock_check_call.side_effect = [
            FileNotFoundError(),
            subprocess.CalledProcessError(1, "x"),
        ]

        raised = self.assertRaises(
            errors.RequiredCommandNotFound,
            file_utils.requires_command_success(
                "foo", not_found_fmt="uhm? {cmd_list!r} -> {command}"
            ).__enter__,
        )

        self.assertThat(str(raised), Equals("uhm? ['foo'] -> foo"))

        raised = self.assertRaises(
            errors.RequiredCommandFailure,
            file_utils.requires_command_success(
                "foo", failure_fmt="failed {cmd_list!r} -> {command}"
            ).__enter__,
        )

        self.assertThat(str(raised), Equals("failed ['foo'] -> foo"))


class RequiresPathExistsTestCase(unit.TestCase):
    def setUp(self):
        super(RequiresPathExistsTestCase, self).setUp()
        with open("bar", "w") as fd:
            fd.write("test")

    def test_requires_path_exists_works(self):
        file_utils.requires_path_exists("bar").__enter__()

    def test_requires_path_exists_fails(self):
        raised = self.assertRaises(
            errors.RequiredPathDoesNotExist,
            file_utils.requires_path_exists("foo").__enter__,
        )

        self.assertIsInstance(raised, errors.SnapcraftError)
        self.assertThat(str(raised), Equals("Required path does not exist: 'foo'"))

    def test_requires_path_exists_custom_error(self):
        raised = self.assertRaises(
            errors.RequiredPathDoesNotExist,
            file_utils.requires_path_exists(
                "foo", error_fmt="what? {path!r}"
            ).__enter__,
        )

        self.assertThat(str(raised), Equals("what? 'foo'"))


class TestGetLinkerFromFile(unit.TestCase):
    def test_get_linker_version_from_basename(self):
        self.assertThat(
            file_utils.get_linker_version_from_file("ld-2.26.so"), Equals("2.26")
        )

    def test_get_linker_version_from_path(self):
        self.assertThat(
            file_utils.get_linker_version_from_file("/lib/x86/ld-2.23.so"),
            Equals("2.23"),
        )


class TestGetLinkerFromFileErrors(unit.TestCase):
    def test_bad_file_formatlinker_raises_exception(self):
        self.assertRaises(
            errors.SnapcraftEnvironmentError,
            file_utils.get_linker_version_from_file,
            linker_file="lib64/ld-linux-x86-64.so.2",
        )


_BIN_PATHS = [
    os.path.join("usr", "local", "sbin"),
    os.path.join("usr", "local", "bin"),
    os.path.join("usr", "sbin"),
    os.path.join("usr", "bin"),
    os.path.join("sbin"),
    os.path.join("bin"),
]


class TestGetToolPath:

    scenarios = [
        (i, dict(tool_path=pathlib.Path(i) / "tool-command")) for i in _BIN_PATHS
    ]

    def test_get_tool_from_host_path(self, monkeypatch, tool_path, fake_exists):
        abs_tool_path = pathlib.Path("/") / tool_path
        fake_exists.paths = [abs_tool_path]
        monkeypatch.setattr(shutil, "which", lambda x: abs_tool_path.as_posix())

        assert file_utils.get_snap_tool_path("tool-command") == abs_tool_path.as_posix()

    def test_get_tool_from_snapcraft_snap_path(self, in_snap, tool_path, fake_exists):
        abs_tool_path = pathlib.Path("/snap/snapcraft/current") / tool_path
        fake_exists.paths = [abs_tool_path]

        assert file_utils.get_snap_tool_path("tool-command") == abs_tool_path.as_posix()

    def test_get_tool_from_docker_snap_path(
        self, monkeypatch, in_snap, tool_path, fake_exists
    ):
        abs_tool_path = pathlib.Path("/snap/snapcraft/current") / tool_path
        fake_exists.paths = [abs_tool_path]
        monkeypatch.setattr(common, "is_process_container", lambda: True)

        assert file_utils.get_snap_tool_path("tool-command") == abs_tool_path.as_posix()


def test_get_host_tool_finds_command(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda x: "/usr/bin/foo")

    assert file_utils.get_host_tool_path(command_name="foo", package_name="foo")


def test_get_host_tool_failure(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda x: None)

    with pytest.raises(errors.SnapcraftHostToolNotFoundError) as error:
        file_utils.get_host_tool_path(command_name="foo", package_name="foo-pkg")

        assert error.command_name == "foo"
        assert error.package_name == "foo-pkg"


class GetToolPathErrorsTest(testtools.TestCase):
    def test_get_snap_tool_path_fails(self):
        self.assertRaises(
            errors.ToolMissingError,
            file_utils.get_snap_tool_path,
            "non-existent-tool-command",
        )

    def test_get_snap_tool_path_in_container_fails_root(self):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        self.assertRaises(
            errors.ToolMissingError,
            file_utils.get_snap_tool_path,
            "non-existent-tool-command",
        )
