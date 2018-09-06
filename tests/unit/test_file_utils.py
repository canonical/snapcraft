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
import re
import subprocess
from unittest import mock

import fixtures
import testtools
import testscenarios
from testtools.matchers import Equals

from snapcraft import file_utils
from snapcraft.internal.errors import (
    RequiredCommandFailure,
    RequiredCommandNotFound,
    RequiredPathDoesNotExist,
    SnapcraftEnvironmentError,
    SnapcraftError,
)
from tests import fixture_setup, unit


class ReplaceInFileTestCase(unit.TestCase):

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

    def test_replace_in_file(self):
        os.makedirs("bin")

        with open(self.file_path, "w") as f:
            f.write(self.contents)

        file_utils.replace_in_file(
            "bin", re.compile(r""), re.compile(r"#!.*python"), r"#!/usr/bin/env python"
        )

        with open(self.file_path, "r") as f:
            self.assertThat(f.read(), Equals(self.expected))

    def test_replace_in_file_with_permission_error(self):
        os.makedirs("bin")
        file_info = {
            "path": os.path.join("bin", "readonly"),
            "contents": "#!/foo/bar/baz/python",
            "expected": "#!/foo/bar/baz/python",
        }
        with open(file_info["path"], "w") as f:
            f.write(file_info["contents"])

        # Use a mock here to force a PermissionError, even within a docker
        # container which always runs with elevated permissions
        with mock.patch("snapcraft.file_utils.open", side_effect=PermissionError("")):
            file_utils.replace_in_file(
                "bin",
                re.compile(r""),
                re.compile(r"#!.*python"),
                r"#!/usr/bin/env python",
            )

        with open(file_info["path"], "r") as f:
            self.assertThat(f.read(), Equals(file_info["expected"]))


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
            NotADirectoryError, file_utils.link_or_copy_tree, "1", "qux"
        )

        self.assertThat(str(raised), Equals("'1' is not a directory"))

    def test_link_file_into_directory(self):
        os.mkdir("qux")
        raised = self.assertRaises(
            NotADirectoryError, file_utils.link_or_copy_tree, "1", "qux"
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
            NotADirectoryError, file_utils.link_or_copy_tree, "foo", "qux"
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


class ExecutableExistsTestCase(unit.TestCase):
    def test_file_does_not_exist(self):
        workdir = self.useFixture(fixtures.TempDir()).path
        self.assertFalse(
            file_utils.executable_exists(os.path.join(workdir, "doesnotexist"))
        )

    def test_file_exists_but_not_readable(self):
        workdir = self.useFixture(fixtures.TempDir()).path
        path = os.path.join(workdir, "notreadable")
        with open(path, "wb"):
            pass
        os.chmod(path, 0)

        self.assertFalse(file_utils.executable_exists(path))

    def test_file_exists_but_not_executable(self):
        workdir = self.useFixture(fixtures.TempDir()).path
        path = os.path.join(workdir, "notexecutable")
        with open(path, "wb"):
            pass
        os.chmod(path, 0o444)

        self.assertFalse(file_utils.executable_exists(path))

    def test_executable_exists_and_executable(self):
        workdir = self.useFixture(fixtures.TempDir()).path
        path = os.path.join(workdir, "notexecutable")
        with open(path, "wb"):
            pass
        os.chmod(path, 0o555)

        self.assertTrue(file_utils.executable_exists(path))


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
            RequiredCommandNotFound,
            file_utils.requires_command_success("foo").__enter__,
        )

        self.assertIsInstance(raised, SnapcraftError)
        self.assertThat(str(raised), Equals("'foo' not found."))

    @mock.patch("subprocess.check_call")
    def test_requires_command_success_error(self, mock_check_call):
        mock_check_call.side_effect = [subprocess.CalledProcessError(1, "x")]

        raised = self.assertRaises(
            RequiredCommandFailure, file_utils.requires_command_success("foo").__enter__
        )

        self.assertIsInstance(raised, SnapcraftError)
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
            RequiredCommandNotFound,
            file_utils.requires_command_success(
                "foo", not_found_fmt="uhm? {cmd_list!r} -> {command}"
            ).__enter__,
        )

        self.assertThat(str(raised), Equals("uhm? ['foo'] -> foo"))

        raised = self.assertRaises(
            RequiredCommandFailure,
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
            RequiredPathDoesNotExist, file_utils.requires_path_exists("foo").__enter__
        )

        self.assertIsInstance(raised, SnapcraftError)
        self.assertThat(str(raised), Equals("Required path does not exist: 'foo'"))

    def test_requires_path_exists_custom_error(self):
        raised = self.assertRaises(
            RequiredPathDoesNotExist,
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
            SnapcraftEnvironmentError,
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


class GetToolPathTest(testscenarios.WithScenarios, testtools.TestCase):

    scenarios = [
        (i, dict(tool_path=os.path.join(i, "tool-command"))) for i in _BIN_PATHS
    ]

    def _patch(self, root: str):
        tool_path = os.path.join(root, self.tool_path)
        original_exists = os.path.exists

        def _fake_exists(path):
            if path == tool_path:
                return True
            else:
                return original_exists(path)

        patcher = mock.patch("os.path.exists", side_effect=_fake_exists)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_get_tool_from_host_path(self):
        self._patch(os.path.sep)

        patcher = mock.patch(
            "shutil.which", return_value=os.path.join(os.path.sep, self.tool_path)
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        self.assertThat(
            file_utils.get_tool_path("tool-command"),
            Equals(os.path.join(os.path.sep, self.tool_path)),
        )

    def test_get_tool_from_snapcraft_snap_path(self):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        snap_root = os.path.join(os.path.sep, "snap", "snapcraft", "current")
        self._patch(snap_root)

        self.assertThat(
            file_utils.get_tool_path("tool-command"),
            Equals(os.path.join(snap_root, self.tool_path)),
        )

    def test_get_tool_from_docker_snap_path(self):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        snap_root = os.path.join(os.path.sep, "snap", "snapcraft", "current")
        self._patch(snap_root)

        with mock.patch(
            "snapcraft.internal.common.is_docker_instance", return_value=True
        ):
            self.assertThat(
                file_utils.get_tool_path("tool-command"),
                Equals(os.path.join(snap_root, self.tool_path)),
            )

    def test_get_tool_from_docker_deb_path(self):
        self._patch(os.path.sep)

        patcher = mock.patch(
            "shutil.which",
            return_value=os.path.join(
                os.path.sep, "bin", os.path.basename(self.tool_path)
            ),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        with mock.patch(
            "snapcraft.internal.common.is_docker_instance", return_value=True
        ):
            self.assertThat(
                file_utils.get_tool_path("tool-command"),
                Equals(
                    os.path.join(os.path.sep, "bin", os.path.basename(self.tool_path))
                ),
            )


class GetToolPathErrorsTest(testtools.TestCase):
    def test_get_tool_path_fails(self):
        self.assertRaises(
            file_utils.ToolMissingError,
            file_utils.get_tool_path,
            "non-existent-tool-command",
        )

    def test_get_tool_path_in_container_fails_root(self):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        self.assertRaises(
            file_utils.ToolMissingError,
            file_utils.get_tool_path,
            "non-existent-tool-command",
        )
