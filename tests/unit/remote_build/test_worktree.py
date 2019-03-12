# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import git

from snapcraft.internal.remote_build import Worktree
from testtools.matchers import Equals, NotEquals
from tests import unit
from . import TestDir


class WorktreeTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self._source = self.useFixture(TestDir())
        self._source.create_file("foo")
        self._source.create_file("bar")
        self._source.create_dir("dir")
        self._source.create_file("dir", "baz")
        self._source.create_dir("empty_dir")

        self._dest = self.useFixture(TestDir())
        self._wt = Worktree(
            self._source.path,
            self._dest.path,
            ignore=["test_*.snap", "buildlog_*.txt*", "parts", "stage", "prime"],
        )
        self._wt.sync()

    def test_worktree_creation(self):
        self.assertTrue(self._dest.exists("foo"))
        self.assertTrue(self._dest.exists("bar"))
        self.assertTrue(self._dest.exists("dir", "baz"))
        self.assertTrue(self._dest.exists("empty_dir"))

    def test_worktree_new_file(self):
        self._source.create_file("new_file")
        self.assertFalse(self._dest.exists("new_file"))
        self._wt.sync()
        self.assertTrue(self._dest.exists("new_file"))

    def test_worktree_remove_file(self):
        self.assertTrue(self._dest.exists("foo"))
        self._source.unlink("foo")
        self._wt.sync()
        self.assertFalse(self._dest.exists("foo"))

    def test_worktree_remove_empty_dir(self):
        self.assertTrue(self._dest.exists("empty_dir"))
        self._source.rmdir("empty_dir")
        self._wt.sync()
        self.assertFalse(self._dest.exists("empty_dir"))

    def test_worktree_remove_dir(self):
        self.assertTrue(self._dest.exists("dir"))
        self._source.unlink("dir", "baz")
        self._source.rmdir("dir")
        self._wt.sync()
        self.assertFalse(self._dest.exists("dir"))

    def test_git_repository_creation(self):
        self.assertTrue(self._dest.exists(".git"))
        repo = git.Repo(self._dest.path)
        self.assertThat(len(repo.head.object.hexsha), Equals(40))

    def test_worktree_ignored_files(self):
        for name in [
            ".gitignore",
            "test_0.1_amd64.snap",
            "other_0.1_amd64.snap",
            "buildlog_i386.txt.gz",
            "buildlog_amd64.txt",
        ]:
            self._source.create_file(name)
        for dirname in [".svn", ".bzr", "parts", "stage", "prime"]:
            self._source.create_dir(dirname)
        self._wt.sync()
        self.assertTrue(self._dest.exists("other_0.1_amd64.snap"))
        for name in [
            ".gitignore",
            "test_0.1_amd64.snap",
            "buildlog_i386.txt.gz",
            "buildlog_amd64.txt",
            ".svn",
            ".bzr",
            "parts",
            "stage",
            "prime",
        ]:
            self.assertFalse(self._dest.exists(name))

    def test_sync_commit_add_file(self):
        repo = git.Repo(self._dest.path)
        sha = repo.head.object.hexsha
        self._wt.sync()
        self.assertThat(repo.head.object.hexsha, Equals(sha))
        self._source.create_file("new_file")
        self._wt.sync()
        self.assertThat(repo.head.object.hexsha, NotEquals(sha))

    def test_sync_commit_remove_file(self):
        repo = git.Repo(self._dest.path)
        sha = repo.head.object.hexsha
        self._wt.sync()
        self.assertThat(repo.head.object.hexsha, Equals(sha))
        self._source.unlink("foo")
        self._wt.sync()
        self.assertThat(repo.head.object.hexsha, NotEquals(sha))
