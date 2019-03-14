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

import os

from snapcraft.internal.remote_build import Repo, errors
from subprocess import run, check_output, DEVNULL
from testtools.matchers import Contains, Equals, FileExists, Not
from tests import unit
from unittest import mock
from . import Chdir, TestDir


class RepoTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self._dir = self.useFixture(TestDir())
        self._dir.create_file("foo")
        self._dir.create_file("bar")
        self._dir.create_dir("dir")
        self._dir.create_file("dir", "baz")
        self._dir.create_dir("empty_dir")

    def _run(self, *cmd) -> int:
        with Chdir(self._dir.path):
            res = run(cmd, stdout=DEVNULL, stderr=DEVNULL)
        return res.returncode

    def _check_output(self, *cmd) -> bytes:
        with Chdir(self._dir.path):
            res = check_output(cmd)
        return res

    def test_repo_creation(self):
        Repo(self._dir.path)
        res = self._check_output("git", "status", "-b")
        self.assertThat(str(res), Contains("branch master"))

    def test_repo_add(self):
        repo = Repo(self._dir.path)
        res = self._run("git", "ls-files", "--error-unmatch", "foo")
        self.assertThat(res, Equals(1))
        repo.add("foo")
        res = self._run("git", "ls-files", "--error-unmatch", "foo")
        self.assertThat(res, Equals(0))

    def test_repo_remove(self):
        repo = Repo(self._dir.path)
        repo.add("foo")
        repo.commit()
        res = self._run("git", "ls-files", "--error-unmatch", "foo")
        self.assertThat(res, Equals(0))
        repo.remove("foo")
        res = self._run("git", "ls-files", "--error-unmatch", "foo")
        self.assertThat(res, Equals(1))

    def test_repo_commit(self):
        repo = Repo(self._dir.path)
        repo.add("foo")
        res = self._check_output("git", "status", "--short", "foo")
        self.assertThat(str(res), Contains("A  foo"))
        repo.commit()
        res = self._check_output("git", "status", "--short", "foo")
        self.assertThat(res, Equals(b""))

    def test_repo_add_remote_error(self):
        repo = Repo(self._dir.path)
        raised = self.assertRaises(
            errors.RemoteBuilderNotSupportedError, repo.add_remote, "foo", "user", "id"
        )
        self.assertThat(str(raised), Contains("is not supported"))

    def test_repo_add_remote(self):
        repo = Repo(self._dir.path)
        url = repo.add_remote("launchpad", "user", "id")
        self.assertThat(url, Equals("git+ssh://user@git.launchpad.net/~user/+git/id/"))

        res = self._check_output("git", "remote", "-v")
        self.assertThat(
            str(res),
            Contains(
                "launchpad\\tgit+ssh://user@git.launchpad.net/~user/+git/id/ (push)"
            ),
        )

    def test_repo_push_remote_error(self):
        repo = Repo(self._dir.path)
        raised = self.assertRaises(
            errors.RemoteBuilderNotSupportedError,
            repo.push_remote,
            "foo",
            "user",
            "branch",
            "id",
        )
        self.assertThat(str(raised), Contains("is not supported"))

    @mock.patch("git.Repo")
    def test_push_remote(self, mock_repo):
        repo = Repo(self._dir.path)
        repo.push_remote("launchpad", "user", "branch", "id")
        mock_repo.assert_has_calls(
            [
                mock.call(self._dir.path),
                mock.call().git.push(
                    "git+ssh://user@git.launchpad.net/~user/+git/id/",
                    "branch",
                    force=True,
                ),
                mock.call().git.push(
                    "git+ssh://user@git.launchpad.net/~user/+git/id/", "--tags"
                ),
            ]
        )

    @mock.patch("git.Repo")
    def test_push(self, mock_repo):
        repo = Repo(self._dir.path)
        repo.add_remote("launchpad", "user", "id")
        repo.push()
        mock_repo.assert_has_calls(
            [
                mock.call(self._dir.path),
                mock.call().create_remote(
                    "launchpad", url="git+ssh://user@git.launchpad.net/~user/+git/id/"
                ),
                mock.call().create_remote().push(refspec="master:master"),
            ],
            any_order=True,
        )

    def test_reset(self):
        repo = Repo(self._dir.path)
        repo.add("foo")
        repo.commit()
        path = os.path.join(self._dir.path, "foo")
        self.assertThat(path, FileExists())
        self._dir.unlink("foo")
        self.assertThat(path, Not(FileExists()))
        repo.reset()
        self.assertThat(path, FileExists())

    def test_is_dirty(self):
        repo = Repo(self._dir.path)
        repo.add("foo")
        repo.commit()
        self.assertFalse(repo.is_dirty)
        print("hello", file=open(os.path.join(self._dir.path, "foo"), "a"))
        self.assertTrue(repo.is_dirty)

    def test_uncommited_files(self):
        repo = Repo(self._dir.path)
        repo.add("foo")
        repo.commit()
        self.assertThat(repo.uncommitted_files, Equals([]))
        print("hello", file=open(os.path.join(self._dir.path, "foo"), "a"))
        self.assertThat(repo.uncommitted_files, Equals(["foo"]))
