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
import shutil
from subprocess import CalledProcessError
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit
from tests.subprocess_utils import call, call_with_output


# LP: #1733584
class TestGit(unit.sources.SourceTestCase):  # type: ignore
    def setUp(self):

        super().setUp()
        patcher = mock.patch("snapcraft.sources.Git._get_source_details")
        self.mock_get_source_details = patcher.start()
        self.mock_get_source_details.return_value = ""
        self.addCleanup(patcher.stop)

    def test_pull(self):
        git = sources.Git("git://my-source", "source_dir")

        git.pull()

        self.mock_run.assert_called_once_with(
            ["git", "clone", "--recursive", "git://my-source", "source_dir"]
        )

    def test_pull_with_depth(self):
        git = sources.Git("git://my-source", "source_dir", source_depth=2)

        git.pull()

        self.mock_run.assert_called_once_with(
            [
                "git",
                "clone",
                "--recursive",
                "--depth",
                "2",
                "git://my-source",
                "source_dir",
            ]
        )

    def test_pull_branch(self):
        git = sources.Git("git://my-source", "source_dir", source_branch="my-branch")
        git.pull()

        self.mock_run.assert_called_once_with(
            [
                "git",
                "clone",
                "--recursive",
                "--branch",
                "my-branch",
                "git://my-source",
                "source_dir",
            ]
        )

    def test_pull_tag(self):
        git = sources.Git("git://my-source", "source_dir", source_tag="tag")
        git.pull()

        self.mock_run.assert_called_once_with(
            [
                "git",
                "clone",
                "--recursive",
                "--branch",
                "tag",
                "git://my-source",
                "source_dir",
            ]
        )

    def test_pull_commit(self):
        git = sources.Git(
            "git://my-source",
            "source_dir",
            source_commit="2514f9533ec9b45d07883e10a561b248497a8e3c",
        )
        git.pull()

        self.mock_run.assert_has_calls(
            [
                mock.call(
                    ["git", "clone", "--recursive", "git://my-source", "source_dir"]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "checkout",
                        "2514f9533ec9b45d07883e10a561b248497a8e3c",
                    ]
                ),
            ]
        )

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        git = sources.Git("git://my-source", "source_dir")
        git.pull()

        self.mock_run.assert_has_calls(
            [
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "fetch",
                        "--prune",
                        "--recurse-submodules=yes",
                    ]
                ),
                mock.call(
                    ["git", "-C", "source_dir", "reset", "--hard", "origin/master"]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "submodule",
                        "update",
                        "--recursive",
                        "--force",
                    ]
                ),
            ]
        )

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        git = sources.Git("git://my-source", "source_dir", source_tag="tag")
        git.pull()

        self.mock_run.assert_has_calls(
            [
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "fetch",
                        "--prune",
                        "--recurse-submodules=yes",
                    ]
                ),
                mock.call(
                    ["git", "-C", "source_dir", "reset", "--hard", "refs/tags/tag"]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "submodule",
                        "update",
                        "--recursive",
                        "--force",
                    ]
                ),
            ]
        )

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        git = sources.Git(
            "git://my-source",
            "source_dir",
            source_commit="2514f9533ec9b45d07883e10a561b248497a8e3c",
        )
        git.pull()

        self.mock_run.assert_has_calls(
            [
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "fetch",
                        "--prune",
                        "--recurse-submodules=yes",
                    ]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "reset",
                        "--hard",
                        "2514f9533ec9b45d07883e10a561b248497a8e3c",
                    ]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "submodule",
                        "update",
                        "--recursive",
                        "--force",
                    ]
                ),
            ]
        )

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        git = sources.Git("git://my-source", "source_dir", source_branch="my-branch")
        git.pull()

        self.mock_run.assert_has_calls(
            [
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "fetch",
                        "--prune",
                        "--recurse-submodules=yes",
                    ]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "reset",
                        "--hard",
                        "refs/heads/my-branch",
                    ]
                ),
                mock.call(
                    [
                        "git",
                        "-C",
                        "source_dir",
                        "submodule",
                        "update",
                        "--recursive",
                        "--force",
                    ]
                ),
            ]
        )

    def test_init_with_source_branch_and_tag_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Git,
            "git://mysource",
            "source_dir",
            source_tag="tag",
            source_branch="branch",
        )

        self.assertThat(raised.source_type, Equals("git"))
        self.assertThat(raised.options, Equals(["source-tag", "source-branch"]))

    def test_init_with_source_branch_and_commit_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Git,
            "git://mysource",
            "source_dir",
            source_commit="2514f9533ec9b45d07883e10a561b248497a8e3c",
            source_branch="branch",
        )

        self.assertThat(raised.source_type, Equals("git"))
        self.assertThat(raised.options, Equals(["source-branch", "source-commit"]))

    def test_init_with_source_tag_and_commit_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Git,
            "git://mysource",
            "source_dir",
            source_commit="2514f9533ec9b45d07883e10a561b248497a8e3c",
            source_tag="tag",
        )

        self.assertThat(raised.source_type, Equals("git"))
        self.assertThat(raised.options, Equals(["source-tag", "source-commit"]))

    def test_source_checksum_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Git,
            "git://mysource",
            "source_dir",
            source_checksum="md5/d9210476aac5f367b14e513bdefdee08",
        )

        self.assertThat(raised.source_type, Equals("git"))
        self.assertThat(raised.option, Equals("source-checksum"))

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["git"] is sources.Git)


class GitBaseTestCase(unit.TestCase):
    def rm_dir(self, dir):
        if os.path.exists(dir):
            shutil.rmtree(dir)

    def clean_dir(self, dir):
        self.rm_dir(dir)
        os.mkdir(dir)
        self.addCleanup(self.rm_dir, dir)

    def clone_repo(self, repo, tree):
        self.clean_dir(tree)
        call(["git", "clone", repo, tree])
        os.chdir(tree)
        call(["git", "config", "--local", "user.name", '"Example Dev"'])
        call(["git", "config", "--local", "user.email", "dev@example.com"])

    def add_file(self, filename, body, message):
        with open(filename, "w") as fp:
            fp.write(body)

        call(["git", "add", filename])
        call(["git", "commit", "-am", message])

    def check_file_contents(self, path, expected):
        body = None
        with open(path) as fp:
            body = fp.read()
        self.assertThat(body, Equals(expected))


class TestGitConflicts(GitBaseTestCase):
    """Test that git pull errors don't kill the parser"""

    def test_git_conflicts(self):

        repo = "/tmp/conflict-test.git"
        working_tree = "/tmp/git-conflict-test"
        conflicting_tree = "{}-conflict".format(working_tree)
        git = sources.Git(repo, working_tree, silent=True)

        self.clean_dir(repo)
        self.clean_dir(working_tree)
        self.clean_dir(conflicting_tree)

        os.chdir(repo)
        call(["git", "init", "--bare"])

        self.clone_repo(repo, working_tree)

        # check out the original repo
        self.clone_repo(repo, conflicting_tree)

        # add a file to the repo
        os.chdir(working_tree)
        self.add_file("fake", "fake 1", "fake 1")
        call(["git", "push", repo])

        git.pull()

        os.chdir(conflicting_tree)
        self.add_file("fake", "fake 2", "fake 2")
        call(["git", "push", "-f", repo])

        os.chdir(working_tree)
        git.pull()

        body = None
        with open(os.path.join(working_tree, "fake")) as fp:
            body = fp.read()

        self.assertThat(body, Equals("fake 2"))

    def test_git_submodules(self):
        """Test that updates to submodules are pulled"""
        repo = "/tmp/submodules.git"
        sub_repo = "/tmp/subrepo"
        working_tree = "/tmp/git-submodules"
        working_tree_two = "{}-two".format(working_tree)
        sub_working_tree = "/tmp/git-submodules-sub"
        git = sources.Git(repo, working_tree, silent=True)

        self.clean_dir(repo)
        self.clean_dir(sub_repo)
        self.clean_dir(working_tree)
        self.clean_dir(working_tree_two)
        self.clean_dir(sub_working_tree)

        os.chdir(sub_repo)
        call(["git", "init", "--bare"])

        self.clone_repo(sub_repo, sub_working_tree)
        self.add_file("sub-file", "sub-file", "sub-file")
        call(["git", "push", sub_repo])

        os.chdir(repo)
        call(["git", "init", "--bare"])

        self.clone_repo(repo, working_tree)
        call(["git", "submodule", "add", sub_repo])
        call(["git", "commit", "-am", "added submodule"])
        call(["git", "push", repo])

        git.pull()

        self.check_file_contents(
            os.path.join(working_tree, "subrepo", "sub-file"), "sub-file"
        )

        # add a file to the repo
        os.chdir(sub_working_tree)
        self.add_file("fake", "fake 1", "fake 1")
        call(["git", "push", sub_repo])

        os.chdir(working_tree)
        git.pull()

        # this shouldn't cause any change
        self.check_file_contents(
            os.path.join(working_tree, "subrepo", "sub-file"), "sub-file"
        )
        self.assertFalse(os.path.exists(os.path.join(working_tree, "subrepo", "fake")))

        # update the submodule
        self.clone_repo(repo, working_tree_two)
        call(["git", "submodule", "update", "--init", "--recursive", "--remote"])
        call(["git", "add", "subrepo"])
        call(["git", "commit", "-am", "updated submodule"])
        call(["git", "push"])

        os.chdir(working_tree)
        git.pull()

        # new file should be there now
        self.check_file_contents(
            os.path.join(working_tree, "subrepo", "sub-file"), "sub-file"
        )
        self.check_file_contents(
            os.path.join(working_tree, "subrepo", "fake"), "fake 1"
        )


class GitDetailsTestCase(GitBaseTestCase):
    def setUp(self):
        def _add_and_commit_file(filename, content=None, message=None):
            if not content:
                content = filename
            if not message:
                message = filename

            with open(filename, "w") as fp:
                fp.write(content)
            call(["git", "add", filename])
            call(["git", "commit", "-am", message])

        super().setUp()
        self.working_tree = "git-test"
        self.source_dir = "git-checkout"
        self.clean_dir(self.working_tree)
        os.chdir(self.working_tree)
        call(["git", "init"])
        call(["git", "config", "user.name", '"Example Dev"'])
        call(["git", "config", "user.email", "dev@example.com"])
        _add_and_commit_file("testing")
        self.expected_commit = call_with_output(["git", "rev-parse", "HEAD"])

        _add_and_commit_file("testing-2")
        call(["git", "tag", "test-tag"])
        self.expected_tag = "test-tag"

        _add_and_commit_file("testing-3")
        self.expected_branch = "test-branch"
        call(["git", "branch", self.expected_branch])

        os.chdir("..")

        self.git = sources.Git(
            self.working_tree,
            self.source_dir,
            silent=True,
            source_commit=self.expected_commit,
        )
        self.git.pull()

        self.source_details = self.git._get_source_details()

    def test_git_details_commit(self):
        self.assertThat(
            self.source_details["source-commit"], Equals(self.expected_commit)
        )

    def test_git_details_branch(self):
        shutil.rmtree(self.source_dir)
        self.git = sources.Git(
            self.working_tree,
            self.source_dir,
            silent=True,
            source_branch=self.expected_branch,
        )
        self.git.pull()

        self.source_details = self.git._get_source_details()
        self.assertThat(
            self.source_details["source-branch"], Equals(self.expected_branch)
        )

    def test_git_details_tag(self):
        self.git = sources.Git(
            self.working_tree, self.source_dir, silent=True, source_tag="test-tag"
        )
        self.git.pull()

        self.source_details = self.git._get_source_details()
        self.assertThat(self.source_details["source-tag"], Equals(self.expected_tag))


class GitGenerateVersionBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("subprocess.check_output")
        self.output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.Popen")
        self.popen_mock = patcher.start()
        self.addCleanup(patcher.stop)


class GitGenerateVersionTestCase(GitGenerateVersionBaseTestCase):

    scenarios = (
        ("only_tag", dict(return_value="2.28", expected="2.28")),
        (
            "tag+commits",
            dict(return_value="2.28-28-gabcdef1", expected="2.28+git28.abcdef1"),
        ),
        (
            "tag+dirty",
            dict(
                return_value="2.28-29-gabcdef1-dirty",
                expected="2.28+git29.abcdef1-dirty",
            ),
        ),
    )

    def test_version(self):
        self.output_mock.return_value = self.return_value.encode("utf-8")
        self.assertThat(sources.Git.generate_version(), Equals(self.expected))


class GitGenerateVersionNoTagTestCase(GitGenerateVersionBaseTestCase):
    def test_version(self):
        self.output_mock.side_effect = CalledProcessError(1, [])
        proc_mock = mock.Mock()
        proc_mock.returncode = 0
        proc_mock.communicate.return_value = (b"abcdef1", b"")
        self.popen_mock.return_value = proc_mock

        expected = "0+git.abcdef1"
        self.assertThat(sources.Git.generate_version(), Equals(expected))


class GitGenerateVersionNoGitTestCase(GitGenerateVersionBaseTestCase):
    def test_version(self):
        self.output_mock.side_effect = CalledProcessError(1, [])
        proc_mock = mock.Mock()
        proc_mock.returncode = 2
        proc_mock.communicate.return_value = (b"", b"No .git")
        self.popen_mock.return_value = proc_mock

        self.assertRaises(sources.errors.VCSError, sources.Git.generate_version)
