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
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit
from tests.subprocess_utils import call, call_with_output


# LP: #1733584
class TestMercurial(unit.sources.SourceTestCase):  # type: ignore
    def setUp(self):
        super().setUp()
        patcher = mock.patch("snapcraft.sources.Mercurial._get_source_details")
        self.mock_get_source_details = patcher.start()
        self.mock_get_source_details.return_value = ""
        self.addCleanup(patcher.stop)

    def test_pull(self):
        hg = sources.Mercurial("hg://my-source", "source_dir")
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "clone", "hg://my-source", "source_dir"]
        )

    def test_pull_branch(self):
        hg = sources.Mercurial(
            "hg://my-source", "source_dir", source_branch="my-branch"
        )
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "clone", "-u", "my-branch", "hg://my-source", "source_dir"]
        )

    def test_pull_tag(self):
        hg = sources.Mercurial("hg://my-source", "source_dir", source_tag="tag")
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "clone", "-u", "tag", "hg://my-source", "source_dir"]
        )

    def test_pull_commit(self):
        hg = sources.Mercurial("hg://my-source", "source_dir", source_commit="2")
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "clone", "-u", "2", "hg://my-source", "source_dir"]
        )

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial("hg://my-source", "source_dir")
        hg.pull()

        self.mock_run.assert_called_once_with(["hg", "pull", "hg://my-source"])

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial("hg://my-source", "source_dir", source_tag="tag")
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "pull", "-r", "tag", "hg://my-source"]
        )

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial("hg://my-source", "source_dir", source_commit="2")
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "pull", "-r", "2", "hg://my-source"]
        )

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial(
            "hg://my-source", "source_dir", source_branch="my-branch"
        )
        hg.pull()

        self.mock_run.assert_called_once_with(
            ["hg", "pull", "-b", "my-branch", "hg://my-source"]
        )

    def test_init_with_source_branch_and_tag_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Mercurial,
            "hg://mysource",
            "source_dir",
            source_tag="tag",
            source_branch="branch",
        )

        self.assertThat(raised.source_type, Equals("mercurial"))
        self.assertThat(raised.options, Equals(["source-tag", "source-branch"]))

    def test_init_with_source_commit_and_tag_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Mercurial,
            "hg://mysource",
            "source_dir",
            source_commit="2",
            source_tag="tag",
        )

        self.assertThat(raised.source_type, Equals("mercurial"))
        self.assertThat(raised.options, Equals(["source-tag", "source-commit"]))

    def test_init_with_source_commit_and_branch_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Mercurial,
            "hg://mysource",
            "source_dir",
            source_commit="2",
            source_branch="branch",
        )

        self.assertThat(raised.source_type, Equals("mercurial"))
        self.assertThat(raised.options, Equals(["source-branch", "source-commit"]))

    def test_init_with_source_depth_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Mercurial,
            "hg://mysource",
            "source_dir",
            source_depth=2,
        )

        self.assertThat(raised.source_type, Equals("mercurial"))
        self.assertThat(raised.option, Equals("source-depth"))

    def test_source_checksum_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Mercurial,
            "hg://mysource",
            "source_dir",
            source_checksum="md5/d9210476aac5f367b14e513bdefdee08",
        )

        self.assertThat(raised.source_type, Equals("mercurial"))
        self.assertThat(raised.option, Equals("source-checksum"))

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["mercurial"] is sources.Mercurial)


class MercurialBaseTestCase(unit.TestCase):
    def rm_dir(self, dir):
        if os.path.exists(dir):
            shutil.rmtree(dir)

    def clean_dir(self, dir):
        self.rm_dir(dir)
        os.mkdir(dir)
        self.addCleanup(self.rm_dir, dir)

    def clone_repo(self, repo, tree):
        self.clean_dir(tree)
        call(["hg", "clone", repo, tree])
        os.chdir(tree)

    def add_file(self, filename, body, message):
        with open(filename, "w") as fp:
            fp.write(body)

        call(["hg", "add", filename])
        call(["hg", "commit", "-am", message])

    def check_file_contents(self, path, expected):
        body = None
        with open(path) as fp:
            body = fp.read()
        self.assertThat(body, Equals(expected))


class MercurialDetailsTestCase(MercurialBaseTestCase):
    def setUp(self):
        super().setUp()
        self.working_tree = "hg-test"
        self.source_dir = "hg-checkout"
        self.clean_dir(self.working_tree)
        self.clean_dir(self.source_dir)
        os.chdir(self.working_tree)
        call(["hg", "init"])
        with open("testing", "w") as fp:
            fp.write("testing")
        call(["hg", "add", "testing"])
        call(["hg", "commit", "-m", "testing", "-u", "Test User <t@example.com>"])
        call(["hg", "tag", "-u", "test", "test-tag"])
        self.expected_commit = call_with_output(["hg", "id"]).split()[0]
        self.expected_branch = call_with_output(["hg", "branch"])
        self.expected_tag = "test-tag"

        os.chdir("..")

        self.hg = sources.Mercurial(self.working_tree, self.source_dir, silent=True)
        self.hg.pull()

        self.source_details = self.hg._get_source_details()

    def test_hg_details_commit(self):
        self.assertThat(
            self.source_details["source-commit"], Equals(self.expected_commit)
        )

    def test_hg_details_branch(self):
        self.clean_dir(self.source_dir)
        self.hg = sources.Mercurial(
            self.working_tree, self.source_dir, silent=True, source_branch="default"
        )
        self.hg.pull()

        self.source_details = self.hg._get_source_details()
        self.assertThat(
            self.source_details["source-branch"], Equals(self.expected_branch)
        )

    def test_hg_details_tag(self):
        self.clean_dir(self.source_dir)
        self.hg = sources.Mercurial(
            self.working_tree, self.source_dir, silent=True, source_tag="test-tag"
        )
        self.hg.pull()

        self.source_details = self.hg._get_source_details()
        self.assertThat(self.source_details["source-tag"], Equals(self.expected_tag))
