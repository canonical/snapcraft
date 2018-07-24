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

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import fixture_setup, unit


# LP: #1733584
class TestBazaar(unit.sources.SourceTestCase):  # type: ignore
    def setUp(self):
        super().setUp()

        # Mock _get_source_details() since not all tests have a
        # full repo checkout
        patcher = mock.patch("snapcraft.sources.Bazaar._get_source_details")
        self.mock_get_source_details = patcher.start()
        self.mock_get_source_details.return_value = ""
        self.addCleanup(patcher.stop)

    def test_pull(self):
        bzr = sources.Bazaar("lp:my-source", "source_dir")

        bzr.pull()

        self.mock_rmdir.assert_called_once_with("source_dir")
        self.mock_run.assert_called_once_with(
            ["bzr", "branch", "lp:my-source", "source_dir"]
        )

    def test_pull_tag(self):
        bzr = sources.Bazaar("lp:my-source", "source_dir", source_tag="tag")
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ["bzr", "branch", "-r", "tag:tag", "lp:my-source", "source_dir"]
        )

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        bzr = sources.Bazaar("lp:my-source", "source_dir", source_tag="tag")
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ["bzr", "pull", "-r", "tag:tag", "lp:my-source", "-d", "source_dir"]
        )

    def test_pull_commit(self):
        bzr = sources.Bazaar("lp:my-source", "source_dir", source_commit="2")
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ["bzr", "branch", "-r", "2", "lp:my-source", "source_dir"]
        )

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        bzr = sources.Bazaar("lp:my-source", "source_dir", source_commit="2")
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ["bzr", "pull", "-r", "2", "lp:my-source", "-d", "source_dir"]
        )

    def test_init_with_source_branch_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Bazaar,
            "lp:mysource",
            "source_dir",
            source_branch="branch",
        )

        self.assertThat(raised.source_type, Equals("bzr"))
        self.assertThat(raised.option, Equals("source-branch"))

    def test_init_with_source_depth_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Bazaar,
            "lp://mysource",
            "source_dir",
            source_depth=2,
        )

        self.assertThat(raised.source_type, Equals("bzr"))
        self.assertThat(raised.option, Equals("source-depth"))

    def test_init_with_source_tag_and_commit_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            sources.Bazaar,
            "lp://mysource",
            "source_dir",
            source_tag="tag",
            source_commit="2",
        )

        self.assertThat(raised.source_type, Equals("bzr"))
        self.assertThat(raised.options, Equals(["source-tag", "source-commit"]))

    def test_source_checksum_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Bazaar,
            "lp://mysource",
            "source_dir",
            source_checksum="md5/d9210476aac5f367b14e513bdefdee08",
        )

        self.assertThat(raised.source_type, Equals("bzr"))
        self.assertThat(raised.option, Equals("source-checksum"))

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["bzr"] is sources.Bazaar)


class BazaarDetailsTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.working_tree = "bzr-test"
        self.bzr_repo = fixture_setup.BzrRepo(self.working_tree)
        self.useFixture(self.bzr_repo)
        self.source_dir = "bzr-checkout"
        os.mkdir(self.source_dir)

        self.bzr = sources.Bazaar(self.working_tree, self.source_dir, silent=True)
        self.bzr.pull()

        self.source_details = self.bzr._get_source_details()

    def test_bzr_details_commit(self):
        self.assertThat(
            self.source_details["source-commit"], Equals(self.bzr_repo.commit)
        )

    def test_bzr_details_tag(self):
        self.bzr = sources.Bazaar(
            self.working_tree, self.source_dir, source_tag="feature-tag", silent=True
        )
        self.bzr.pull()

        self.source_details = self.bzr._get_source_details()
        self.assertThat(self.source_details["source-tag"], Equals("feature-tag"))
