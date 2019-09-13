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
import subprocess
from unittest import mock

import fixtures
from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit


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

    def test_pull_failure(self):
        self.mock_run.side_effect = subprocess.CalledProcessError(1, [])

        hg = sources.Mercurial("hg://my-source", "source_dir")
        raised = self.assertRaises(sources.errors.SnapcraftPullError, hg.pull)
        self.assertThat(raised.command, Equals("hg clone hg://my-source source_dir"))
        self.assertThat(raised.exit_code, Equals(1))


def get_side_effect(original_call):
    def side_effect(cmd, *args, **kwargs):
        if len(cmd) > 1 and cmd[1] == "id":
            return "mock-commit".encode()
        elif cmd[0] == "hg":
            return
        return original_call(cmd, *args, **kwargs)

    return side_effect


class MercurialDetailsTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.working_tree = "hg-test"
        self.source_dir = "hg-checkout"

        os.mkdir(self.source_dir)
        # Simulate that we have already cloned the code.
        os.mkdir(os.path.join(self.source_dir, ".hg"))

        self.fake_check_output = self.useFixture(
            fixtures.MockPatch(
                "subprocess.check_output",
                side_effect=get_side_effect(subprocess.check_output),
            )
        )
        self.fake_check_call = self.useFixture(
            fixtures.MockPatch(
                "subprocess.check_call",
                side_effect=get_side_effect(subprocess.check_call),
            )
        )

    def test_hg_details_commit(self):
        hg = sources.Mercurial(self.working_tree, self.source_dir, silent=True)
        hg.pull()

        source_details = hg._get_source_details()
        self.assertThat(source_details["source-commit"], Equals("mock-commit"))

        self.fake_check_output.mock.assert_has_calls(
            [
                mock.call(["hg", "id", self.source_dir]),
                mock.call(["hg", "id", self.source_dir]),
            ]
        )
        self.fake_check_call.mock.assert_called_once_with(
            ["hg", "pull", self.working_tree], stderr=-3, stdout=-3
        )

    def test_hg_details_branch(self):
        hg = sources.Mercurial(
            self.working_tree, self.source_dir, silent=True, source_branch="test-branch"
        )
        hg.pull()

        source_details = hg._get_source_details()
        self.assertThat(source_details["source-branch"], Equals("test-branch"))

        self.fake_check_output.mock.assert_not_called()
        self.fake_check_call.mock.assert_called_once_with(
            ["hg", "pull", "-b", "test-branch", self.working_tree], stderr=-3, stdout=-3
        )

    def test_hg_details_tag(self):
        hg = sources.Mercurial(
            self.working_tree, self.source_dir, silent=True, source_tag="test-tag"
        )
        hg.pull()

        source_details = hg._get_source_details()
        self.assertThat(source_details["source-tag"], Equals("test-tag"))

        self.fake_check_output.mock.assert_not_called()
        self.fake_check_call.mock.assert_called_once_with(
            ["hg", "pull", "-r", "test-tag", self.working_tree], stderr=-3, stdout=-3
        )
