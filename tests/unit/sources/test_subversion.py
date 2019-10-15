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
class TestSubversion(unit.sources.SourceTestCase):  # type: ignore
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.sources.Subversion._get_source_details")
        self.mock_get_source_details = patcher.start()
        self.mock_get_source_details.return_value = ""
        self.addCleanup(patcher.stop)

    def test_pull_remote(self):
        svn = sources.Subversion("svn://my-source", "source_dir")
        svn.pull()
        self.mock_run.assert_called_once_with(
            ["svn", "checkout", "svn://my-source", "source_dir"]
        )

    def test_pull_remote_commit(self):
        svn = sources.Subversion("svn://my-source", "source_dir", source_commit="2")
        svn.pull()
        self.mock_run.assert_called_once_with(
            ["svn", "checkout", "svn://my-source", "source_dir", "-r", "2"]
        )

    def test_pull_local_absolute_path(self):
        svn = sources.Subversion(self.path, "source_dir")
        svn.pull()
        self.mock_run.assert_called_once_with(
            ["svn", "checkout", "file://" + self.path, "source_dir"]
        )

    def test_pull_local_relative_path(self):
        os.mkdir("my-source")
        svn = sources.Subversion("my-source", "source_dir")
        svn.pull()
        self.mock_run.assert_called_once_with(
            [
                "svn",
                "checkout",
                "file://{}".format(os.path.join(self.path, "my-source")),
                "source_dir",
            ]
        )

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True
        svn = sources.Subversion("svn://my-source", "source_dir")
        svn.pull()
        self.mock_run.assert_called_once_with(["svn", "update"], cwd=svn.source_dir)

    def test_init_with_source_tag_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Subversion,
            "svn://mysource",
            "source_dir",
            source_tag="tag",
        )
        self.assertThat(raised.source_type, Equals("svn"))
        self.assertThat(raised.option, Equals("source-tag"))

    def test_init_with_source_branch_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Subversion,
            "svn://mysource",
            "source_dir",
            source_branch="branch",
        )
        self.assertThat(raised.source_type, Equals("svn"))
        self.assertThat(raised.option, Equals("source-branch"))

    def test_init_with_source_depth_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Subversion,
            "svn://mysource",
            "source_dir",
            source_depth=2,
        )

        self.assertThat(raised.source_type, Equals("svn"))
        self.assertThat(raised.option, Equals("source-depth"))

    def test_source_checksum_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            sources.Subversion,
            "svn://mysource",
            "source_dir",
            source_checksum="md5/d9210476aac5f367b14e513bdefdee08",
        )

        self.assertThat(raised.source_type, Equals("svn"))
        self.assertThat(raised.option, Equals("source-checksum"))

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["subversion"] is sources.Subversion)

    def test_pull_failure(self):
        self.mock_run.side_effect = subprocess.CalledProcessError(1, [])

        svn = sources.Subversion("svn://my-source", "source_dir")
        raised = self.assertRaises(sources.errors.SnapcraftPullError, svn.pull)
        self.assertThat(
            raised.command, Equals("svn checkout svn://my-source source_dir")
        )
        self.assertThat(raised.exit_code, Equals(1))


def get_side_effect(original_call):
    def side_effect(cmd, *args, **kwargs):
        if len(cmd) > 1 and cmd[1] == "info":
            return "mock-commit".encode()
        elif cmd[0] == "svn":
            return
        return original_call(cmd, *args, **kwargs)

    return side_effect


class SubversionDetailsTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.repo_tree = "svn-repo"
        self.working_tree = "svn-test"
        self.source_dir = "svn-checkout"

        self.expected_commit = "1"

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

    def test_svn_details_commit(self):
        svn = sources.Subversion(self.repo_tree, self.source_dir, silent=True)
        svn.pull()

        source_details = svn._get_source_details()
        self.assertThat(source_details["source-commit"], Equals("mock-commit"))

        self.fake_check_call.mock.assert_called_once_with(
            ["svn", "checkout", "svn-repo", "svn-checkout"], stderr=-3, stdout=-3
        )
