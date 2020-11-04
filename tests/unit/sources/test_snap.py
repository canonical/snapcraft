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
import sys
from unittest import mock

from testtools.matchers import DirExists, Equals, FileExists, MatchesRegex

from snapcraft.internal import sources
from tests import unit


class TestSnap(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.test_file_path = os.path.join(
            os.path.dirname(unit.__file__), "..", "data", "test-snap.snap"
        )

        self.dest_dir = "dst"
        os.makedirs(self.dest_dir)

    def test_pull_snap_file_must_extract(self):
        snap_source = sources.Snap(self.test_file_path, self.dest_dir)
        snap_source.pull()

        self.assertThat(os.path.join(self.dest_dir, "meta.basic"), DirExists())
        self.assertThat(
            os.path.join(self.dest_dir, "meta.basic", "snap.yaml"), FileExists()
        )

    @mock.patch.object(sources.Snap, "provision")
    def test_pull_snap_must_not_clean_targets(self, mock_provision):
        snap_source = sources.Snap(self.test_file_path, self.dest_dir)
        snap_source.pull()

        mock_provision.assert_called_once_with(
            self.dest_dir,
            clean_target=False,
            src=os.path.join(self.dest_dir, "test-snap.snap"),
        )

    def test_has_source_handler_entry_on_linux(self):
        if sys.platform == "linux":
            self.assertTrue(sources._source_handler["snap"] is sources.Snap)
        else:
            self.assertRaises(KeyError, sources._source_handler["snap"])

    @mock.patch(
        "subprocess.check_output", side_effect=subprocess.CalledProcessError(1, [])
    )
    def test_pull_failure_bad_unsquash(self, mock_run):
        snap_source = sources.Snap(self.test_file_path, self.dest_dir)
        raised = self.assertRaises(sources.errors.SnapcraftPullError, snap_source.pull)
        self.assertThat(
            raised.command,
            MatchesRegex(
                r"unsquashfs -force -dest {0}/\w+ {0}/dst/test-snap.snap".format(
                    self.path
                )
            ),
        )
        self.assertThat(raised.exit_code, Equals(1))


class TestGetName(unit.TestCase):
    def test_get_name(self):
        os.mkdir("meta")
        with open(os.path.join("meta", "snap.yaml"), "w") as snap_yaml_file:
            print("name: my-snap", file=snap_yaml_file)
        self.assertThat(sources._snap._get_snap_name("."), Equals("my-snap"))

    def test_no_name_yaml(self):
        os.mkdir("meta")
        with open(os.path.join("meta", "snap.yaml"), "w") as snap_yaml_file:
            print("summary: no name", file=snap_yaml_file)
        self.assertRaises(
            sources.errors.InvalidSnapError, sources._snap._get_snap_name, "."
        )

    def test_no_snap_yaml(self):
        os.mkdir("meta")
        self.assertRaises(
            sources.errors.InvalidSnapError, sources._snap._get_snap_name, "."
        )
