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
import sys

from unittest import mock
from testtools.matchers import FileExists

from snapcraft.internal import sources
from tests import unit
from tests.file_utils import get_snapcraft_path


class TestRpm(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.rpm_file_path = os.path.join(
            get_snapcraft_path(),
            "tests",
            "integration",
            "snaps",
            "rpm-hello",
            "small-0.1-1.noarch.rpm",
        )

        self.dest_dir = "dst"
        os.makedirs(self.dest_dir)

    def test_pull_rpm_file_must_extract(self):
        rpm_source = sources.Rpm(self.rpm_file_path, self.dest_dir)
        rpm_source.pull()

        self.assertThat(os.path.join(self.dest_dir, "bin", "hello"), FileExists())

    @mock.patch.object(sources.Rpm, "provision")
    def test_pull_rpm_must_not_clean_targets(self, mock_provision):
        rpm_source = sources.Rpm(self.rpm_file_path, self.dest_dir)
        rpm_source.pull()

        mock_provision.assert_called_once_with(
            self.dest_dir,
            clean_target=False,
            src=os.path.join(self.dest_dir, "small-0.1-1.noarch.rpm"),
        )

    def test_has_source_handler_entry_on_linux(self):
        if sys.platform == "linux":
            self.assertTrue(sources._source_handler["rpm"] is sources.Rpm)
        else:
            self.assertRaises(KeyError, sources._source_handler["rpm"])
