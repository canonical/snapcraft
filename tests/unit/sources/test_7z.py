# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Tim Süberkrüb
# Copyright (C) 2017-2019 Canonical Ltd
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
import subprocess

import fixtures
from testtools.matchers import Equals, MatchesRegex

from snapcraft.internal import sources
from tests import unit


def get_side_effect(original_call):
    def side_effect(cmd, *args, **kwargs):
        if cmd[0] == "7z":
            return "".encode()
        return original_call(cmd, *args, **kwargs)

    return side_effect


class Test7z(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.source_dir = "7z-source-dir"
        self.test_7z_file_path = "fake-7z-file.7z"

        os.makedirs(self.source_dir)
        open(self.test_7z_file_path, "w").close()

        self.fake_check_output = self.useFixture(
            fixtures.MockPatch(
                "subprocess.check_output",
                side_effect=get_side_effect(subprocess.check_output),
            )
        )

    def test_pull_7z_file_must_extract(self):
        seven_zip_source = sources.SevenZip(self.test_7z_file_path, self.source_dir)
        seven_zip_source.pull()

    def test_extract_and_keep_7zfile(self):
        seven_zip_source = sources.SevenZip(self.test_7z_file_path, self.source_dir)
        # This is the first step done by pull. We don't call pull to call the
        # second step with a different keep_7z value.
        shutil.copy2(seven_zip_source.source, seven_zip_source.source_dir)
        seven_zip_source.provision(dst=self.source_dir, keep_7z=True)

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["7z"] is sources.SevenZip)

    def test_pull_failure(self):
        self.fake_check_output.mock.side_effect = subprocess.CalledProcessError(1, [])

        os.makedirs("dst")
        seven_zip = sources.SevenZip(self.test_7z_file_path, "dst")
        raised = self.assertRaises(sources.errors.SnapcraftPullError, seven_zip.pull)
        self.assertThat(
            raised.command, MatchesRegex("7z x .*/{}".format(self.test_7z_file_path))
        )
        self.assertThat(raised.exit_code, Equals(1))
