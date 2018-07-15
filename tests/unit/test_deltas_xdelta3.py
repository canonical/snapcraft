# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import fixtures
import logging
import random
import shutil
from unittest import mock

from progressbar import AnimatedMarker, ProgressBar
from testtools import matchers as m

from snapcraft import file_utils
from snapcraft.internal import deltas  # noqa
from tests import fixture_setup, unit


class XDelta3TestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeTerminal())
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

        self.patch(file_utils, "executable_exists", lambda a: True)

        self.workdir = self.useFixture(fixtures.TempDir()).path
        self.source_file = os.path.join(self.workdir, "source.snap")
        self.target_file = os.path.join(self.workdir, "target.snap")

        with open(self.source_file, "wb") as f:
            f.write(b"This is the source file.")
        with open(self.target_file, "wb") as f:
            f.write(b"This is the target file.")

    def generate_snap_pair(self):
        """Generate more realistic snap data.

        Most tests don't need realistic data, and can use the default dummy
        data that's created by setUp. However, tests that actually run xdelta3
        could use more accurate data. This file generates larger
        binary files. The files will be similar with roughly 0.1 variance along
        1KB block boundaries.
        """
        mean_size = 2 ** 20
        size_stddev = 2 ** 15
        snap_size = int(random.normalvariate(mean_size, size_stddev))

        scratchdir = self.useFixture(fixtures.TempDir()).path
        self.source_file = os.path.join(scratchdir, "source-snap")
        self.target_file = os.path.join(scratchdir, "target-snap")
        # source snap is completely random:
        with open(self.source_file, "wb") as source, open(
            self.target_file, "wb"
        ) as target:
            for i in range(0, snap_size, 1024):
                block = os.urandom(1024)
                source.write(block)
                if random.randint(0, 9) == 0:
                    target.write(os.urandom(1024))
                else:
                    target.write(block)

    def test_raises_DeltaToolError_when_xdelta3_not_installed(self):
        self.patch(file_utils, "executable_exists", lambda a: False)
        self.patch(shutil, "which", lambda a: None)

        self.assertThat(
            lambda: deltas.XDelta3Generator(
                source_path=self.source_file, target_path=self.target_file
            ),
            m.raises(deltas.errors.DeltaToolError),
        )
        self.assertRaises(
            deltas.errors.DeltaToolError,
            deltas.XDelta3Generator,
            source_path=self.source_file,
            target_path=self.target_file,
        )

    def test_xdelta3(self):
        self.generate_snap_pair()
        base_delta = deltas.XDelta3Generator(
            source_path=self.source_file, target_path=self.target_file
        )
        path = base_delta.make_delta(is_for_test=True)

        self.assertThat(path, m.FileExists())
        expect_path = "{}.{}".format(
            base_delta.target_path, base_delta.delta_file_extname
        )
        self.assertThat(path, m.Equals(expect_path))

    def test_xdelta3_with_progress_indicator(self):
        self.generate_snap_pair()
        base_delta = deltas.XDelta3Generator(
            source_path=self.source_file, target_path=self.target_file
        )

        message = "creating delta file from {!r}...".format(self.source_file)
        maxval = 10
        progress_indicator = ProgressBar(
            widgets=[message, AnimatedMarker()], maxval=maxval
        )
        progress_indicator.start()

        path = base_delta.make_delta(
            progress_indicator=progress_indicator, is_for_test=True
        )
        progress_indicator.finish()

        self.assertThat(path, m.FileExists())
        expect_path = "{}.{}".format(
            base_delta.target_path, base_delta.delta_file_extname
        )
        self.assertThat(path, m.Equals(expect_path))

    def test_xdelta3_with_custom_output_dir(self):
        self.generate_snap_pair()
        base_delta = deltas.XDelta3Generator(
            source_path=self.source_file, target_path=self.target_file
        )
        delta_filename = "{}.{}".format(
            os.path.split(base_delta.target_path)[1], base_delta.delta_file_extname
        )

        existed_output_dir = self.useFixture(fixtures.TempDir()).path
        path = base_delta.make_delta(existed_output_dir, is_for_test=True)

        expect_path = os.path.join(existed_output_dir, delta_filename)
        self.assertThat(path, m.FileExists())
        self.assertThat(path, m.Equals(expect_path))

        none_existed_output_dir = (
            self.useFixture(fixtures.TempDir()).path + "/whatever/"
        )
        path = base_delta.make_delta(none_existed_output_dir, is_for_test=True)

        expect_path = os.path.join(none_existed_output_dir, delta_filename)
        self.assertThat(path, m.FileExists())
        self.assertThat(path, m.Equals(expect_path))

    def test_xdelta3_logs(self):
        self.generate_snap_pair()
        base_delta = deltas.XDelta3Generator(
            source_path=self.source_file, target_path=self.target_file
        )
        base_delta.make_delta(is_for_test=True)

        self.assertThat(
            self.fake_logger.output,
            m.Contains(
                "Generating xdelta3 delta for {}".format(
                    os.path.basename(base_delta.target_path)
                )
            ),
        )
        self.assertThat(
            self.fake_logger.output, m.Contains("xdelta3 delta diff generation")
        )

    @mock.patch("subprocess.Popen")
    def test_xdelta3_return_invalid_code(self, mock_subproc_popen):
        # mock the subprocess.Popen with a unexpected returncode
        process_mock = mock.Mock()
        attrs = {"returncode": -1}
        process_mock.configure_mock(**attrs)
        mock_subproc_popen.return_value = process_mock

        self.generate_snap_pair()
        base_delta = deltas.XDelta3Generator(
            source_path=self.source_file, target_path=self.target_file
        )

        self.assertThat(
            lambda: base_delta.make_delta(is_for_test=True),
            m.raises(deltas.errors.DeltaGenerationError),
        )
