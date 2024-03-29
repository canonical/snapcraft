# -*- mode:python; indent-tabs-mode:nil; tab-width:4 -*-
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

import logging
import os

import fixtures
from testtools import TestCase
from testtools import matchers as m

from snapcraft_legacy.internal import deltas
from tests.legacy import fixture_setup


class BaseDeltaGenerationTestCase(TestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeTerminal())
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        self.delta_tool_path = "/usr/bin/xdelta3"
        self.workdir = self.useFixture(fixtures.TempDir()).path
        self.source_file = os.path.join(self.workdir, "source.snap")
        self.target_file = os.path.join(self.workdir, "target.snap")

        with open(self.source_file, "wb") as f:
            f.write(b"This is the source file.")
        with open(self.target_file, "wb") as f:
            f.write(b"This is the target file.")

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft_legacy.file_utils.get_snap_tool_path",
                side_effect=lambda x: os.path.join("/usr", "bin", x),
            )
        )

    def test_find_unique_file_name(self):
        tmp_delta = deltas.BaseDeltasGenerator(
            source_path=self.source_file,
            target_path=self.target_file,
            delta_format="xdelta3",
            delta_tool=self.delta_tool_path,
        )

        unique_file_name = tmp_delta.find_unique_file_name(tmp_delta.source_path)
        self.assertThat(unique_file_name, m.Equals(tmp_delta.source_path + "-0"))
        with open(unique_file_name, "wb") as f:
            f.write(b"tmp file.")

        self.assertThat(
            tmp_delta.find_unique_file_name(tmp_delta.source_path),
            m.Equals(tmp_delta.source_path + "-1"),
        )

    def test_not_set_delta_property_correctly(self):
        self.assertThat(
            lambda: deltas.BaseDeltasGenerator(
                source_path=self.source_file,
                target_path=self.target_file,
                delta_format="not-defined",
                delta_tool="/usr/bin/xdelta3",
            ),
            m.raises(deltas.errors.DeltaFormatOptionError),
        )
        exception = self.assertRaises(
            deltas.errors.DeltaFormatOptionError,
            deltas.BaseDeltasGenerator,
            source_path=self.source_file,
            target_path=self.target_file,
            delta_format="invalid-delta-format",
            delta_tool=self.delta_tool_path,
        )
        expected = """delta_format must be a option in ['xdelta3'].
for now delta_format='invalid-delta-format'"""
        self.assertThat(str(exception), m.Equals(expected))

    def test_file_existence_failed(self):
        class Tmpdelta(deltas.BaseDeltasGenerator):
            pass

        self.assertThat(
            lambda: deltas.BaseDeltasGenerator(
                source_path="invalid-source-file",
                target_path=self.target_file,
                delta_format="xdelta3",
                delta_tool="delta-gen-tool-path",
            ),
            m.raises(ValueError),
        )
        self.assertThat(
            lambda: deltas.BaseDeltasGenerator(
                source_path=self.source_file,
                target_path="invalid-target_file",
                delta_format="xdelta3",
                delta_tool="delta-gen-tool-path",
            ),
            m.raises(ValueError),
        )

    def test_subclass_not_implement_get_delta_cmd(self):
        tmp_delta = deltas.BaseDeltasGenerator(
            source_path=self.source_file,
            target_path=self.target_file,
            delta_format="xdelta3",
            delta_tool=self.delta_tool_path,
        )

        self.assertThat(
            lambda: tmp_delta.make_delta(is_for_test=True),
            m.raises(NotImplementedError),
        )

    def test_large_delta_raises_error(self):
        delta_file = os.path.join(self.workdir, "target.snap.delta")
        target_file = os.path.join(self.workdir, "target.snap")
        source_file = os.path.join(self.workdir, "source.snap")

        with open(delta_file, "wb") as delta, open(source_file, "wb") as source:
            delta.seek(999 - 1)
            delta.write(b"\0")
            source.seek(1000 - 1)
            source.write(b"\0")

        generator = deltas.BaseDeltasGenerator(
            source_path=source_file,
            target_path=target_file,
            delta_format="xdelta3",
            delta_tool=self.delta_tool_path,
        )

        self.assertThat(
            lambda: generator._check_delta_size_constraint(delta_file),
            m.raises(deltas.errors.DeltaGenerationTooBigError),
        )
