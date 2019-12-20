# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
from testtools.matchers import Equals

from snapcraft.internal.meta import command
from tests import unit


class CommandTests(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_get_command_path(self):
        command_path = os.path.join(self.path, "foo")
        self.assertThat(
            command._get_command_path(command="foo", prime_dir=self.path),
            Equals(command_path),
        )
        self.assertThat(
            command._get_command_path(command="/foo", prime_dir=self.path),
            Equals(command_path),
        )
        self.assertThat(
            command._get_command_path(command="$SNAP/foo", prime_dir=self.path),
            Equals(command_path),
        )
        self.assertThat(
            command._get_command_path(command="/$SNAP/foo", prime_dir=self.path),
            Equals(command_path),
        )

    def test_massage_command(self):
        self.assertThat(
            command._massage_command(command="/foo", prime_dir=self.path),
            Equals("/foo"),
        )
