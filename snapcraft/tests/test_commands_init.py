# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import fixtures

from snapcraft import tests
from snapcraft.commands import init


class InitCommandTestCase(tests.TestCase):

    def test_init_with_existing_snapcraft_yaml_must_fail(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('snapcraft.yaml', 'w').close()

        with self.assertRaises(SystemExit) as raised:
            init.main()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'snapcraft.yaml already exists!\n', fake_logger.output)

    def test_init_must_write_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        init.main()

        self.assertEqual('Created snapcraft.yaml.\n', fake_logger.output)
