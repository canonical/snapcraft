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

from snapcraft.main import main
from snapcraft import tests


class InitCommandTestCase(tests.TestCase):

    def test_init_with_existing_snapcraft_yaml_must_fail(self):
        open('snapcraft.yaml', 'w').close()

        with self.assertRaises(SystemExit) as raised:
            main(['init'])

        self.assertEqual(
            'snapcraft.yaml already exists!', str(raised.exception))

    def test_init_must_write_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        expected_yaml = """name: # the name of the snap
version: # the version of the snap
summary: # 79 char long summary
description: # a longer description for the snap
confinement: devmode # devmode means no confinement is supported"""

        main(['init'])

        self.assertEqual('Created snapcraft.yaml.\n', fake_logger.output)

        # Verify the generated yaml
        with open('snapcraft.yaml', 'r') as f:
            self.assertEqual(f.read(), expected_yaml)
