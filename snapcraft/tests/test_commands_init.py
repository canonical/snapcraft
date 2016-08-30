# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('snapcraft.yaml', 'w').close()

        with self.assertRaises(SystemExit) as raised:
            main(['init'])

        self.assertEqual(1, raised.exception.code)
        self.assertEqual(
            fake_logger.output,
            'snapcraft.yaml already exists!\n')

    def test_init_with_existing_dot_snapcraft_yaml_must_fail(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('.snapcraft.yaml', 'w').close()

        with self.assertRaises(SystemExit) as raised:
            main(['init'])

        self.assertEqual(1, raised.exception.code)
        self.assertEqual(
            fake_logger.output,
            '.snapcraft.yaml already exists!\n')

    def test_init_must_write_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        expected_yaml = """name: my-snap  # the name of the snap
version: 0  # the version of the snap
summary: This is my-snap's summary  # 79 char long summary
description: This is my-snap's description  # a longer description for the snap
confinement: devmode  # use "strict" to enforce system access only via \
declared interfaces
grade: devel # use "stable" to assert the snap quality

parts:
    my-part:  # Replace with a part name of your liking
        # Get more information about plugins by running
        # snapcraft help plugins
        # and more information about the available plugins
        # by running
        # snapcraft list-plugins
        plugin: nil"""

        main(['init'])

        self.assertEqual(
            'Created snapcraft.yaml.\nEdit the file to your '
            'liking or run `snapcraft` to get started\n',
            fake_logger.output)

        # Verify the generated yaml
        with open('snapcraft.yaml', 'r') as f:
            self.assertEqual(f.read(), expected_yaml)
