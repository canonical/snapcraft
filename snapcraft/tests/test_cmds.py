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
import os

import fixtures

import snapcraft.internal
from snapcraft.internal import common
from snapcraft import tests


class TestCommands(tests.TestCase):

    def setUp(self):
        super().setUp()
        common.set_schemadir(os.path.join(__file__,
                             '..', '..', '..', 'schema'))

    def test_load_config_with_invalid_plugin_exits_with_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('my-icon.png', 'w').close()
        with open('snapcraft.yaml', 'w') as f:
            f.write('''name: test-package
version: 1
summary: test
description: test
icon: my-icon.png
confinement: strict
grade: stable

parts:
  part1:
    plugin: does-not-exist
''')

        raised = self.assertRaises(
            SystemExit,
            snapcraft.internal.load_config)

        self.assertEqual(raised.code, 1, 'Wrong exit code returned.')
        self.assertIn(
            'Issue while loading plugin: unknown plugin: does-not-exist\n',
            fake_logger.output)
