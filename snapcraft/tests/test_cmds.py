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
import tempfile

import fixtures

import snapcraft.yaml

from snapcraft import (
    cmds,
    common,
    lifecycle,
    tests
)


class TestCommands(tests.TestCase):

    def setUp(self):
        super().setUp()
        common.set_schemadir(os.path.join(__file__,
                             '..', '..', '..', 'schema'))

    def test_check_for_collisions(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = lifecycle.load_plugin('part1', 'jdk', {'source': '.'})
        part1.code.installdir = tmpdir + '/install1'
        os.makedirs(part1.installdir + '/a')
        open(part1.installdir + '/a/1', mode='w').close()

        part2 = lifecycle.load_plugin('part2', 'jdk', {'source': '.'})
        part2.code.installdir = tmpdir + '/install2'
        os.makedirs(part2.installdir + '/a')
        with open(part2.installdir + '/1', mode='w') as f:
            f.write('1')
        open(part2.installdir + '/2', mode='w').close()
        with open(part2.installdir + '/a/2', mode='w') as f:
            f.write('a/2')

        part3 = lifecycle.load_plugin('part3', 'jdk', {'source': '.'})
        part3.code.installdir = tmpdir + '/install3'
        os.makedirs(part3.installdir + '/a')
        os.makedirs(part3.installdir + '/b')
        with open(part3.installdir + '/1', mode='w') as f:
            f.write('2')
        with open(part2.installdir + '/2', mode='w') as f:
            f.write('1')
        open(part3.installdir + '/a/2', mode='w').close()

        self.assertTrue(cmds._check_for_collisions([part1, part2]))
        self.assertEqual('', fake_logger.output)

        self.assertFalse(cmds._check_for_collisions([part1, part2, part3]))
        self.assertEqual(
            'Error: parts part2 and part3 have the following file paths in '
            'common which have different contents:\n'
            '  1\n'
            '  a/2\n',
            fake_logger.output)

    def test_load_config_with_invalid_plugin_exits_with_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('my-icon.png', 'w').close()
        with open('snapcraft.yaml', 'w') as f:
            f.write('''name: test-package
version: 1
vendor: me <me@me.com>
summary: test
description: test
icon: my-icon.png

parts:
  part1:
    plugin: does-not-exist
''')

        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.load_config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'Issue while loading plugin: unknown plugin: does-not-exist\n',
            fake_logger.output)
