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
from unittest import mock

import fixtures

from snapcraft import cmds
from snapcraft.tests import TestCase


class TestCommands(TestCase):

    def test_check_for_collisions(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = mock.Mock()
        part1.names.return_value = ['part1']
        part1.installdir = tmpdir + '/install1'
        os.makedirs(part1.installdir + '/a')
        open(part1.installdir + '/a/1', mode='w').close()

        part2 = mock.Mock()
        part2.names.return_value = ['part2']
        part2.installdir = tmpdir + '/install2'
        os.makedirs(part2.installdir + '/a')
        open(part2.installdir + '/1', mode='w').close()
        open(part2.installdir + '/2', mode='w').close()
        open(part2.installdir + '/a/2', mode='w').close()

        part3 = mock.Mock()
        part3.names.return_value = ['part3']
        part3.installdir = tmpdir + '/install3'
        os.makedirs(part3.installdir + '/a')
        os.makedirs(part3.installdir + '/b')
        open(part3.installdir + '/1', mode='w').close()
        open(part3.installdir + '/a/2', mode='w').close()

        self.assertTrue(cmds.check_for_collisions([part1, part2]))
        self.assertEqual('', fake_logger.output)

        self.assertFalse(cmds.check_for_collisions([part1, part2, part3]))
        self.assertEqual(
            'Error: parts part2 and part3 have the following files in common:\n'
            '  1\n'
            '  a/2\n',
            fake_logger.output)


class InitTestCase(TestCase):

    def test_init_with_existing_snapcraft_yaml_must_fail(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('snapcraft.yaml', 'w').close()

        with self.assertRaises(SystemExit) as raised:
            cmds.init('dummy args')
        self.assertEqual(raised.exception.code, 1)
        self.assertEqual(
            'snapcraft.yaml already exists!\n', fake_logger.output)

    def test_init_without_parts_must_write_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        snap_without_parts = type('obj', (object, ), {'part': []})
        with self.assertRaises(SystemExit) as raised:
            cmds.init(snap_without_parts)

        self.assertEqual(raised.exception.code, 0)
        self.assertEqual(
            'Wrote the following as snapcraft.yaml:\n'
            'parts:\n',
            fake_logger.output)
