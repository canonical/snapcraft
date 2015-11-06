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

import io
import logging
import os
import tempfile
from unittest import mock

import fixtures

from snapcraft import (
    cmds,
    common,
    tests
)


class TestCommands(tests.TestCase):

    def test_check_for_collisions(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = mock.Mock()
        part1.name = 'part1'
        part1.code.options.stage = ['*']
        part1.installdir = tmpdir + '/install1'
        os.makedirs(part1.installdir + '/a')
        open(part1.installdir + '/a/1', mode='w').close()

        part2 = mock.Mock()
        part2.name = 'part2'
        part2.code.options.stage = ['*']
        part2.installdir = tmpdir + '/install2'
        os.makedirs(part2.installdir + '/a')
        with open(part2.installdir + '/1', mode='w') as f:
            f.write('1')
        open(part2.installdir + '/2', mode='w').close()
        with open(part2.installdir + '/a/2', mode='w') as f:
            f.write('a/2')

        part3 = mock.Mock()
        part3.name = 'part3'
        part3.code.options.stage = ['*']
        part3.installdir = tmpdir + '/install3'
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

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_list_plugins(self, mock_stdout):
        expected_list = '''ant
autotools
catkin
cmake
copy
go
jdk
make
maven
python2
python3
qml
roscore
scons
tar-content
'''
        cmds.list_plugins()
        self.assertEqual(mock_stdout.getvalue(), expected_list)


class CleanTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('shutil.rmtree')
        self.mock_rmtree = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('os.path.exists')
        self.mock_exists = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('os.listdir')
        self.mock_listdir = patcher.start()
        self.mock_listdir.return_value = []
        self.addCleanup(patcher.stop)

        patcher = mock.patch('os.rmdir')
        self.mock_rmdir = patcher.start()
        self.addCleanup(patcher.stop)

        class FakePart:

            def __init__(self, name, partdir):
                self.name = name
                self.partdir = partdir

            def names(self):
                return [self.name, ]

        class FakeConfig:
            all_parts = [
                FakePart('part1', 'partdir1'),
                FakePart('part2', 'partdir2'),
                FakePart('part3', 'partdir3'),
            ]

        self.fake_config = FakeConfig()

        patcher = mock.patch('snapcraft.cmds._load_config')
        self.mock_load_config = patcher.start()
        self.mock_load_config.return_value = self.fake_config
        self.addCleanup(patcher.stop)

    def test_clean_all(self):
        cmds.clean({})

        self.mock_exists.assert_has_calls([
            mock.call('partdir1'),
            mock.call().__bool__(),
            mock.call('partdir2'),
            mock.call().__bool__(),
            mock.call('partdir3'),
            mock.call().__bool__(),
            mock.call(common.get_partsdir()),
            mock.call().__bool__(),
            mock.call(common.get_stagedir()),
            mock.call().__bool__(),
            mock.call(common.get_snapdir()),
            mock.call().__bool__(),
        ])

        self.mock_rmtree.assert_has_calls([
            mock.call('partdir1'),
            mock.call('partdir2'),
            mock.call('partdir3'),
            mock.call(common.get_stagedir()),
            mock.call(common.get_snapdir()),
        ])

        self.mock_rmdir.assert_called_once_with(common.get_partsdir())

    def test_everything_is_clean(self):
        self.mock_exists.return_value = False
        self.mock_listdir.side_effect = FileNotFoundError()

        cmds.clean({})

        self.mock_exists.assert_has_calls([
            mock.call('partdir1'),
            mock.call('partdir2'),
            mock.call('partdir3'),
            mock.call(common.get_partsdir()),
            mock.call(common.get_stagedir()),
            mock.call(common.get_snapdir()),
        ])

        self.assertFalse(self.mock_rmdir.called)
        self.assertFalse(self.mock_rmtree.called)

    def test_no_parts_defined(self):
        self.fake_config.all_parts = []

        self.mock_load_config.return_value = self.fake_config

        cmds.clean({})

        self.mock_exists.assert_has_calls([
            mock.call(common.get_stagedir()),
            mock.call().__bool__(),
            mock.call(common.get_snapdir()),
            mock.call().__bool__(),
        ])

        self.mock_rmtree.assert_has_calls([
            mock.call(common.get_stagedir()),
            mock.call(common.get_snapdir()),
        ])

        self.mock_rmdir.assert_called_once_with(common.get_partsdir())


class InitTestCase(tests.TestCase):

    def test_init_with_existing_snapcraft_yaml_must_fail(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('snapcraft.yaml', 'w').close()

        with self.assertRaises(SystemExit) as raised:
            cmds.init('dummy args')
        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'snapcraft.yaml already exists!\n', fake_logger.output)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_init_without_parts_must_write_snapcraft_yaml(self, mock_stdout):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        snap_without_parts = type('obj', (object, ), {'part': []})
        with self.assertRaises(SystemExit) as raised:
            cmds.init(snap_without_parts)

        self.assertEqual(raised.exception.code, 0, 'Wrong exit code returned.')
        self.assertEqual(
            'Wrote the following as snapcraft.yaml.\n',
            fake_logger.output)

        expected_out = '''
name: # the name of the snap
version: # the version of the snap
# The vendor for the snap (replace 'Vendor <email@example.com>')
vendor: Vendor <email@example.com>
summary: # 79 char long summary
description: # A longer description for the snap
icon: # A path to an icon for the package
'''
        self.assertEqual(mock_stdout.getvalue(), expected_out)
