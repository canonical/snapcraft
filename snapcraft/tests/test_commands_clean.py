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
from unittest import mock

# Non-Standard Library modules
import fixtures

# Snapcraft modules
from snapcraft import common
from snapcraft import tests
from snapcraft.commands import clean


class CleanCommandTestCase(tests.TestCase):

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

        self.clean_calls = []

        class FakePart:

            def __init__(self, name, partdir):
                self.name = name
                self.partdir = partdir

            def names(self):
                return [self.name, ]

            def clean(s):
                self.clean_calls.append(s.name)

        class FakeConfig:
            all_parts = [
                FakePart('part1', 'partdir1'),
                FakePart('part2', 'partdir2'),
                FakePart('part3', 'partdir3'),
            ]

        self.fake_config = FakeConfig()

        patcher = mock.patch('snapcraft.yaml.load_config')
        self.mock_load_config = patcher.start()
        self.mock_load_config.return_value = self.fake_config
        self.addCleanup(patcher.stop)

    def test_clean_all(self):
        clean.main()

        self.mock_exists.assert_has_calls([
            mock.call(common.get_partsdir()),
            mock.call().__bool__(),
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
        self.assertEqual(self.clean_calls, ['part1', 'part2', 'part3'])

    def test_clean_all_when_all_parts_specified(self):
        clean.main(['part1', 'part2', 'part3'])

        self.mock_exists.assert_has_calls([
            mock.call(common.get_partsdir()),
            mock.call().__bool__(),
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
        self.assertEqual(self.clean_calls, ['part1', 'part2', 'part3'])

    def test_partial_clean(self):
        clean.main(['part1', ])

        self.mock_exists.assert_has_calls([
            mock.call(common.get_partsdir()),
            mock.call().__bool__(),
            mock.call(common.get_snapdir()),
            mock.call().__bool__(),
        ])

        self.mock_rmtree.assert_has_calls([
            mock.call(common.get_snapdir()),
        ])

        self.mock_rmdir.assert_called_once_with(common.get_partsdir())
        self.assertEqual(self.clean_calls, ['part1'])

    def test_everything_is_clean(self):
        self.mock_exists.return_value = False
        self.mock_listdir.side_effect = FileNotFoundError()

        clean.main()

        self.mock_exists.assert_has_calls([
            mock.call(common.get_partsdir()),
            mock.call(common.get_stagedir()),
            mock.call(common.get_snapdir()),
        ])

        self.assertFalse(self.mock_rmdir.called)
        self.assertFalse(self.mock_rmtree.called)
        self.assertEqual(self.clean_calls, ['part1', 'part2', 'part3'])

    def test_no_parts_defined(self):
        self.fake_config.all_parts = []

        self.mock_load_config.return_value = self.fake_config

        clean.main()

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

    def test_part_to_remove_not_defined_exits_with_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        with self.assertRaises(SystemExit) as raised:
            clean.main(['does-not-exist'])
        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            "The part named 'does-not-exist' is not defined in "
            "'snapcraft.yaml'\n",
            fake_logger.output)
