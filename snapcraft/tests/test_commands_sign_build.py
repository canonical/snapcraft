# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import os
import logging
from unittest import mock
import shutil
import subprocess

import fixtures

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)


class SnapTest(fixtures.TempDir):
    """Test fixture for copying testing snaps in a temporary directory.

    So they can be manipulated in isolation and all the testing mess
    gets cleaned up automatically.
    """
    data_dir = os.path.join(os.path.dirname(tests.__file__), 'data')

    def __init__(self, test_snap_name):
        super(SnapTest, self).__init__()
        self.test_snap_name = test_snap_name

    def _setUp(self):
        super(SnapTest, self)._setUp()
        test_snap_path = os.path.join(self.data_dir, self.test_snap_name)
        self.snap_path = os.path.join(self.path, self.test_snap_name)
        shutil.copyfile(test_snap_path, self.snap_path)


class SignBuildTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)
        self.snap_test = SnapTest('test-snap.snap')
        self.useFixture(self.snap_test)

    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_snapd_not_installed(self, mock_installed,
                                            mock_check_output):
        mock_installed.return_value = False

        raised = self.assertRaises(
            SystemExit,
            main, ['sign-build', self.snap_test.snap_path, '--local'])

        mock_installed.assert_called_with('snapd')
        self.assertEqual(0, mock_check_output.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'The snapd package is not installed.', self.fake_logger.output)

    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_nonexisting_snap(self, mock_installed,
                                         mock_check_output):
        mock_installed.return_value = True

        raised = self.assertRaises(
            SystemExit,
            main, ['sign-build', 'nonexisting.snap'])

        self.assertEqual(1, raised.code)
        self.assertEqual(0, mock_check_output.call_count)
        self.assertIn(
            'The file \'nonexisting.snap\' does not exist.',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_missing_account_info(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }

        raised = self.assertRaises(
            SystemExit,
            main, ['sign-build', self.snap_test.snap_path])

        self.assertEqual(0, mock_check_output.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'Your account lacks permission to assert builds for this '
            'snap. Make sure you are logged in as the publisher of '
            "'test-snap' for series '16'",
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_no_usable_keys(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.side_effect = [
            '[]',
        ]

        raised = self.assertRaises(
            SystemExit,
            main, ['sign-build', self.snap_test.snap_path])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            'You have no usable keys.',
            'Please create at least one key with `snapcraft create-key` '
            'for use with snap.',
            ], self.fake_logger.output.splitlines())

        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertFalse(os.path.exists(snap_build_path))

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_no_usable_named_key(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.side_effect = [
            '[{"name": "default"}]',
        ]

        raised = self.assertRaises(
            SystemExit,
            main,
            ['sign-build', '--key-name', 'zoing', self.snap_test.snap_path])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            'You have no usable key named "zoing".',
            'See the keys available in your system with `snapcraft keys`.'
        ], self.fake_logger.output.splitlines())

        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertFalse(os.path.exists(snap_build_path))

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_unregistered_key(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'account_keys': [{'public-key-sha3-384': 'another_hash'}],
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
        ]

        raised = self.assertRaises(
            SystemExit,
            main, ['sign-build', self.snap_test.snap_path])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            'The key \'default\' is not registered in the Store.',
            'Please register it with `snapcraft register-key \'default\'` '
            'before signing and pushing signatures to the Store.',
        ], self.fake_logger.output.splitlines())
        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertFalse(os.path.exists(snap_build_path))

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_snapd_failure(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'account_keys': [{'public-key-sha3-384': 'a_hash'}],
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
            subprocess.CalledProcessError(1, ['a', 'b'])
        ]

        raised = self.assertRaises(
            SystemExit,
            main, ['sign-build', self.snap_test.snap_path])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            'Failed to sign build assertion for {}.'.format(
                self.snap_test.snap_path),
            ], self.fake_logger.output.splitlines())

        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertFalse(os.path.exists(snap_build_path))

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_locally_successfully(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.side_effect = [
            '[{"name": "default"}]',
            b'Mocked assertion'
        ]

        main(['sign-build', self.snap_test.snap_path, '--local'])

        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertTrue(os.path.exists(snap_build_path))
        self.assertEqual([
            'Build assertion {} saved to disk.'.format(snap_build_path),
        ], self.fake_logger.output.splitlines())
        mock_check_output.assert_called_with([
            'snap', 'sign-build', '--developer-id=abcd', '--snap-id=snap-id',
            '--grade=stable', '-k', 'default', self.snap_test.snap_path,
        ])

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_missing_grade(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'account_keys': [{'public-key-sha3-384': 'a_hash'}],
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {'name': 'test-snap'}
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
            b'Mocked assertion'
        ]

        main(['sign-build', self.snap_test.snap_path, '--local'])

        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertTrue(os.path.exists(snap_build_path))
        self.assertEqual([
            'Build assertion {} saved to disk.'.format(snap_build_path),
        ], self.fake_logger.output.splitlines())
        mock_check_output.assert_called_with([
            'snap', 'sign-build', '--developer-id=abcd', '--snap-id=snap-id',
            '--grade=stable', '-k', 'default', self.snap_test.snap_path,
        ])

    @mock.patch.object(storeapi.SCAClient, 'push_snap_build')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_push_successfully(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info, mock_push_snap_build):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'account_keys': [{'public-key-sha3-384': 'a_hash'}],
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
            b'Mocked assertion'
        ]

        main(['sign-build', self.snap_test.snap_path])

        snap_build_path = self.snap_test.snap_path + '-build'
        self.assertTrue(os.path.exists(snap_build_path))
        self.assertEqual([
            'Build assertion {} saved to disk.'.format(snap_build_path),
            'Build assertion {} pushed to the Store.'.format(snap_build_path),
        ], self.fake_logger.output.splitlines())
        mock_check_output.assert_called_with([
            'snap', 'sign-build', '--developer-id=abcd', '--snap-id=snap-id',
            '--grade=stable', '-k', 'default', self.snap_test.snap_path,
        ])
        mock_push_snap_build.assert_called_with('snap-id', 'Mocked assertion')

    @mock.patch.object(storeapi.SCAClient, 'push_snap_build')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_push_existing(
            self, mock_installed, mock_get_snap_data, mock_get_account_info,
            mock_push_snap_build):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }

        snap_build_path = self.snap_test.snap_path + '-build'
        with open(snap_build_path, 'wb') as fd:
            fd.write(b'Already signed assertion')

        main(['sign-build', self.snap_test.snap_path])

        self.assertEqual([
            'A signed build assertion for this snap already exists.',
            'Build assertion {} pushed to the Store.'.format(snap_build_path),
        ], self.fake_logger.output.splitlines())
        mock_push_snap_build.assert_called_with(
            'snap-id', 'Already signed assertion')
