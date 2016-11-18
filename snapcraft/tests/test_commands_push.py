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

import glob
import logging
import os
import os.path
from unittest import mock

import docopt
import fixtures
from xdg import BaseDirectory

from snapcraft import (
    storeapi,
    tests
)
from snapcraft.internal.cache._snap import _rewrite_snap_filename_with_revision
from snapcraft.main import main
from snapcraft.storeapi.errors import StoreUploadError
from snapcraft.tests import fixture_setup


class PushCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        patcher = mock.patch('snapcraft.internal.lifecycle.ProgressBar')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_push_without_snap_must_raise_exception(self):
        with self.assertRaises(docopt.DocoptExit) as raised:
            main(['push'])

        self.assertTrue('Usage:' in str(raised.exception))

    def test_push_a_snap(self):
        self.useFixture(fixture_setup.FakeTerminal())

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['push', snap_file])

        self.assertIn(
            'Uploading my-snap-name_0.1_amd64.snap.\n'
            'Revision 9 of \'my-snap-name\' created.',
            self.fake_logger.output)

        mock_upload.assert_called_once_with('my-snap-name', snap_file)

    def test_push_without_login_must_raise_exception(self):
        snap_path = os.path.join(
            os.path.dirname(tests.__file__), 'data',
            'test-snap.snap')
        with self.assertRaises(SystemExit):
            main(['push', snap_path])
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    def test_push_nonexisting_snap_must_raise_exception(self):
        with self.assertRaises(SystemExit):
            main(['push', 'test-unexisting-snap'])

    def test_push_with_updown_error(self):
        # We really don't know of a reason why this would fail
        # aside from a 5xx style error on the server.
        self.useFixture(fixture_setup.FakeTerminal())

        class MockResponse:
            text = 'stub error'
            reason = 'stub reason'

        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.side_effect = StoreUploadError(MockResponse())

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        with self.assertRaises(SystemExit):
            main(['push', snap_file])

    def test_upload_raises_deprecation_warning(self):
        self.useFixture(fixture_setup.FakeTerminal())

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['upload', snap_file])

        self.assertIn(
            'Uploading my-snap-name_0.1_amd64.snap.\n'
            'Revision 9 of \'my-snap-name\' created.',
            self.fake_logger.output)

        mock_upload.assert_called_once_with('my-snap-name', snap_file)

    def test_push_and_release_a_snap(self):
        self.useFixture(fixture_setup.FakeTerminal())

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        patcher = mock.patch.object(storeapi.StoreClient, 'release')
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            'opened_channels': ['beta'],
            'channel_map': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'revision': 9, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'channel': 'edge', 'info': 'tracking'}
            ]
        }

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['push', snap_file, '--release', 'beta'])

        self.assertIn(
            'Uploading my-snap-name_0.1_amd64.snap.\n'
            'Revision 9 of \'my-snap-name\' created.',
            self.fake_logger.output)

        mock_upload.assert_called_once_with('my-snap-name', snap_file)
        mock_release.assert_called_once_with('my-snap-name', 9, ['beta'])

    def test_push_and_release_a_snap_to_N_channels(self):
        self.useFixture(fixture_setup.FakeTerminal())

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        patcher = mock.patch.object(storeapi.StoreClient, 'release')
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            'opened_channels': ['beta,edge,candidate'],
            'channel_map': [
                {'channel': 'stable', 'info': 'none'},
                {'revision': 9, 'channel': 'candidate', 'version': '0',
                 'info': 'specific'},
                {'revision': 9, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'revision': 9, 'channel': 'edge', 'version': '0',
                 'info': 'specific'},
            ]
        }

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['push', snap_file, '--release', 'edge,beta,candidate'])

        self.assertIn(
            'Uploading my-snap-name_0.1_amd64.snap.\n'
            'Revision 9 of \'my-snap-name\' created.',
            self.fake_logger.output)

        mock_upload.assert_called_once_with('my-snap-name', snap_file)
        mock_release.assert_called_once_with('my-snap-name', 9,
                                             ['edge', 'beta', 'candidate'])


class PushCommandDeltasTestCase(tests.TestCase):

    scenarios = [
        ('with deltas', dict(enable_deltas=True)),
        ('without deltas', dict(enable_deltas=False)),
    ]

    def test_push_revision_cached_with_experimental_deltas(self):
        self.useFixture(fixture_setup.FakeTerminal())
        if self.enable_deltas:
            self.useFixture(fixture_setup.DeltaUploads())

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        snap_revision = 9
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': snap_revision,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['push', snap_file])

        revision_cache = os.path.join(
            BaseDirectory.xdg_cache_home,
            'snapcraft',
            'my-snap-name',
            'revisions')
        cached_snap = _rewrite_snap_filename_with_revision(
            snap_file,
            snap_revision)

        self.assertEqual(self.enable_deltas, os.path.isfile(
            os.path.join(revision_cache, cached_snap)))


class PushCommandDeltasWithPruneTestCase(tests.TestCase):

    scenarios = [
        ('delete other cache files with valid name', {
            'cached_snaps': [
                'a-cached-snap_0.3_amd64_8.snap',
                'another-cached-snap_1.0_arm64_6.snap']
        }),
        ('delete other cache files with invalid name', {
            'cached_snaps': [
                'a-cached-snap_0.3_amd64.snap',
                'cached-snap-without-revision_1.0_arm64.snap',
                'another-cached-snap-without-version_arm64.snap']
        })
    ]

    def test_push_revision_prune_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())
        self.useFixture(fixture_setup.DeltaUploads())

        snap_revision = 9

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': snap_revision,
        }

        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        revision_cache = os.path.join(
            BaseDirectory.xdg_cache_home,
            'snapcraft', 'my-snap-name', 'revisions')

        os.makedirs(revision_cache)

        for cached_snap in self.cached_snaps:
            open(os.path.join(revision_cache, cached_snap), 'a').close()

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', snap_file])

        real_cached_snap = _rewrite_snap_filename_with_revision(
            snap_file,
            snap_revision
        )
        self.assertTrue(
            os.path.isfile(os.path.join(revision_cache, real_cached_snap)))

        for snap in self.cached_snaps:
            self.assertFalse(
                os.path.isfile(os.path.join(revision_cache, snap)))
        self.assertEqual(1, len(os.listdir(revision_cache)))
