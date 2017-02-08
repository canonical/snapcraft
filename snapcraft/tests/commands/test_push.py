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
from testtools.matchers import FileExists, Not

import snapcraft
from snapcraft import (
    file_utils,
    storeapi,
    tests
)
from snapcraft.main import main
from snapcraft.storeapi.errors import (
    StoreDeltaApplicationError,
    StorePushError,
    StoreUploadError
)
from snapcraft.tests import fixture_setup


class PushCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        patcher = mock.patch('snapcraft.internal.lifecycle.ProgressBar')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.storeapi.StoreClient.push_precheck')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_push_without_snap_must_raise_exception(self):
        raised = self.assertRaises(
            docopt.DocoptExit,
            main, ['push'])

        self.assertTrue('Usage:' in str(raised))

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

        self.assertRegexpMatches(
            self.fake_logger.output,
            ".*Pushing 'my-snap-name_0\.1_\w*.snap' to the store\.\n"
            "Revision 9 of 'my-snap-name' created\.",
        )

        mock_upload.assert_called_once_with('my-snap-name', snap_file)

    def test_push_without_login_must_raise_exception(self):
        snap_path = os.path.join(
            os.path.dirname(tests.__file__), 'data',
            'test-snap.snap')
        self.assertRaises(
            SystemExit,
            main, ['push', snap_path])
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    def test_push_nonexisting_snap_must_raise_exception(self):
        self.assertRaises(
            SystemExit,
            main, ['push', 'test-unexisting-snap'])

    def test_push_unregistered_snap_must_raise_exception(self):
        self.useFixture(fixture_setup.FakeTerminal())

        class MockResponse:
            status_code = 404
            error_list = [{'code': 'resource-not-found',
                           'message': 'Snap not found for name=my-snap-name'}]

        patcher = mock.patch.object(storeapi.StoreClient, 'push_precheck')
        mock_precheck = patcher.start()
        self.addCleanup(patcher.stop)
        mock_precheck.side_effect = StorePushError(
            'my-snap-name', MockResponse())

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        self.assertRaises(
            SystemExit,
            main, ['push', snap_file])

        self.assertIn(
            'You are not the publisher or allowed to push revisions for this '
            'snap. To become the publisher, run `snapcraft register '
            'my-snap-name` and try to push again.',
            self.fake_logger.output)

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

        self.assertRaises(
            SystemExit,
            main, ['push', snap_file])

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

        self.assertRegexpMatches(
            self.fake_logger.output,
            ".*Pushing 'my-snap-name_0\.1_\w*.snap\' to the store\.\n"
            "Revision 9 of 'my-snap-name' created.",
        )

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
            'channel_map_tree': {
                'latest': {
                    '16': {
                        'amd64':
                        [
                            {'channel': 'stable', 'info': 'none'},
                            {'channel': 'candidate', 'info': 'none'},
                            {'revision': 9, 'channel': 'beta', 'version': '0',
                             'info': 'specific'},
                            {'channel': 'edge', 'info': 'tracking'}
                        ]
                    }
                }
            }
        }

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['push', snap_file, '--release', 'beta'])

        self.assertRegexpMatches(
            self.fake_logger.output,
            ".*Pushing 'my-snap-name_0\.1_\w*\.snap\' to the store\.\n"
            "Revision 9 of 'my-snap-name' created\.\n"
            "The 'beta' channel is now open\.\n")

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
            'channel_map_tree': {
                'latest': {
                    '16': {
                        'amd64':
                        [
                            {'channel': 'stable', 'info': 'none'},
                            {'revision': 9, 'channel': 'candidate',
                             'version': '0', 'info': 'specific'},
                            {'revision': 9, 'channel': 'beta', 'version': '0',
                             'info': 'specific'},
                            {'revision': 9, 'channel': 'edge', 'version': '0',
                             'info': 'specific'},
                        ]
                    }
                }
            }
        }

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['push', snap_file, '--release', 'edge,beta,candidate'])

        self.assertRegexpMatches(
            self.fake_logger.output,
            ".*Pushing 'my-snap-name_0\.1_\w*.snap\' to the store\.\n"
            "Revision 9 of 'my-snap-name' created\.\n"
            "The 'beta,edge,candidate' channel is now open\.\n"
        )

        mock_upload.assert_called_once_with('my-snap-name', snap_file)
        mock_release.assert_called_once_with('my-snap-name', 9,
                                             ['edge', 'beta', 'candidate'])


class PushCommandDeltasTestCase(tests.TestCase):

    scenarios = [
        ('with deltas', dict(enable_deltas=True)),
        ('without deltas', dict(enable_deltas=False)),
    ]

    def setUp(self):
        super().setUp()

        self.latest_snap_revision = 8
        self.new_snap_revision = self.latest_snap_revision + 1

        patcher = mock.patch('snapcraft.storeapi.StoreClient.push_precheck')
        patcher.start()
        self.addCleanup(patcher.stop)

        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': self.new_snap_revision,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'get_snap_history')
        mock_release = patcher.start()
        mock_release.return_value = [self.latest_snap_revision]
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        self.mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        self.mock_upload.return_value = mock_tracker

        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def test_push_revision_cached_with_experimental_deltas(self):
        self.useFixture(fixture_setup.FakeTerminal())
        if self.enable_deltas:
            self.useFixture(fixture_setup.DeltaUploads())

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', snap_file])

        snap_cache = os.path.join(
            BaseDirectory.xdg_cache_home,
            'snapcraft',
            'projects',
            'my-snap-name',
            'snap_hashes',
            self.deb_arch,
        )
        cached_snap = os.path.join(
            snap_cache, file_utils.calculate_sha3_384(snap_file))

        if self.enable_deltas:
            self.assertThat(cached_snap, FileExists())
        else:
            self.assertThat(cached_snap, Not(FileExists()))

    def test_push_revision_uses_available_delta(self):
        self.useFixture(fixture_setup.FakeTerminal())
        if self.enable_deltas:
            self.useFixture(fixture_setup.DeltaUploads())

        # Create a source snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', snap_file])

        # create an additional snap, potentially a delta target
        main(['snap'])
        new_snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', new_snap_file])

        _, kwargs = self.mock_upload.call_args
        if self.enable_deltas:
            self.assertEqual(kwargs.get('delta_format'), 'xdelta3')
        else:
            self.assertIsNone(kwargs.get('delta_format'))

    def test_push_with_upload_failure_falls_back(self):
        self.useFixture(fixture_setup.FakeTerminal())
        self.useFixture(fixture_setup.DeltaUploads())

        # Create a source snap to delta from
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', snap_file])

        # create a target snap
        main(['snap'])

        # Raise exception in delta upload
        patcher = mock.patch('snapcraft._store._push_delta')
        mock_push_delta = patcher.start()
        self.addCleanup(patcher.stop)
        mock_push_delta.side_effect = StoreDeltaApplicationError(
            'There has been a problem while processing a snap delta.')

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

        # Upload and ensure fallback is called
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', snap_file])
            mock_upload.assert_called_once_with('my-snap-name', snap_file)


class PushCommandDeltasWithPruneTestCase(tests.TestCase):

    scenarios = [
        ('delete other cache files with valid name', {
            'cached_snaps': [
                'a-cached-snap_0.3_{}_8.snap',
                'another-cached-snap_1.0_fakearch_6.snap']
        }),
        ('delete other cache files with invalid name', {
            'cached_snaps': [
                'a-cached-snap_0.3_{}.snap',
                'cached-snap-without-revision_1.0_fakearch.snap',
                'another-cached-snap-without-version_fakearch.snap']
        })
    ]

    def test_push_revision_prune_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())
        self.useFixture(fixture_setup.DeltaUploads())

        snap_revision = 9

        patcher = mock.patch('snapcraft.storeapi.StoreClient.push_precheck')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(storeapi.StoreClient, 'get_snap_history')
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = [snap_revision]

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

        deb_arch = snapcraft.ProjectOptions().deb_arch

        snap_cache = os.path.join(
            BaseDirectory.xdg_cache_home,
            'snapcraft',
            'projects',
            'my-snap-name',
            'snap_hashes',
            deb_arch
        )
        os.makedirs(snap_cache)

        for cached_snap in self.cached_snaps:
            cached_snap = cached_snap.format(deb_arch)
            open(os.path.join(snap_cache, cached_snap), 'a').close()

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker'):
            main(['push', snap_file])

        real_cached_snap = os.path.join(
            snap_cache,
            file_utils.calculate_sha3_384(snap_file)
        )

        self.assertThat(os.path.join(snap_cache, real_cached_snap),
                        FileExists())

        for snap in self.cached_snaps:
            snap = snap.format(deb_arch)
            self.assertThat(os.path.join(snap_cache, snap),
                            Not(FileExists()))
        self.assertEqual(1, len(os.listdir(snap_cache)))
