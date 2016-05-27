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
from __future__ import absolute_import, unicode_literals
import json
import os
from unittest.mock import call, patch

from requests import Response

from snapcraft import tests
from snapcraft.storeapi._download import download


class DownloadBaseTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        # setup patches
        patcher = patch('snapcraft.storeapi._download.get_oauth_session')
        self.mock_get_oauth_session = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('snapcraft.storeapi._download.logger')
        self.mock_logger = patcher.start()
        self.addCleanup(patcher.stop)

        self.mock_get = self.mock_get_oauth_session.return_value.get


class DownloadTestCase(DownloadBaseTestCase):

    def test_download_files_get_metadata_failed(self):
        self.mock_get.side_effect = Exception('some error')

        with self.assertRaises(Exception) as raised:
            download('os', 'edge', 'os.snap', 'amd64')

        self.assertEqual('some error', str(raised.exception))

        self.mock_logger.info.assert_called_once_with(
            "Getting details for 'os'")
        self.assertFalse(os.path.exists('os.snap'))

    def test_download(self):
        snap_content = b'1234567890'
        snap_sha512 = ('12b03226a6d8be9c6e8cd5e55dc6c7920caaa39df14aab92d5e'
                       '3ea9340d1c8a4d3d0b8e4314f1f6ef131ba4bf1ceb9186ab87c'
                       '801af0d5c95b1befb8cedae2b9')
        mock_details = self.mock_get.return_value
        mock_details.ok = True
        mock_details.content = json.dumps({
            '_embedded': {
                    'clickindex:package': [{
                        'download_url': 'http://localhost',
                        'anon_download_url': 'http://localhost',
                        'download_sha512': snap_sha512,
                    }],
            }
        }).encode('utf-8')
        mock_snap = Response()
        mock_snap.status_code = 200
        mock_snap._content = snap_content

        self.mock_get.side_effect = [mock_details, mock_snap]

        download('os', 'edge', 'os.snap', 'amd64')

        self.mock_logger.info.assert_has_calls([
            call("Getting details for 'os'"),
            call("Downloading 'os'"),
            call("Successfully downloaded 'os'")])
        self.assertTrue(os.path.exists('os.snap'))

    def test_download_redownload_as_hash_mismatches(self):
        with open('os.snap', 'wb') as f:
            f.write(b'0000000')

        snap_content = b'1234567890'
        snap_sha512 = ('12b03226a6d8be9c6e8cd5e55dc6c7920caaa39df14aab92d5e'
                       '3ea9340d1c8a4d3d0b8e4314f1f6ef131ba4bf1ceb9186ab87c'
                       '801af0d5c95b1befb8cedae2b9')
        mock_details = self.mock_get.return_value
        mock_details.ok = True
        mock_details.content = json.dumps({
            '_embedded': {
                    'clickindex:package': [{
                        'download_url': 'http://localhost',
                        'anon_download_url': 'http://localhost',
                        'download_sha512': snap_sha512,
                    }],
            }
        }).encode('utf-8')
        mock_snap = Response()
        mock_snap.status_code = 200
        mock_snap._content = snap_content

        self.mock_get.side_effect = [mock_details, mock_snap]

        download('os', 'edge', 'os.snap', 'amd64')

        self.mock_logger.info.assert_has_calls([
            call("Getting details for 'os'"),
            call("Downloading 'os'"),
            call("Successfully downloaded 'os'")])
        self.assertTrue(os.path.exists('os.snap'))

    def test_download_fails_due_to_hash_mismatch(self):
        snap_content = b'1234567890'
        mock_details = self.mock_get.return_value
        mock_details.ok = True
        mock_details.content = json.dumps({
            '_embedded': {
                    'clickindex:package': [{
                        'download_url': 'http://localhost',
                        'anon_download_url': 'http://localhost',
                        'download_sha512': '12345',
                    }],
            }
        }).encode('utf-8')
        mock_snap = Response()
        mock_snap.status_code = 200
        mock_snap._content = snap_content

        self.mock_get.side_effect = [mock_details, mock_snap]

        with self.assertRaises(RuntimeError) as raised:
            download('os', 'edge', 'os.snap', 'amd64')

        self.assertEqual("Failed to download 'os'", str(raised.exception))
        self.mock_logger.info.assert_has_calls([
            call("Getting details for 'os'"),
            call("Downloading 'os'")])
        self.assertTrue(os.path.exists('os.snap'))

    def test_snap_already_downloaded(self):
        snap_content = b'1234567890'
        snap_sha512 = ('12b03226a6d8be9c6e8cd5e55dc6c7920caaa39df14aab92d5e'
                       '3ea9340d1c8a4d3d0b8e4314f1f6ef131ba4bf1ceb9186ab87c'
                       '801af0d5c95b1befb8cedae2b9')

        with open('os.snap', 'wb') as f:
            f.write(snap_content)

        mock_details = self.mock_get.return_value
        mock_details.ok = True
        mock_details.content = json.dumps({
            '_embedded': {
                    'clickindex:package': [{
                        'download_url': 'http://localhost',
                        'anon_download_url': 'http://localhost',
                        'download_sha512': snap_sha512,
                    }],
            }
        }).encode('utf-8')
        mock_snap = Response()
        mock_snap.status_code = 200
        mock_snap._content = snap_content

        self.mock_get.side_effect = [mock_details, mock_snap]

        download('os', 'edge', 'os.snap', 'amd64')

        self.mock_logger.info.assert_has_calls([
            call("Getting details for 'os'"),
            call("Already downloaded 'os'")])
        self.assertTrue(os.path.exists('os.snap'))

    def test_download_fails_when_not_logged_in(self):
        self.mock_get_oauth_session.return_value = None

        with self.assertRaises(EnvironmentError) as raised:
            download('os', 'edge', 'os.snap', 'amd64')

        self.assertEqual(
            'No valid credentials found. Have you run "snapcraft login"?',
            str(raised.exception))
