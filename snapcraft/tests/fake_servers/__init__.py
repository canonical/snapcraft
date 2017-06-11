# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Canonical Ltd
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

from collections import OrderedDict
from datetime import datetime
import json
import logging
import http.server
import os
import urllib.parse


import pymacaroons
import yaml

import snapcraft.tests


logger = logging.getLogger(__name__)


class BaseHTTPRequestHandler(http.server.BaseHTTPRequestHandler):

    def log_message(*args):
        logger.debug(args)

    def raise_not_implemented(self, path):
        logger.error(
            'Not implemented {} in {} server: {}'.format(
                path, self.__class__.__name__))
        raise NotImplementedError(path)


class FakeFileHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        data = 'Test fake compressed file'
        self.send_response(200)
        self.send_header('Content-Length', len(data))
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(data.encode())


class FakePartsServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakePartsRequestHandler)


class FakePartsRequestHandler(BaseHTTPRequestHandler):

    _date_format = '%a, %d %b %Y %H:%M:%S GMT'
    _parts_date = datetime(2016, 7, 7, 10, 0, 20)

    def do_GET(self):
        logger.debug('Handling getting parts')
        if 'If-Modified-Since' in self.headers:
            ims_date = datetime.strptime(
                self.headers['If-Modified-Since'], self._date_format)
        else:
            ims_date = None
        if ims_date is not None and ims_date >= self._parts_date:
            self.send_response(304)
            response = {}
        else:
            self.send_response(200)
            response = OrderedDict((
                ('curl', OrderedDict((
                    ('plugin', 'autotools'),
                    ('source', 'http://curl.org'),
                    ('description', 'test entry for curl'),
                    ('maintainer', 'none'),
                ))),
                ('part1', OrderedDict((
                    ('plugin', 'go'),
                    ('source', 'http://source.tar.gz'),
                    ('description', 'test entry for part1'),
                    ('maintainer', 'none'),
                ))),
                ('long-described-part', OrderedDict((
                    ('plugin', 'go'),
                    ('source', 'http://source.tar.gz'),
                    ('description', 'this is a repetitive description ' * 3),
                    ('maintainer', 'none'),
                ))),
                ('multiline-part', OrderedDict((
                    ('plugin', 'go'),
                    ('source', 'http://source.tar.gz'),
                    ('description', 'this is a multiline description\n' * 3),
                    ('maintainer', 'none'),
                ))),
            ))
        self.send_header('Content-Type', 'text/plain')
        if 'NO_CONTENT_LENGTH' not in os.environ:
            self.send_header('Content-Length', '1000')
        self.send_header(
            'Last-Modified', self._parts_date.strftime(self._date_format))
        self.send_header('ETag', '1111')
        self.end_headers()
        self.wfile.write(yaml.dump(response).encode())


class FakePartsWikiServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakePartsWikiRequestHandler)


class FakePartsWikiRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        logger.debug('Handling getting parts')
        self.send_response(200)
        response = """
---
origin: https://github.com/sergiusens/curl.git
parts: [curl]
description:
  Description here
maintainer: none
"""
        self.send_header('Content-Type', 'text/plain')
        if 'NO_CONTENT_LENGTH' not in os.environ:
            self.send_header('Content-Length', len(response.encode()))
        self.end_headers()
        self.wfile.write(response.encode())


class FakePartsWikiWithSlashesServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakePartsWikiWithSlashesRequestHandler)


class FakePartsWikiWithSlashesRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        logger.debug('Handling getting parts')
        self.send_response(200)
        response = """
---
origin: https://github.com/sergiusens/curl.git
parts: [curl/a]
description:
  Description here
maintainer: none
---
origin: https://github.com/sergiusens/curl.git
parts: [curl-a]
description:
  Description here
maintainer: none
"""
        self.send_header('Content-Type', 'text/plain')
        if 'NO_CONTENT_LENGTH' not in os.environ:
            self.send_header('Content-Length', len(response.encode()))
        self.end_headers()
        self.wfile.write(response.encode())


class FakePartsWikiOriginServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakePartsWikiOriginRequestHandler)


class FakePartsWikiOriginRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        logger.debug('Handling getting part origin')
        self.send_response(200)
        response = """
parts:
  somepart:
    source: https://github.com/someuser/somepart.git
    plugin: nil
"""
        self.send_header('Content-Type', 'text/plain')
        if 'NO_CONTENT_LENGTH' not in os.environ:
            self.send_header('Content-Length', len(response.encode()))
        self.end_headers()
        self.wfile.write(response.encode())


class FakeSSOServer(http.server.HTTPServer):

    def __init__(self, fake_store, server_address):
        super().__init__(
            server_address, FakeSSORequestHandler)
        self.fake_store = fake_store


class FakeSSORequestHandler(BaseHTTPRequestHandler):

    _API_PATH = '/api/v2/'

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        tokens_discharge_path = urllib.parse.urljoin(
            self._API_PATH, 'tokens/discharge')
        tokens_refresh_path = urllib.parse.urljoin(
            self._API_PATH, 'tokens/refresh')
        if parsed_path.path.startswith(tokens_discharge_path):
            self._handle_tokens_discharge_request()
        elif parsed_path.path.startswith(tokens_refresh_path):
            self._handle_tokens_refresh_request()
        else:
            self.raise_not_implemented(self.path)

    def _handle_tokens_discharge_request(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)
        logger.debug(
            'Handling tokens discharge request with content {}'.format(
                data))
        if (data['password'] == 'test correct password' and
            ('otp' not in data or
             data['otp'] == 'test correct one-time password')):
            self._send_tokens_discharge()
        elif data['password'] == 'test requires 2fa':
            self._send_twofactor_required_error()
        elif data['password'] == 'test 401 invalid json':
            self._send_401_invalid_json()
        else:
            self._send_invalid_credentials_error()

    def _send_tokens_discharge(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'discharge_macaroon': pymacaroons.Macaroon().serialize()
            }
        self.wfile.write(
            json.dumps(response).encode())

    def _send_twofactor_required_error(self):
        self.send_response(401)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'error_list': [{
                'code': 'twofactor-required',
                'message': '2-factor authentication required.',
            }],
        }
        self.wfile.write(json.dumps(response).encode())

    def _send_401_invalid_json(self):
        self.send_response(401)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'invalid{json')

    def _send_invalid_credentials_error(self):
        self.send_response(401)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'error_list': [{
                'code': 'invalid-credentials',
                'message': 'Provided email/password is not correct.',
            }],
        }
        self.wfile.write(json.dumps(response).encode())

    def _handle_tokens_refresh_request(self):
        self._send_tokens_discharge()
        self.server.fake_store.needs_refresh = False


class FakeStoreSearchServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeStoreSearchRequestHandler)


class FakeStoreSearchRequestHandler(BaseHTTPRequestHandler):

    # XXX This fake server as reused as download server, to avoid passing a
    # port as an argument. --elopio - 2016-05-01

    _API_PATH = '/api/v1/'

    def do_GET(self):
        parsed_path = urllib.parse.urlparse(self.path)
        details_path = urllib.parse.urljoin(self._API_PATH,  'snaps/details/')
        download_path = '/download-snap/'
        if parsed_path.path.startswith(details_path):
            self._handle_details_request(
                parsed_path.path[len(details_path):])
        elif parsed_path.path.startswith(download_path):
            self._handle_download_request(
                parsed_path.path[len(download_path):])
        else:
            self.raise_not_implemented(self.path)

    def _handle_details_request(self, package):
        logger.debug(
            'Handling details request for package {}, with headers {}'.format(
                package, self.headers))
        if 'User-Agent' not in self.headers:
            self.send_response(500)
            self.end_headers()
            return
        response = self._get_details_response(package)
        if response is None:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/hal+json')
        self.end_headers()
        self.wfile.write(json.dumps(response).encode())

    def _get_details_response(self, package):
        # ubuntu-core is used in integration tests with fake servers.

        # sha512sum snapcraft/tests/data/test-snap.snap
        test_sha512 = (
            '69d57dcacf4f126592d4e6ff689ad8bb8a083c7b9fe44f6e738ef'
            'd22a956457f14146f7f067b47bd976cf0292f2993ad864ccb498b'
            'fda4128234e4c201f28fe9')

        if package in ('test-snap', 'ubuntu-core'):
            sha512 = test_sha512
        elif package == 'test-snap-with-wrong-sha':
            sha512 = 'wrong sha'
        elif package == 'test-snap-branded-store':
            # Branded-store snaps require Store pinning and authorization.
            if (self.headers.get('X-Ubuntu-Store') != 'Test-Branded' or
                    self.headers.get('Authorization') is None):
                return None
            sha512 = test_sha512
        else:
            return None

        response = {
            'anon_download_url': urllib.parse.urljoin(
                'http://localhost:{}'.format(self.server.server_port),
                'download-snap/test-snap.snap'),
            'download_sha3_384': '1234567890',
            'download_sha512': sha512,
            'snap_id': 'good',
            'developer_id': package + '-developer-id',
            'release': ['16'],
        }
        return response

    def _handle_download_request(self, snap):
        logger.debug('Handling download request for snap {}'.format(snap))
        if 'User-Agent' not in self.headers:
            self.send_repsonse(500)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.end_headers()
        # TODO create a test snap during the test instead of hardcoding it.
        # --elopio - 2016-05-01
        snap_path = os.path.join(
            os.path.dirname(snapcraft.tests.__file__), 'data',
            'test-snap.snap')
        with open(snap_path, 'rb') as snap_file:
            self.wfile.write(snap_file.read())
