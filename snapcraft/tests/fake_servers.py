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

from collections import OrderedDict
from datetime import datetime
import json
import logging
import http.server
import os
import re
import urllib.parse

import pymacaroons
import yaml

import snapcraft.tests


logger = logging.getLogger(__name__)


class BaseHTTPRequestHandler(http.server.BaseHTTPRequestHandler):

    def log_message(*args):
        logger.debug(args)


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

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeSSORequestHandler)


class FakeSSORequestHandler(BaseHTTPRequestHandler):

    _API_PATH = '/api/v2/'

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        tokens_discharge_path = urllib.parse.urljoin(
            self._API_PATH, 'tokens/discharge')
        if parsed_path.path.startswith(tokens_discharge_path):
            self._handle_tokens_discharge_request()
        else:
            logger.error('Not implemented path in fake SSO server: {}'.format(
                self.path))
            raise NotImplementedError(self.path)

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
            self._send_tokens_discharge(data)
        else:
            self._send_invalid_credentials_error()

    def _send_tokens_discharge(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'discharge_macaroon': pymacaroons.Macaroon().serialize()
            }
        self.wfile.write(
            json.dumps(response).encode())

    def _send_invalid_credentials_error(self):
        self.send_response(401)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'code': 'INVALID_CREDENTIALS',
            'message': 'Provided email/password is not correct.',
            'message': 'Provided email/password is not correct.',
        }
        self.wfile.write(json.dumps(response).encode())


class FakeStoreUploadServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeStoreUploadRequestHandler)


class FakeStoreUploadRequestHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        if parsed_path.path.startswith('/unscanned-upload/'):
            self._handle_upload_request()
        else:
            logger.error(
                'Not implemented path in fake Store Upload server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_upload_request(self):
        logger.info('Handling upload request')
        if 'UPDOWN_BROKEN' in os.environ:
            response_code = 500
            content_type = 'text/plain'
            response = b'Broken'
        else:
            response_code = 200
            content_type = 'application/octet-stream'
            response = json.dumps({'upload_id': 'test-upload-id'}).encode()
        self.send_response(response_code)
        self.send_header('Content-Type', content_type)
        self.end_headers()
        self.wfile.write(response)


class FakeStoreAPIServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeStoreAPIRequestHandler)
        self.account_keys = []


class FakeStoreAPIRequestHandler(BaseHTTPRequestHandler):

    _DEV_API_PATH = '/dev/api/'

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        acl_path = urllib.parse.urljoin(self._DEV_API_PATH, 'acl/')
        account_key_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'account/account-key')
        register_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'register-name/')
        upload_path = urllib.parse.urljoin(self._DEV_API_PATH, 'snap-push/')
        release_path = urllib.parse.urljoin(self._DEV_API_PATH,
                                            'snap-release/')
        if parsed_path.path.startswith(acl_path):
            permission = parsed_path.path[len(acl_path):].strip('/')
            self._handle_acl_request(permission)
        elif parsed_path.path == account_key_path:
            self._handle_account_key_request()
        elif parsed_path.path.startswith(register_path):
            self._handle_registration_request()
        elif parsed_path.path.startswith(upload_path):
            self._handle_upload_request()
        elif parsed_path.path.startswith(release_path):
            self._handle_release_request()
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_acl_request(self, permission):
        logger.debug(
            'Handling ACL request for {}'.format(permission))
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        macaroon = pymacaroons.Macaroon(
            caveats=[
                pymacaroons.Caveat(
                    caveat_id='test caveat',
                    location='localhost',
                    verification_key_id='test verifiacion')
            ])
        response = {'macaroon': macaroon.serialize()}
        self.wfile.write(json.dumps(response).encode())

    def _handle_account_key_request(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)
        logger.debug(
            'Handling account-key request with content {}'.format(data))
        account_key_request = data['account_key_request']

        if account_key_request == 'test-not-implemented':
            self._handle_account_key_501()
        elif account_key_request == 'test-invalid-data':
            error = {
                'code': 'invalid-field',
                'message': 'The account-key-request assertion is not valid.',
            }
            self._handle_account_key_400({'error_list': [error]})
        else:
            self._handle_successful_account_key(account_key_request)

    def _handle_successful_account_key(self, account_key_request):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        # Extremely basic assertion parsing, just enough to make tests work.
        # Don't copy this.
        key_name = re.search(
            '^name: (.*)$', account_key_request, flags=re.MULTILINE).group(1)
        key_id = re.search(
            '^public-key-sha3-384: (.*)$', account_key_request,
            flags=re.MULTILINE).group(1)
        self.server.account_keys.append(
            {'name': key_name, 'public-key-sha3-384': key_id})
        response = {
            'account_key': {
                'account-id': 'abcd',
                'name': key_name,
                'public-key-sha3-384': key_id,
            },
        }
        self.wfile.write(json.dumps(response).encode())

    def _handle_account_key_400(self, error):
        self.send_response(400)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(error).encode())

    def _handle_account_key_501(self):
        self.send_response(501)
        self.send_header('Content-Type', 'text/plain')
        self.end_headers()
        self.wfile.write(b'Not Implemented')

    def _handle_registration_request(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)
        logger.debug(
            'Handling registration request with content {}'.format(data))

        if data['snap_name'] == 'test-already-registered-snap-name':
            self._handle_register_409('already_registered')
        elif data['snap_name'] == 'test-reserved-snap-name':
            self._handle_register_409('reserved_name')
        elif data['snap_name'].startswith('test-too-fast'):
            self._handle_register_429('register_window')
        elif data['snap_name'] == 'snap-name-no-clear-error':
            self._handle_unclear_registration_error()
        else:
            self._handle_successful_registration()

    def _handle_successful_registration(self):
        self.send_response(201)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {'snap_id': 'test-snap-id'}
        self.wfile.write(json.dumps(response).encode())

    def _handle_register_409(self, error_code):
        self.send_response(409)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'status': 409,
            'code': error_code,
            'register_name_url': 'https://myapps.com/register-name/',
        }
        self.wfile.write(json.dumps(response).encode())

    def _handle_register_429(self, error_code):
        self.send_response(409)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'status': 429,
            'code': error_code,
        }
        if error_code == 'register_window':
            response['retry_after'] = 177
        self.wfile.write(json.dumps(response).encode())

    def _handle_unclear_registration_error(self):
        self.send_response(409)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'status': 409,
            'code': 'unexistent_error_code',
        }
        self.wfile.write(json.dumps(response).encode())

    def _handle_upload_request(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)
        logger.debug(
            'Handling upload request with content {}'.format(data))

        response_code = 202
        details_path = 'details/upload-id/good-snap'
        if data['name'] == 'test-review-snap':
            details_path = 'details/upload-id/review-snap'
        elif data['name'] == 'test-snap-unregistered':
            response_code = 404

        logger.debug('Handling upload request')
        self.send_response(response_code)
        if response_code == 404:
            self.send_header('Content-Type', 'text/plain')
            data = b''
        else:
            self.send_header('Content-Type', 'application/json')
            response = {
                'status_details_url': urllib.parse.urljoin(
                    'http://localhost:{}/'.format(self.server.server_port),
                    details_path
                    ),
            }
            data = json.dumps(response).encode()
        self.end_headers()

        self.wfile.write(data)

    def _handle_release_request(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)
        logger.debug(
            'Handling release request with content {}'.format(data))
        response_code = 200
        content_type = 'application/json'
        if data['name'] == 'test-snap-unregistered':
            response_code = 404
            content_type = 'text/plain'
            data = b''
        elif 'alpha' in data['channels']:
            response_code = 400
            response = {
                'errors': 'Not a valid channel: alpha',
            }
            data = json.dumps(response).encode()
        elif data['name'] == 'test-snap' or data['name'].startswith('u1test'):
            response = {
                'opened_channels': data['channels'],
                'channel_map': [
                    {'channel': 'stable', 'info': 'none'},
                    {'channel': 'candidate', 'info': 'none'},
                    {'revision': int(data['revision']), 'channel': 'beta',
                     'version': '0', 'info': 'specific'},
                    {'channel': 'edge', 'info': 'tracking'}
                ]
            }
            data = json.dumps(response).encode()
        else:
            raise NotImplementedError(
                'Cannot handle release request for {!r}'.format(data['name']))

        logger.debug('Handling release request')
        self.send_response(response_code)
        self.send_header('Content-Type', content_type)
        self.end_headers()

        self.wfile.write(data)

    def do_GET(self):
        parsed_path = urllib.parse.urlparse(self.path)
        details_good = urllib.parse.urljoin(
            self._DEV_API_PATH, '/details/upload-id/good-snap')
        details_review = urllib.parse.urljoin(
            self._DEV_API_PATH, '/details/upload-id/review-snap')
        account_path = urllib.parse.urljoin(self._DEV_API_PATH, 'account')
        if parsed_path.path.startswith(details_good):
            self._handle_scan_complete_request('ready_to_release', True)
        elif parsed_path.path.startswith(details_review):
            self._handle_scan_complete_request('need_manual_review', False)
        elif parsed_path.path == account_path:
            self._handle_account_request()
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(parsed_path)

    def _handle_scan_complete_request(self, code, can_release):
        logger.debug('Handling scan complete request')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'code': code,
            'url': '/dev/click-apps/5349/rev/1',
            'can_release': can_release,
            'revision': '1',
            'processed': True
        }
        self.wfile.write(json.dumps(response).encode())

    def _handle_account_request(self):
        logger.debug('Handling account request')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({
            'account_id': 'abcd',
            'account_keys': self.server.account_keys,
        }).encode())


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
        search_path = urllib.parse.urljoin(self._API_PATH,  'search')
        download_path = '/download-snap/'
        if parsed_path.path.startswith(search_path):
            self._handle_search_request(
                urllib.parse.parse_qs(parsed_path.query)['q'])
        elif parsed_path.path.startswith(download_path):
            self._handle_download_request(parsed_path[len(download_path):])
        else:
            logger.error(
                'Not implemented path in fake Store Search server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_search_request(self, query):
        if len(query) > 1 or 'package_name:' not in query[0]:
            logger.error(
                'Not implemented query in fake Store Search server: {}'.format(
                    query))
            raise NotImplementedError(query)
        query = query[0]
        package = query.split('package_name:')[1].strip('"')
        logger.debug(
            'Handling search request for package {}, with headers {}'.format(
                package, self.headers))
        self.send_response(200)
        self.send_header('Content-Type', 'application/hal+json')
        self.end_headers()
        response = self._get_search_response(package)
        self.wfile.write(json.dumps(response).encode())

    def _get_search_response(self, package):
        # ubuntu-core is used in integration tests with fake servers.
        if package in ('test-snap', 'ubuntu-core'):
            # sha512sum snapcraft/tests/data/test-snap.snap
            sha512 = (
                '69d57dcacf4f126592d4e6ff689ad8bb8a083c7b9fe44f6e738ef'
                'd22a956457f14146f7f067b47bd976cf0292f2993ad864ccb498b'
                'fda4128234e4c201f28fe9')
        elif package == 'test-snap-with-wrong-sha':
            sha512 = 'wrong sha'
        else:
            return {}
        response = {'_embedded': {
            'clickindex:package': [
                {'download_url': urllib.parse.urljoin(
                    'http://localhost:{}'.format(self.server.server_port),
                    'download-snap/test-snap.snap'),
                 'download_sha512': sha512}]}}
        return response

    def _handle_download_request(self, snap):
        logger.debug('Handling download request for snap {}'.format(snap))
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
