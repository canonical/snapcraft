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

    def __init__(self, fake_store, server_address):
        super().__init__(
            server_address, FakeStoreAPIRequestHandler)
        self.fake_store = fake_store
        self.account_keys = []
        self.registered_names = []


class FakeStoreAPIRequestHandler(BaseHTTPRequestHandler):

    _DEV_API_PATH = '/dev/api/'

    def do_POST(self):
        self._handle_refresh()
        parsed_path = urllib.parse.urlparse(self.path)
        acl_path = urllib.parse.urljoin(self._DEV_API_PATH, 'acl/')
        account_key_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'account/account-key')
        snap_id_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/')
        register_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'register-name/')
        upload_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snap-push/')
        release_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snap-release/')
        agreement_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'agreement/')

        if parsed_path.path.startswith(acl_path):
            permission = parsed_path.path[len(acl_path):].strip('/')
            self._handle_acl_request(permission)
        elif parsed_path.path == account_key_path:
            self._handle_account_key_request()
        elif parsed_path.path.startswith(snap_id_path):
            if parsed_path.path.endswith('/builds'):
                self._handle_sign_build_request()
            elif parsed_path.path.endswith('/close'):
                self._handle_close_request()
        elif parsed_path.path.startswith(register_path):
            self._handle_registration_request()
        elif parsed_path.path.startswith(upload_path):
            self._handle_upload_request()
        elif parsed_path.path.startswith(release_path):
            self._handle_release_request()
        elif parsed_path.path.startswith(agreement_path):
            self._handle_sign_request()
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_refresh(self):
        if self.server.fake_store.needs_refresh:
            self._handle_needs_refresh()
            return

    def _handle_needs_refresh(self):
        self.send_response(401)
        self.send_header('WWW-Authenticate', 'Macaroon needs_refresh=1')
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        error = {
            'code': 'macaroon-permission-required',
            'message': 'Authorization Required',
        }
        self.wfile.write(json.dumps({'error_list': [error]}).encode())

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

    def _handle_sign_build_request(self):
        logger.debug('Handling sign-build request')
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        snap_build = json.loads(string_data)['assertion']

        if snap_build == 'test-not-implemented':
            self.send_response(501)
            self.send_header('Content-Type', 'application/json')
            error = {
                'error_list': [
                    {'code': 'feature-disabled',
                     'message': ('The snap-build assertions are currently '
                                 'disabled.')},
                ],
            }
            content = json.dumps(error).encode()
        elif snap_build == 'test-invalid-data':
            self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            error = {
                'error_list': [
                    {'code': 'invalid-field',
                     'message': 'The snap-build assertion is not valid.'},
                ],
            }
            content = json.dumps(error).encode()
        elif snap_build == 'test-unexpected-data':
            self.send_response(500)
            self.send_header('Content-Type', 'text/plain')
            content = b'unexpected chunk of data'
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            response = {
                'type': 'snap-build',
                'foo': 'bar',
            }
            content = json.dumps(response).encode()
        self.end_headers()
        self.wfile.write(content)

    def _handle_close_request(self):
        logger.debug('Handling close request')
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        payload = json.loads(string_data)
        channels = payload['channels']

        if channels == ['invalid']:
            self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            error = {
                'error_list': [
                    {'code': 'invalid-field',
                     'message': ('The \'channels\' field content is not '
                                 'valid.')},
                ],
            }
            content = json.dumps(error).encode()
        elif channels == ['unexpected']:
            self.send_response(500)
            self.send_header('Content-Type', 'text/plain')
            content = b'unexpected chunk of data'
        elif channels == ['broken-plain']:
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            content = b'plain data'
        elif channels == ['broken-json']:
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            response = {
                'closed_channels': channels,
            }
            content = json.dumps(response).encode()
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            response = {
                'closed_channels': channels,
                'channel_maps': {
                    'amd64': [
                        {'channel': 'stable', 'info': 'none'},
                        {'channel': 'candidate', 'info': 'none'},
                        {'channel': 'beta', 'info': 'specific',
                         'version': '1.1', 'revision': 42},
                        {'channel': 'edge', 'info': 'tracking'}
                    ]
                },
            }
            content = json.dumps(response).encode()

        self.end_headers()
        self.wfile.write(content)

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
        snap_name = data['snap_name']

        if data['snap_name'] == 'test-already-registered-snap-name':
            self._handle_register_409('already_registered')
        elif data['snap_name'] == 'test-reserved-snap-name':
            self._handle_register_409('reserved_name')
        elif data['snap_name'].startswith('test-too-fast'):
            self._handle_register_429('register_window')
        elif data['snap_name'] == 'snap-name-no-clear-error':
            self._handle_unclear_registration_error()
        else:
            self._handle_successful_registration(snap_name)

    def _handle_successful_registration(self, name):
        self.server.registered_names.append(name)
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

    def _handle_sign_request(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)

        if 'STORE_DOWN' in os.environ:
            response_code = 500
            content_type = 'text/plain'
            response = b'Broken'
        else:
            if data['latest_tos_accepted'] is not True:
                response_code = 400
                content_type = 'application/json'
                content = {
                    "error_list": [{
                        "message": "`latest_tos_accepted` must be `true`",
                        "code": "bad-request",
                        "extra": {"latest_tos_accepted": "true"}}]}
                response = json.dumps(content).encode()
            else:
                response_code = 200
                content_type = 'application/json'
                content = {"content": {
                    "latest_tos_accepted": True,
                    "tos_url": 'http://fake-url.com',
                    "latest_tos_date": '2000-01-01',
                    "accepted_tos_date": '2010-10-10'
                    }
                }
                response = json.dumps(content).encode()

        self.send_response(response_code)
        self.send_header('Content-Type', content_type)
        self.end_headers()
        self.wfile.write(response)

    # This function's complexity is correlated to the number of
    # url paths, no point in checking that.
    def do_GET(self):  # noqa: C901
        if self.server.fake_store.needs_refresh:
            self._handle_needs_refresh()
            return
        parsed_path = urllib.parse.urlparse(self.path)
        details_good = urllib.parse.urljoin(
            self._DEV_API_PATH, '/details/upload-id/good-snap')
        details_review = urllib.parse.urljoin(
            self._DEV_API_PATH, '/details/upload-id/review-snap')
        account_path = urllib.parse.urljoin(self._DEV_API_PATH, 'account')
        snap_path = urllib.parse.urljoin(self._DEV_API_PATH, 'snaps')
        good_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/good/validations')
        no_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/snap-id/validations')
        bad_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/bad/validations')
        err_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/err/validations')

        if parsed_path.path.startswith(details_good):
            self._handle_scan_complete_request('ready_to_release', True)
        elif parsed_path.path.startswith(details_review):
            self._handle_scan_complete_request('need_manual_review', False)
        elif parsed_path.path == account_path:
            self._handle_account_request()
        elif parsed_path.path.startswith(good_validations_path):
            self._handle_validation_request('good')
        elif parsed_path.path.startswith(bad_validations_path):
            self._handle_validation_request('bad')
        elif parsed_path.path.startswith(err_validations_path):
            self._handle_validation_request('err')
        elif parsed_path.path.startswith(no_validations_path):
            self._handle_validation_request('no')
        elif parsed_path.path.startswith(snap_path):
            if parsed_path.path.endswith('/history'):
                self._handle_snap_history()
            elif parsed_path.path.endswith('/status'):
                self._handle_snap_status()
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(parsed_path)

    def _handle_validation_request(self, code):
        logger.debug('Handling validation request')
        if code == 'good':
            response = [{
                "approved-snap-id": "snap-id-1",
                "approved-snap-revision": "3",
                "approved-snap-name": "snap-1",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": True,
            }, {
                "approved-snap-id": "snap-id-2",
                "approved-snap-revision": "5",
                "approved-snap-name": "snap-2",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": False,
            }, {
                "approved-snap-id": "snap-id-3",
                "approved-snap-revision": "-",
                "approved-snap-name": "snap-3",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": True,
            }]
            response = json.dumps(response).encode()
            status = 200
        elif code == 'bad':
            response = 'foo'.encode()
            status = 200
        elif code == 'no':
            response = json.dumps([]).encode()
            status = 200
        elif code == 'err':
            status = 503
            response = {'error_list': [{'message': 'error'}]}
            response = json.dumps(response).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(response)

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
        snaps = {
            'basic': {'snap-id': 'snap-id'},
            'ubuntu-core': {'snap-id': 'good'},
            }
        snaps.update({
            n: {'snap-id': 'fake-snap-id'}
            for n in self.server.registered_names})
        self.wfile.write(json.dumps({
            'account_id': 'abcd',
            'account_keys': self.server.account_keys,
            'snaps': {'16': snaps},
        }).encode())

    def _handle_snap_history(self):
        logger.debug('Handling account request')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        revisions = [{
            'series': ['16'],
            'channels': [],
            'version': '2.0.1',
            'timestamp': '2016-09-27T19:23:40Z',
            'current_channels': ['beta', 'edge'],
            'arch': 'i386',
            'revision': 2
        }, {
            'series': ['16'],
            'channels': ['stable', 'edge'],
            'version': '2.0.2',
            'timestamp': '2016-09-27T18:38:43Z',
            'current_channels': ['stable', 'candidate', 'beta'],
            'arch': 'amd64',
            'revision': 1,
        }]

        parsed_qs = urllib.parse.parse_qs(
            urllib.parse.urlparse(self.path).query)
        if 'arch' in parsed_qs:
            output = [
                rev for rev in revisions if rev['arch'] in parsed_qs['arch']]
        else:
            output = revisions
        self.wfile.write(json.dumps(output).encode())

    def _handle_snap_status(self):
        logger.debug('Handling account request')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        channel_map = {
            'i386': [
                {
                    'info': 'none',
                    'channel': 'stable'
                },
                {
                    'info': 'none',
                    'channel': 'beta'
                },
                {
                    'info': 'specific',
                    'version': '1.0-i386',
                    'channel': 'edge',
                    'revision': 3
                },
            ],
            'amd64': [
                {
                    'info': 'specific',
                    'version': '1.0-amd64',
                    'channel': 'stable',
                    'revision': 2
                },
                {
                    'info': 'specific',
                    'version': '1.1-amd64',
                    'channel': 'beta',
                    'revision': 4
                },
                {
                    'info': 'tracking',
                    'channel': 'edge'
                },
            ],
        }

        parsed_qs = urllib.parse.parse_qs(
            urllib.parse.urlparse(self.path).query)
        if 'arch' in parsed_qs:
            arch = parsed_qs['arch'][0]
            if arch in channel_map:
                output = {arch: channel_map[arch]}
            else:
                output = {}
        else:
            output = channel_map
        self.wfile.write(json.dumps(output).encode())

    def do_PUT(self):
        if self.server.fake_store.needs_refresh:
            self._handle_needs_refresh()
            return
        parsed_path = urllib.parse.urlparse(self.path)
        good_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/good/validations')
        bad_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/bad/validations')
        err_validations_path = urllib.parse.urljoin(
            self._DEV_API_PATH, 'snaps/err/validations')

        if parsed_path.path.startswith(good_validations_path):
            self._handle_push_validation_request('good')
        elif parsed_path.path.startswith(bad_validations_path):
            self._handle_push_validation_request('bad')
        elif parsed_path.path.startswith(err_validations_path):
            self._handle_push_validation_request('err')
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(parsed_path)

    def _handle_push_validation_request(self, code):
        string_data = self.rfile.read(
            int(self.headers['Content-Length']))
        if code == 'good':
            response = string_data
            status = 200
        elif code == 'err':
            response = {'error_list': [{'message': 'error'}]}
            response = json.dumps(response).encode()
            status = 501
        elif code == 'bad':
            response = 'foo'.encode()
            status = 200
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(response)


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
            logger.error(
                'Not implemented path in fake Store Search server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_details_request(self, package):
        logger.debug(
            'Handling details request for package {}, with headers {}'.format(
                package, self.headers))
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
        if package in ('test-snap', 'ubuntu-core'):
            # sha512sum snapcraft/tests/data/test-snap.snap
            sha512 = (
                '69d57dcacf4f126592d4e6ff689ad8bb8a083c7b9fe44f6e738ef'
                'd22a956457f14146f7f067b47bd976cf0292f2993ad864ccb498b'
                'fda4128234e4c201f28fe9')
        elif package == 'test-snap-with-wrong-sha':
            sha512 = 'wrong sha'
        else:
            return None
        response = {
            'anon_download_url': urllib.parse.urljoin(
                'http://localhost:{}'.format(self.server.server_port),
                'download-snap/test-snap.snap'),
            'download_sha512': sha512,
            'snap_id': 'good',
            'developer_id': package + '-developer-id',
            'release': ['16'],
        }
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
