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

import json
import logging
import http.server
import urllib.parse


logger = logging.getLogger(__name__)


class FakeSSOServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeSSORequestHandler)


class FakeSSORequestHandler(http.server.BaseHTTPRequestHandler):

    _API_PATH = '/api/v2/'

    def log_message(*args):
        logger.debug(args)

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        oauth_path = self._API_PATH + 'tokens/oauth'
        if parsed_path.path.startswith(oauth_path):
            self._handle_oauth()
        else:
            logger.error('Not implemented path in fake SSO server: {}'.format(
                self.path))

    def _handle_oauth(self):
        string_data = self.rfile.read(
            int(self.headers['Content-Length'])).decode('utf8')
        data = json.loads(string_data)
        logger.debug(
            'Handling tokens discharge request with content {}'.format(
                data))
        if (data['password'] == 'test correct password' and
            ('otp' not in data or
             data['otp'] == 'test correct one-time password')):
            self._send_success()
        else:
            self._send_invalid_credentials_error()

    def _send_success(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'success': True,
            'consumer_key': 'test consumer key',
            'consumer_secret': 'test consumer secret',
            'token_key': 'test token key',
            'token_secret': 'test token secret'
        }
        self.wfile.write(
            json.dumps(response).encode())

    def _send_invalid_credentials_error(self):
        self.send_response(401)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {'success': False}
        self.wfile.write(
            json.dumps(response).encode())


class FakeStoreUploadServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeStoreUploadRequestHandler)


class FakeStoreUploadRequestHandler(http.server.BaseHTTPRequestHandler):

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        if parsed_path.path.startswith('/unscanned-upload/'):
            self._handle_upload_request()
        else:
            logger.error(
                'Not implemented path in fake Store Search server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_upload_request(self):
        logger.info('Handling upload request')
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.end_headers()
        response = {
            'successful': True,
            'upload_id': 'test-upload-id'
        }
        self.wfile.write(json.dumps(response).encode())


class FakeStoreAPIServer(http.server.HTTPServer):

    def __init__(self, server_address):
        super().__init__(
            server_address, FakeStoreAPIRequestHandler)


class FakeStoreAPIRequestHandler(http.server.BaseHTTPRequestHandler):

    _DEV_API_PATH = '/dev/'

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        upload_path = self._DEV_API_PATH + 'click-package-upload/'
        if parsed_path.path.startswith(upload_path):
            self._handle_upload_request()
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_upload_request(self):
        logger.debug('Handling upload request')
        self.send_response(202)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'status_url': urllib.parse.urljoin(
                'http://localhost:{}/'.format(self.server.server_port),
                'dev/api/click-scan-complete/updown/test-upload-id'),
            'success': True
        }
        self.wfile.write(json.dumps(response).encode())

    def do_GET(self):
        parsed_path = urllib.parse.urlparse(self.path)
        scan_complete_path = (
            self._DEV_API_PATH + 'api/click-scan-complete/updown/')
        if parsed_path.path.startswith(scan_complete_path):
            self._handle_scan_complete_request()
        else:
            logger.error(
                'Not implemented path in fake Store API server: {}'.format(
                    self.path))
            raise NotImplementedError(self.path)

    def _handle_scan_complete_request(self):
        logger.debug('Handling scan complete request')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {
            'message': '',
            'application_url': 'test-application-url',
            'revision': 'test-revision',
            'completed': True
        }
        self.wfile.write(json.dumps(response).encode())
