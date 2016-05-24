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
        if data['password'] == 'test correct password':
            self._send_success()
        else:
            self._send_invalid_credentials_error()

    def _send_success(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {'success': True}
        self.wfile.write(
            json.dumps(response).encode())

    def _send_invalid_credentials_error(self):
        self.send_response(401)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        response = {'success': False}
        self.wfile.write(
            json.dumps(response).encode())
