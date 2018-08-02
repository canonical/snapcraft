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


logger = logging.getLogger(__name__)


class BaseHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    def log_message(*args):
        logger.debug(args)

    def raise_not_implemented(self, path):
        logger.error(
            "Not implemented {} in {} server: {}".format(path, self.__class__.__name__)
        )
        raise NotImplementedError(path)


class FakeFileHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith("404-not-found"):
            self.send_response(404)
            self.end_headers()
        else:
            data = "Test fake file"
            self.send_response(200)
            self.send_header("Content-Length", len(data))
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(data.encode())


class FakePartsServer(http.server.HTTPServer):
    def __init__(self, server_address):
        super().__init__(server_address, FakePartsRequestHandler)


class FakePartsRequestHandler(BaseHTTPRequestHandler):

    _date_format = "%a, %d %b %Y %H:%M:%S GMT"
    _parts_date = datetime(2016, 7, 7, 10, 0, 20)

    def do_GET(self):
        logger.debug("Handling getting parts")
        if "If-Modified-Since" in self.headers:
            ims_date = datetime.strptime(
                self.headers["If-Modified-Since"], self._date_format
            )
        else:
            ims_date = None
        if ims_date is not None and ims_date >= self._parts_date:
            self.send_response(304)
            response = {}
        elif "CUSTOM_PARTS" in os.environ:
            self.send_response(200)
            response = OrderedDict(
                (
                    (
                        "curl-custom",
                        OrderedDict(
                            (
                                ("plugin", "autotools"),
                                ("source", "http://curl.org"),
                                ("description", "custom curl part"),
                                ("maintainer", "none"),
                            )
                        ),
                    ),
                )
            )
        else:
            self.send_response(200)
            response = OrderedDict(
                (
                    (
                        "curl",
                        OrderedDict(
                            (
                                ("plugin", "autotools"),
                                ("source", "http://curl.org"),
                                ("description", "test entry for curl"),
                                ("maintainer", "none"),
                            )
                        ),
                    ),
                    (
                        "part1",
                        OrderedDict(
                            (
                                ("plugin", "go"),
                                ("source", "http://source.tar.gz"),
                                ("description", "test entry for part1"),
                                ("maintainer", "none"),
                            )
                        ),
                    ),
                    (
                        "long-described-part",
                        OrderedDict(
                            (
                                ("plugin", "go"),
                                ("source", "http://source.tar.gz"),
                                (
                                    "description",
                                    "this is a repetitive description " * 3,
                                ),
                                ("maintainer", "none"),
                            )
                        ),
                    ),
                    (
                        "multiline-part",
                        OrderedDict(
                            (
                                ("plugin", "go"),
                                ("source", "http://source.tar.gz"),
                                (
                                    "description",
                                    "this is a multiline description\n" * 3,
                                ),
                                ("maintainer", "none"),
                            )
                        ),
                    ),
                )
            )
        self.send_header("Content-Type", "text/plain")
        if "NO_CONTENT_LENGTH" not in os.environ:
            self.send_header("Content-Length", "1000")
        self.send_header("Last-Modified", self._parts_date.strftime(self._date_format))
        self.send_header("ETag", "1111")
        self.end_headers()
        self.wfile.write(yaml.dump(response).encode())


class FakePartsWikiServer(http.server.HTTPServer):
    def __init__(self, server_address):
        super().__init__(server_address, FakePartsWikiRequestHandler)


class FakePartsWikiRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        logger.debug("Handling getting parts")
        self.send_response(200)
        response = """
---
origin: https://github.com/sergiusens/curl.git
parts: [curl]
description:
  Description here
maintainer: none
"""
        self.send_header("Content-Type", "text/plain")
        if "NO_CONTENT_LENGTH" not in os.environ:
            self.send_header("Content-Length", len(response.encode()))
        self.end_headers()
        self.wfile.write(response.encode())


class FakePartsWikiWithSlashesServer(http.server.HTTPServer):
    def __init__(self, server_address):
        super().__init__(server_address, FakePartsWikiWithSlashesRequestHandler)


class FakePartsWikiWithSlashesRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        logger.debug("Handling getting parts")
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
        self.send_header("Content-Type", "text/plain")
        if "NO_CONTENT_LENGTH" not in os.environ:
            self.send_header("Content-Length", len(response.encode()))
        self.end_headers()
        self.wfile.write(response.encode())


class FakePartsWikiOriginServer(http.server.HTTPServer):
    def __init__(self, server_address):
        super().__init__(server_address, FakePartsWikiOriginRequestHandler)


class FakePartsWikiOriginRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        logger.debug("Handling getting part origin")
        self.send_response(200)
        response = """
parts:
  somepart:
    source: https://github.com/someuser/somepart.git
    plugin: nil
"""
        self.send_header("Content-Type", "text/plain")
        if "NO_CONTENT_LENGTH" not in os.environ:
            self.send_header("Content-Length", len(response.encode()))
        self.end_headers()
        self.wfile.write(response.encode())


class FakeSSOServer(http.server.HTTPServer):
    def __init__(self, fake_store, server_address):
        super().__init__(server_address, FakeSSORequestHandler)
        self.fake_store = fake_store


class FakeSSORequestHandler(BaseHTTPRequestHandler):

    _API_PATH = "/api/v2/"

    def do_POST(self):
        parsed_path = urllib.parse.urlparse(self.path)
        tokens_discharge_path = urllib.parse.urljoin(self._API_PATH, "tokens/discharge")
        tokens_refresh_path = urllib.parse.urljoin(self._API_PATH, "tokens/refresh")
        if parsed_path.path.startswith(tokens_discharge_path):
            self._handle_tokens_discharge_request()
        elif parsed_path.path.startswith(tokens_refresh_path):
            self._handle_tokens_refresh_request()
        else:
            self.raise_not_implemented(self.path)

    def _handle_tokens_discharge_request(self):
        string_data = self.rfile.read(int(self.headers["Content-Length"])).decode(
            "utf8"
        )
        data = json.loads(string_data)
        logger.debug("Handling tokens discharge request with content {}".format(data))
        if data["password"] == "test correct password" and (
            "otp" not in data or data["otp"] == "test correct one-time password"
        ):
            self._send_tokens_discharge()
        elif data["password"] == "test requires 2fa":
            self._send_twofactor_required_error()
        elif data["password"] == "test 401 invalid json":
            self._send_401_invalid_json()
        else:
            self._send_invalid_credentials_error()

    def _send_tokens_discharge(self):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        response = {"discharge_macaroon": pymacaroons.Macaroon().serialize()}
        self.wfile.write(json.dumps(response).encode())

    def _send_twofactor_required_error(self):
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        response = {
            "error_list": [
                {
                    "code": "twofactor-required",
                    "message": "2-factor authentication required.",
                }
            ]
        }
        self.wfile.write(json.dumps(response).encode())

    def _send_401_invalid_json(self):
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b"invalid{json")

    def _send_invalid_credentials_error(self):
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        response = {
            "error_list": [
                {
                    "code": "invalid-credentials",
                    "message": "Provided email/password is not correct.",
                }
            ]
        }
        self.wfile.write(json.dumps(response).encode())

    def _handle_tokens_refresh_request(self):
        self._send_tokens_discharge()
        self.server.fake_store.needs_refresh = False
