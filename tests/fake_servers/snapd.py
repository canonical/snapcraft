# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
from urllib import parse
from typing import Any, Dict, List  # noqa

from tests import fake_servers


class FakeSnapdRequestHandler(fake_servers.BaseHTTPRequestHandler):

    snaps_result = []  # type: List[Dict[str, Any]]
    snap_details_func = None
    find_result = []  # type: List[Dict[str, Any]]
    find_exit_code = 200  # type: int
    _private_data = {"new_fake_snap_installed": False}

    def do_GET(self):
        parsed_url = parse.urlparse(self.path)
        if parsed_url.path == "/v2/snaps":
            self._handle_snaps()
        elif parsed_url.path.startswith("/v2/snaps/"):
            self._handle_snap_details(parsed_url)
        elif parsed_url.path == "/v2/find":
            self._handle_find(parsed_url)
        else:
            self.wfile.write(parsed_url.path.encode())

    def _handle_snaps(self):
        status_code = self.find_exit_code
        params = self.snaps_result
        self.send_response(status_code)
        self.send_header("Content-Type", "text/application+json")
        self.end_headers()
        response = json.dumps({"result": params}).encode()
        self.wfile.write(response)

    def _handle_snap_details(self, parsed_url):
        status_code = 404
        params = {"message": "not found"}
        type_ = "error"
        snap_name = parsed_url.path.split("/")[-1]
        if self.snap_details_func:
            status_code, params = self.snap_details_func(snap_name)
        else:
            for snap in self.snaps_result:
                if snap["name"] == snap_name:
                    status_code = 200
                    type_ = "sync"
                    params = {}
                    for key in ("channel", "revision", "confinement", "id"):
                        if key in snap:
                            params.update({key: snap[key]})
                    break

        self.send_response(status_code)
        self.send_header("Content-Type", "text/application+json")
        self.end_headers()
        response = json.dumps({"result": params, "type": type_}).encode()

        self.wfile.write(response)

    def _handle_find(self, parsed_url):
        query = parse.parse_qs(parsed_url.query)
        snap_name = query["name"][0]
        status_code = 404
        params = {}
        for result in self.find_result:
            if snap_name in result:
                status_code = 200
                params = result[snap_name]
                break
        if snap_name == "new-fake-snap":
            status_code = 200
            params = {"channels": {"latest/stable": {"confinement": "strict"}}}

        self.send_response(status_code)
        self.send_header("Content-Type", "text/application+json")
        self.end_headers()
        response = json.dumps({"result": [params]}).encode()
        self.wfile.write(response)
