# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017-2018 Canonical Ltd
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
import os
import urllib.parse

from pyramid import response

import tests
from tests.fake_servers import base


logger = logging.getLogger(__name__)


class FakeStoreSearchServer(base.BaseFakeServer):

    # XXX This fake server as reused as download server, to avoid passing a
    # port as an argument. --elopio - 2016-05-01

    _API_PATH = "/api/v1/"

    def configure(self, configurator):
        configurator.add_route(
            "details",
            urllib.parse.urljoin(self._API_PATH, "snaps/details/{snap}"),
            request_method="GET",
        )
        configurator.add_view(self.details, route_name="details")

        configurator.add_route(
            "download", "/download-snap/{snap}", request_method="GET"
        )
        configurator.add_view(self.download, route_name="download")

    def details(self, request):
        snap = request.matchdict["snap"]
        logger.debug(
            "Handling details request for package {}, with headers {}".format(
                snap, request.headers
            )
        )
        if "User-Agent" not in request.headers:
            response_code = 500
            return response.Response(None, response_code)
        payload = self._get_details_payload(request)
        if payload is None:
            response_code = 404
            return response.Response(json.dumps({}).encode(), response_code)
        response_code = 200
        content_type = "application/hal+json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _get_details_payload(self, request):
        # core snap is used in integration tests with fake servers.
        snap = request.matchdict["snap"]
        # sha512sum tests/data/test-snap.snap
        test_sha512 = (
            "69d57dcacf4f126592d4e6ff689ad8bb8a083c7b9fe44f6e738ef"
            "d22a956457f14146f7f067b47bd976cf0292f2993ad864ccb498b"
            "fda4128234e4c201f28fe9"
        )
        revision = "10000"
        confinement = "strict"

        if snap in ("test-snap", "core"):
            sha512 = test_sha512
        elif snap == "snapcraft":
            sha512 = test_sha512
            revision = "25"
            confinement = "classic"
        elif snap == "test-snap-with-wrong-sha":
            sha512 = "wrong sha"
        elif snap == "test-snap-branded-store":
            # Branded-store snaps require Store pinning and authorization.
            if (
                request.headers.get("X-Ubuntu-Store") != "Test-Branded"
                or request.headers.get("Authorization") is None
            ):
                return None
            sha512 = test_sha512
        else:
            return None

        return json.dumps(
            {
                "anon_download_url": urllib.parse.urljoin(
                    "http://localhost:{}".format(self.server.server_port),
                    "download-snap/test-snap.snap",
                ),
                "download_sha3_384": "1234567890",
                "download_sha512": sha512,
                "snap_id": "good",
                "developer_id": snap + "-developer-id",
                "release": ["16"],
                "revision": revision,
                "confinement": confinement,
            }
        ).encode()

    def download(self, request):
        snap = request.matchdict["snap"]
        logger.debug("Handling download request for snap {}".format(snap))
        if "User-Agent" not in request.headers:
            response_code = 500
            return response.Response(None, response_code)
        response_code = 200
        content_type = "application/octet-stream"
        # TODO create a test snap during the test instead of hardcoding it.
        # --elopio - 2016-05-01
        snap_path = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap.snap"
        )

        with open(snap_path, "rb") as snap_file:
            return response.Response(
                snap_file.read(), response_code, [("Content-Type", content_type)]
            )
