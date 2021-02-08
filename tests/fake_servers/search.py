# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2019 Canonical Ltd
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

    def configure(self, configurator):
        configurator.add_route("info", "/v2/snaps/info/{snap}", request_method="GET")
        configurator.add_view(self.info, route_name="info")

        configurator.add_route(
            "download", "/download-snap/{snap}", request_method="GET"
        )
        configurator.add_view(self.download, route_name="download")

    def info(self, request):
        snap = request.matchdict["snap"]
        logger.debug(
            "Handling details request for package {}, with headers {}".format(
                snap, request.headers
            )
        )
        if "User-Agent" not in request.headers:
            response_code = 500
            return response.Response(None, response_code)
        payload = self._get_info_payload(request)
        if payload is None:
            response_code = 404
            return response.Response(json.dumps({}).encode(), response_code)
        response_code = 200
        content_type = "application/hal+json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _get_info_payload(self, request):
        # core snap is used in integration tests with fake servers.
        snap = request.matchdict["snap"]
        # tests/data/test-snap.snap
        test_sha3_384 = (
            "8c0118831680a22090503ee5db98c88dd90ef551d80fc816"
            "dec968f60527216199dacc040cddfe5cec6870db836cb908"
        )
        revision = "10000"
        confinement = "strict"

        if snap in ("snap-1", "test-snap", "core"):
            sha3_384 = test_sha3_384
        elif snap == "snapcraft":
            sha3_384 = test_sha3_384
            revision = "25"
            confinement = "classic"
        elif snap == "test-snap-with-wrong-sha":
            sha3_384 = "wrong sha"
        elif (
            snap == "test-snap-branded-store"
            and request.headers.get("Snap-Device-Store") == "Test-Branded"
        ):
            sha3_384 = test_sha3_384
        else:
            return None

        channel_map = list()
        for arch in ("amd64", "i386", "s390x", "arm64", "armhf", "ppc64el"):
            for risk in ("stable", "edge"):
                channel_map.append(
                    {
                        "channel": {
                            "architecture": arch,
                            "name": risk,
                            "released-at": "019-01-17T15:01:26.537392+00:00",
                            "risk": risk,
                            "track": "latest",
                        },
                        "download": {
                            "deltas": [],
                            "sha3-384": sha3_384,
                            "url": urllib.parse.urljoin(
                                "http://localhost:{}".format(self.server.server_port),
                                "download-snap/test-snap.snap",
                            ),
                        },
                        "created-at": "2019-01-16T14:59:16.711111+00:00",
                        "confinement": confinement,
                        "revision": revision,
                    }
                )

        if snap == "snap-1":
            snap_id = "snap-id-1"
        else:
            snap_id = f"{snap}-snap-id"

        return json.dumps(
            {
                "channel-map": channel_map,
                "snap": {
                    "name": snap,
                    "snap-id": snap_id,
                    "publisher": {
                        "id": snap + "-developer-id",
                        "validation": "unproven",
                    },
                },
                "snap-id": snap_id,
                "name": snap,
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
