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

from pyramid import response

from tests.fake_servers import base


logger = logging.getLogger(__name__)


class FakeStoreUploadServer(base.BaseFakeServer):
    def configure(self, configurator):
        configurator.add_route(
            "unscanned-upload", "/unscanned-upload/", request_method="POST"
        )
        configurator.add_view(self.unscanned_upload, route_name="unscanned-upload")

    def unscanned_upload(self, request):
        logger.info("Handling upload request")
        if "UPDOWN_BROKEN" in os.environ:
            response_code = 500
            content_type = "text/plain"
            payload = b"Broken"
        else:
            response_code = 200
            content_type = "application/octet-stream"
            payload = json.dumps({"upload_id": "test-upload-id"}).encode()
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )
