# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import fixtures
import os
import socketserver
import tempfile
import threading
from unittest import mock

from tests.fake_servers import snapd


class UnixHTTPServer(socketserver.UnixStreamServer):
    def get_request(self):
        request, client_address = self.socket.accept()
        # BaseHTTPRequestHandler expects a tuple with the client address at
        # index 0, so we fake one
        if len(client_address) == 0:
            client_address = (self.server_address,)
        return (request, client_address)


class FakeSnapd(fixtures.Fixture):
    @property
    def snaps_result(self):
        self.request_handler.snaps_result

    @snaps_result.setter
    def snaps_result(self, value):
        self.request_handler.snaps_result = value

    @property
    def snap_details_func(self):
        self.request_handler.snap_details_func

    @snap_details_func.setter
    def snap_details_func(self, value):
        self.request_handler.snap_details_func = value

    @property
    def find_result(self):
        self.request_handler.find_result

    @find_result.setter
    def find_result(self, value):
        self.request_handler.find_result = value

    def __init__(self):
        super().__init__()
        self.request_handler = snapd.FakeSnapdRequestHandler
        self.snaps_result = []
        self.find_result = []
        self.snap_details_func = None

    def setUp(self):
        super().setUp()
        snapd_fake_socket_path = tempfile.mkstemp()[1]
        os.unlink(snapd_fake_socket_path)

        socket_path_patcher = mock.patch(
            "snapcraft.internal.repo.snaps.get_snapd_socket_path_template"
        )
        mock_socket_path = socket_path_patcher.start()
        mock_socket_path.return_value = "http+unix://{}/v2/{{}}".format(
            snapd_fake_socket_path.replace("/", "%2F")
        )
        self.addCleanup(socket_path_patcher.stop)

        self._start_fake_server(snapd_fake_socket_path)

    def _start_fake_server(self, socket):
        self.server = UnixHTTPServer(socket, self.request_handler)
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.start()
        self.addCleanup(self._stop_fake_server, server_thread)

    def _stop_fake_server(self, thread):
        self.server.shutdown()
        self.server.socket.close()
        thread.join()
