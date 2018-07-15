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


from pyramid import config

# LP: #1733579
from wsgiref import simple_server  # type: ignore


class BaseFakeServer:
    def __init__(self, server_address):
        super().__init__()
        self.server_address = server_address
        configurator = config.Configurator()
        self.configure(configurator)
        app = configurator.make_wsgi_app()
        self.server = simple_server.make_server(
            self.server_address[0], self.server_address[1], app
        )
        self.server_port = self.server.server_port
        self.socket = self.server.socket

    def configure(self, configurator):
        # To be defined in subclasses.
        raise NotImplementedError()

    def serve_forever(self):
        self.server.serve_forever()

    def shutdown(self):
        self.server.shutdown()
