# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

import os
import threading
import urllib.parse

import fixtures

from snapcraft.tests import fake_servers


class TempCWD(fixtures.TempDir):

    def setUp(self):
        """Create a temporary directory an cd into it for the test duration."""
        super().setUp()
        current_dir = os.getcwd()
        self.addCleanup(os.chdir, current_dir)
        os.chdir(self.path)


class FakeStore(fixtures.Fixture):

    def setUp(self):
        super().setUp()

        self.fake_sso_server_fixture = FakeSSOServerRunning()
        self.useFixture(self.fake_sso_server_fixture)
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL',
            urllib.parse.urljoin(
                self.fake_sso_server_fixture.url, 'api/v2')))


class _FakeServerRunning(fixtures.Fixture):

    # To be defined by child fixtures.
    fake_server = None

    def setUp(self):
        super().setUp()
        self._start_fake_server()

    def _start_fake_server(self):
        server_address = ('', 0)
        self.server = self.fake_server(server_address)
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.start()
        self.addCleanup(self._stop_fake_server, server_thread)
        self.url = 'http://localhost:{}/'.format(self.server.server_port)

    def _stop_fake_server(self, thread):
        self.server.shutdown()
        self.server.socket.close()
        thread.join()


class FakeSSOServerRunning(_FakeServerRunning):

    fake_server = fake_servers.FakeSSOServer


class StagingStore(fixtures.Fixture):

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_API_ROOT_URL',
            'https://myapps.developer.staging.ubuntu.com/dev/api/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_UPLOAD_ROOT_URL',
            'https://upload.apps.staging.ubuntu.com/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL',
            'https://login.staging.ubuntu.com/api/v2/'))
