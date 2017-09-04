# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import os
import socketserver
import subprocess
import tempfile
from http.server import BaseHTTPRequestHandler
from threading import Thread
from unittest import mock
from urllib import parse

import fixtures
from testtools.matchers import Equals, Is

from snapcraft import tests
from snapcraft.internal.repo import errors, snaps


class FakeSnapCommand(fixtures.Fixture):

    def __init__(self):
        self.calls = []
        self.install_success = True
        self.refresh_success = True
        self._email = '-'

    def _setUp(self):
        original_check_call = snaps.check_call
        original_check_output = snaps.check_output

        def side_effect_check_call(cmd, *args, **kwargs):
            return side_effect(original_check_call, cmd, *args, **kwargs)

        def side_effect_check_output(cmd, *args, **kwargs):
            return side_effect(original_check_output, cmd, *args, **kwargs)

        def side_effect(original, cmd, *args, **kwargs):
            if self._is_snap_command(cmd):
                self.calls.append(cmd)
                return self._fake_snap_command(cmd, *args, **kwargs)
            else:
                return original(cmd, *args, **kwargs)

        check_call_patcher = mock.patch(
            'snapcraft.internal.repo.snaps.check_call',
            side_effect=side_effect_check_call)
        self.mock_call = check_call_patcher.start()
        self.addCleanup(check_call_patcher.stop)

        check_output_patcher = mock.patch(
            'snapcraft.internal.repo.snaps.check_output',
            side_effect=side_effect_check_output)
        self.mock_call = check_output_patcher.start()
        self.addCleanup(check_output_patcher.stop)

    def login(self, email):
        self._email = email

    def _get_snap_cmd(self, snap_cmd):
        try:
            snap_cmd_index = snap_cmd.index('snap')
        except ValueError:
            return ''

        try:
            return snap_cmd[snap_cmd_index+1]
        except IndexError:
            return ''

    def _is_snap_command(self, cmd):
        return self._get_snap_cmd(cmd) in ['install', 'refresh', 'whoami']

    def _fake_snap_command(self, cmd, *args, **kwargs):
        cmd = self._get_snap_cmd(cmd)
        if cmd == 'install' and not self.install_success:
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)
        elif cmd == 'refresh' and not self.refresh_success:
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)
        elif cmd == 'whoami':
            return 'email: {}'.format(self._email).encode()


class FakeSnapdServer(BaseHTTPRequestHandler):

    def do_GET(self):
        parsed_url = parse.urlparse(self.path)
        if parsed_url.path.startswith('/v2/snaps/'):
            self._handle_snaps(parsed_url)
        elif parsed_url.path == '/v2/find':
            self._handle_find(parsed_url)
        else:
            self.wfile.write(parsed_url.path.encode())

    def _handle_snaps(self, parsed_url):
        status_code = 404
        params = {}

        if parsed_url.path.endswith('/fake-snap'):
            status_code = 200
            params = {'channel': 'stable'}
        elif parsed_url.path.endswith('/fake-snap-stable'):
            status_code = 200
            params = {'channel': 'stable'}
        elif parsed_url.path.endswith('/fake-snap-branch'):
            status_code = 200
            params = {'channel': 'candidate/branch'}
        elif parsed_url.path.endswith('/fake-snap-track-stable'):
            status_code = 200
            params = {'channel': 'track/stable'}
        elif parsed_url.path.endswith('/fake-snap-track-stable-branch'):
            status_code = 200
            params = {'channel': 'track/stable/branch'}
        elif parsed_url.path.endswith('/fake-snap-edge'):
            status_code = 200
            params = {'channel': 'edge'}

        self.send_response(status_code)
        self.send_header('Content-Type', 'text/application+json')
        self.end_headers()
        response = json.dumps({'result': params}).encode()
        self.wfile.write(response)

    def _handle_find(self, parsed_url):
        query = parse.parse_qs(parsed_url.query)
        status_code = 404
        params = {}

        if query['name'][0] == 'fake-snap':
            status_code = 200
            params = {'channels': {
                'latest/stable': {'confinement': 'strict'},
                'classic/stable': {'confinement': 'classic'},
                'strict/stable': {'confinement': 'strict'},
                'devmode/stable': {'confinement': 'devmode'},
            }}
        if query['name'][0] == 'new-fake-snap':
            status_code = 200
            params = {'channels': {
                'latest/stable': {'confinement': 'strict'},
            }}

        self.send_response(status_code)
        self.send_header('Content-Type', 'text/application+json')
        self.end_headers()
        response = json.dumps({'result': [params]}).encode()
        self.wfile.write(response)


class UnixHTTPServer(socketserver.UnixStreamServer):

    def get_request(self):
        request, client_address = self.socket.accept()
        # BaseHTTPRequestHandler expects a tuple with the client address at
        # index 0, so we fake one
        if len(client_address) == 0:
            client_address = (self.server_address,)
        return (request, client_address)


class FakeSnapd(fixtures.Fixture):

    def _setUp(self):
        snapd_fake_socket_path = tempfile.mkstemp()[1]
        os.unlink(snapd_fake_socket_path)

        socket_path_patcher = mock.patch(
            'snapcraft.internal.repo.snaps.get_snapd_socket_path_template')
        mock_socket_path = socket_path_patcher.start()
        mock_socket_path.return_value = 'http+unix://{}/v2/{{}}'.format(
            snapd_fake_socket_path.replace('/', '%2F'))
        self.addCleanup(socket_path_patcher.stop)

        self._start_fake_server(snapd_fake_socket_path)

    def _start_fake_server(self, socket):
        self.server = UnixHTTPServer(socket, FakeSnapdServer)
        server_thread = Thread(target=self.server.serve_forever)
        server_thread.start()
        self.addCleanup(self._stop_fake_server, server_thread)

    def _stop_fake_server(self, thread):
        self.server.shutdown()
        self.server.socket.close()
        thread.join()


class SnapPackageBaseTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.useFixture(FakeSnapd())


class SnapPackageCurrentChannelTest(SnapPackageBaseTestCase):

    scenarios = [
        ('stable',
         dict(snap='fake-snap-stable/stable',
              expected='latest/stable')),
        ('latest/stable',
         dict(snap='fake-snap-stable/latest/stable',
              expected='latest/stable')),
        ('candidate/branch',
         dict(snap='fake-snap-branch/candidate/branch',
              expected='latest/candidate/branch')),
        ('track/stable/branch',
         dict(snap='fake-snap-track-stable-branch/track/stable/branch',
              expected='track/stable/branch')),
        ('edge',
         dict(snap='fake-snap-edge/stable',
              expected='latest/edge')),
        ('track/stable',
         dict(snap='fake-snap-track-stable/track/stable',
              expected='track/stable')),
    ]

    def test_get_current_channel(self):
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.get_current_channel(),
                        Equals(self.expected))


class SnapPackageIsInstalledTest(SnapPackageBaseTestCase):

    scenarios = [
        ('installed stable',
         dict(snap='fake-snap-stable',
              expected=True)),
        ('installed stable with channel',
         dict(snap='fake-snap-stable/latest/stable',
              expected=True)),
        ('not installed',
         dict(snap='missing-snap',
              expected=False)),
        ('not installed with channel',
         dict(snap='missing-snap/latest/stable',
              expected=False)),
    ]

    def test_is_installed(self):
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.installed, Is(self.expected))

    def test_is_installed_classmethod(self):
        self.assertThat(snaps.SnapPackage.is_snap_installed(self.snap),
                        Is(self.expected))


class SnapPackageIsInStoreTest(SnapPackageBaseTestCase):

    scenarios = [
        ('in store',
         dict(snap='fake-snap',
              expected=True)),
        ('in store with channel',
         dict(snap='fake-snap/latest/stable',
              expected=True)),
        ('not in store',
         dict(snap='missing-snap',
              expected=False)),
        ('not in store with channel',
         dict(snap='missing-snap/latest/stable',
              expected=False)),
    ]

    def test_is_in_store(self):
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.in_store, Is(self.expected))


class SnapPackageIsClassicTest(SnapPackageBaseTestCase):

    scenarios = [
        ('classic', dict(snap='fake-snap/classic/stable', expected=True)),
        ('strict', dict(snap='fake-snap/strict/stable', expected=False)),
        ('devmode', dict(snap='fake-snap/devmode/stable', expected=False)),
    ]

    def test_is_classic(self):
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.is_classic(), Is(self.expected))


class SnapPackageIsValidTest(SnapPackageBaseTestCase):

    scenarios = [
        ('valid',
         dict(snap='fake-snap', expected=True)),
        ('valid with channel',
         dict(snap='fake-snap/strict/stable', expected=True)),
        ('invalid',
         dict(snap='missing-snap', expected=False)),
        ('invalid with channel',
         dict(snap='missing-snap/strict/stable', expected=False)),

    ]

    def test_is_valid(self):
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.is_valid(), Is(self.expected))

    def test_is_valid_classmethod(self):
        self.assertThat(snaps.SnapPackage.is_valid_snap(self.snap),
                        Is(self.expected))


class SnapPackageLifecycleTest(SnapPackageBaseTestCase):

    def setUp(self):
        super().setUp()
        self.fake_snap_command = FakeSnapCommand()
        self.useFixture(self.fake_snap_command)

    def test_install_classic(self):
        snap_pkg = snaps.SnapPackage('fake-snap/classic/stable')
        snap_pkg.install()
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['sudo', 'snap', 'install', 'fake-snap',
             '--channel', 'classic/stable', '--classic']]))

    def test_install_non_classic(self):
        snap_pkg = snaps.SnapPackage('fake-snap/strict/stable')
        snap_pkg.install()
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['sudo', 'snap', 'install', 'fake-snap',
             '--channel', 'strict/stable']]))

    def test_install_logged_in(self):
        self.fake_snap_command.login('user@email.com')
        snap_pkg = snaps.SnapPackage('fake-snap/strict/stable')
        snap_pkg.install()
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['snap', 'install', 'fake-snap',
             '--channel', 'strict/stable']]))

    def test_install_fails(self):
        self.fake_snap_command.install_success = False
        snap_pkg = snaps.SnapPackage('fake-snap/strict/stable')
        self.assertRaises(errors.SnapInstallError, snap_pkg.install)

    def test_refresh(self):
        snap_pkg = snaps.SnapPackage('fake-snap/strict/stable')
        snap_pkg.refresh()
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['sudo', 'snap', 'refresh', 'fake-snap',
             '--channel', 'strict/stable']]))

    def test_refresh_to_classic(self):
        snap_pkg = snaps.SnapPackage('fake-snap/classic/stable')
        snap_pkg.refresh()
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['sudo', 'snap', 'refresh', 'fake-snap',
             '--channel', 'classic/stable', '--classic']]))

    def test_refresh_logged_in(self):
        self.fake_snap_command.login('user@email.com')
        snap_pkg = snaps.SnapPackage('fake-snap/strict/stable')
        snap_pkg.refresh()
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['snap', 'refresh', 'fake-snap',
             '--channel', 'strict/stable']]))

    def test_refresh_fails(self):
        snap_pkg = snaps.SnapPackage('fake-snap/strict/stable')
        self.fake_snap_command.refresh_success = False
        self.assertRaises(errors.SnapRefreshError, snap_pkg.refresh)

    def test_install_multiple_snaps(self):
        snaps.install_snaps([
            'fake-snap/classic/stable',
            'new-fake-snap'
        ])
        self.assertThat(self.fake_snap_command.calls, Equals([
            ['snap', 'whoami'],
            ['sudo', 'snap', 'refresh', 'fake-snap',
             '--channel', 'classic/stable', '--classic'],
            ['snap', 'whoami'],
            ['sudo', 'snap', 'install', 'new-fake-snap']]))
