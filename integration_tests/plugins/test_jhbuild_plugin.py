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

import http.client
import http.server
import glob
import os
import threading
import integration_tests


class StoppableHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):

    def do_QUIT(self):
        self.send_response(200)
        self.end_headers()
        self.server.stop = True


class StoppableHTTPServer(http.server.HTTPServer):

    def __init__(self, socket, handler):
        self.stop = False
        super(StoppableHTTPServer, self).__init__(socket, handler)

    def serve_forever(self):
        while not self.stop:
            self.handle_request()


class JHBuildPluginTestCase(integration_tests.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.port = 8000
        handler = StoppableHTTPRequestHandler
        httpd = StoppableHTTPServer(("127.0.0.1", cls.port), handler)
        cls.server = threading.Thread(target=httpd.serve_forever)
        cls.server.setDaemon(True)
        cls.server.start()

    @classmethod
    def tearDownClass(cls):
        conn = http.client.HTTPConnection("127.0.0.1:%d" % cls.port)
        conn.request("QUIT", "/")
        conn.getresponse()
        cls.server.join()

    def test_snap(self):

        files = {
            'pull': [os.path.join(self.parts_dir, 'jhbuild', 'jhbuildrc')],
            'build': [
                os.path.join(self.parts_dir, 'jhbuild', 'jhbuildrc'),
                os.path.join(self.parts_dir, 'jhbuild', 'install',
                             'usr', 'share', 'doc', 'dep-module', 'README'),
                os.path.join(self.parts_dir, 'jhbuild', 'install',
                             'usr', 'share', 'doc', 'main-module', 'README'),
            ],
            'stage': [
                os.path.join(self.stage_dir, 'usr', 'share',
                             'doc', 'dep-module', 'README'),
                os.path.join(self.stage_dir, 'usr', 'share',
                             'doc', 'main-module', 'README'),
            ],
            'prime': [
                os.path.join(self.prime_dir, 'usr', 'share',
                             'doc', 'dep-module', 'README'),
                os.path.join(self.prime_dir, 'usr', 'share',
                             'doc', 'main-module', 'README'),
                os.path.join(self.prime_dir, 'command-dep.wrapper'),
                os.path.join(self.prime_dir, 'command-main.wrapper'),
            ],
            'snap': [],
        }

        for stage in ['pull', 'build', 'stage', 'prime', 'snap']:
            self.run_snapcraft(stage, 'jhbuild')

            for path in files[stage]:
                self.assertTrue(os.path.exists(path),
                                "path '%s' does not exist" % path)

        self.assertNotEqual([], glob.glob('test-jhbuild_*.snap'))
