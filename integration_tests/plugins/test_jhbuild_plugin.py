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

import os
import integration_tests


class JHBuildPluginTestCase(integration_tests.TestCase):

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
                self.assertIs(True, os.path.exists(path),
                              "path '%s' does not exist" % path)

        self.assertNotEqual([], [
            snap
            for snap in os.listdir(os.getcwd())
            if snap.startswith('test-jhbuild_') and snap.endswith('.snap')
        ])

        conn = http.client.HTTPConnection("127.0.0.1:%d" % PORT)
        conn.request("QUIT", "/")
        conn.getresponse()
        time.sleep(2)
