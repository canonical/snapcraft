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
import platform
import integration_tests


class JHBuildPluginTestCase(integration_tests.TestCase):

    def test_snap(self):
        project_dir = 'simple-jhbuild'

        if platform.linux_distribution()[2] == 'xenial':
            self.skipTest('This test requires bubblewrap')
        else:
            stages = [
                'pull',
                'build',
                'stage',
                'prime',
                'snap',
            ]

            files = {
                'pull': [
                    os.path.join(
                        'parts',
                        'jhbuild',
                        'src',
                        'jhbuildrc',
                    ),
                    os.path.join(
                        'parts',
                        'jhbuild',
                        'src',
                        '.cache',
                        'build-packages',
                    ),
                    os.path.join(
                        'parts',
                        'jhbuild',
                        'src',
                        '.cache',
                        'stage-packages',
                    ),
                ],
                'build': [
                    os.path.join(
                        'parts',
                        'jhbuild',
                        'install',
                        'etc',
                        'jhbuildrc',
                    ),
                    os.path.join(
                        'parts',
                        'jhbuild',
                        'install',
                        'share',
                        'doc',
                        'dep-module',
                        'README',
                    ),
                    os.path.join(
                        'parts',
                        'jhbuild',
                        'install',
                        'share',
                        'doc',
                        'main-module',
                        'README',
                    ),
                ],
                'stage': [
                    os.path.join(
                        'stage',
                        'etc',
                        'jhbuildrc',
                    ),
                    os.path.join(
                        'stage',
                        'share',
                        'doc',
                        'dep-module',
                        'README',
                    ),
                    os.path.join(
                        'stage',
                        'share',
                        'doc',
                        'main-module',
                        'README',
                    ),
                ],
                'prime': [
                    os.path.join(
                        'prime',
                        'etc',
                        'jhbuildrc',
                    ),
                    os.path.join(
                        'prime',
                        'share',
                        'doc',
                        'dep-module',
                        'README',
                    ),
                    os.path.join(
                        'prime',
                        'share',
                        'doc',
                        'main-module',
                        'README',
                    ),
                    os.path.join(
                        'prime',
                        'command-dep.wrapper',
                    ),
                    os.path.join(
                        'prime',
                        'command-main.wrapper',
                    ),
                ],
                'snap': [
                ],
            }

            for stage in stages:
                self.run_snapcraft([stage], project_dir=project_dir)

                for path in files[stage]:
                    path = os.path.join(self.path, project_dir, path)
                    self.assertIs(True, os.path.exists(path) or path)

            self.assertNotEqual([], [
                snap
                for snap in os.listdir(os.path.join(self.path, project_dir))
                if snap.startswith('test-jhbuild_') and snap.endswith('.snap')
            ])
