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

import integration_tests


class MainTestCase(integration_tests.TestCase):

    def test_main(self):
        project_dir = 'assemble'
        output = self.run_snapcraft('list-plugins', project_dir)
        expected_plugins = [
            'ant',
            'autotools',
            'catkin',
            'cmake',
            'copy',
            'go',
            'jdk',
            'kbuild',
            'kernel',
            'make',
            'maven',
            'nil',
            'nodejs',
            'plainbox-provider',
            'python2',
            'python3',
            'scons',
            'tar-content',
        ]

        self.assertIn('\n'.join(expected_plugins), output)
