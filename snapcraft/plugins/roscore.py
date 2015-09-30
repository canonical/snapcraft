# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright © 2015 Canonical Ltd
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

import snapcraft
import os


class RosCorePlugin(snapcraft.BasePlugin):

    _PLUGIN_STAGE_PACKAGES = [
    ]

    _PLUGIN_STAGE_SOURCES = ('deb http://packages.ros.org/ros/ubuntu/ trusty main\n'
                             'deb http://archive.ubuntu.com/ubuntu/ trusty main universe\n'
                             'deb http://archive.ubuntu.com/ubuntu/ trusty-updates main universe\n'
                             'deb http://archive.ubuntu.com/ubuntu/ trusty-security main universe\n')

    def __init__(self, name, options):
        self.rosversion = options.rosversion if options.rosversion else 'indigo'
        self._PLUGIN_STAGE_PACKAGES.append('ros-' + self.rosversion + '-ros-core')
        super().__init__(name, options)

    def build(self):
        os.makedirs(os.path.join(self.installdir, 'bin'), exist_ok=True)
        with open(os.path.join(self.installdir, 'bin', self.name + '-rosmaster-service'), 'w') as f:
            f.write('#!/bin/bash\n')
            f.write('_CATKIN_SETUP_DIR=' + os.path.join('$SNAP_APP_PATH', 'opt', 'ros', self.rosversion) + '\n')
            f.write('source ' + os.path.join('$SNAP_APP_PATH', 'opt', 'ros', self.rosversion, 'setup.bash') + '\n')
            f.write('exec ' + os.path.join('$SNAP_APP_PATH', 'opt', 'ros', self.rosversion, 'bin', 'rosmaster') + '\n')
        return True

    def snap_fileset (self):
        return ([
            os.path.join('bin', self.name + '-rosmaster-service'),
            '-opt/ros/' + self.rosversion + '/share/*/cmake/*',
            '-opt/ros/' + self.rosversion + '/include',
            '-opt/ros/' + self.rosversion + '/.catkin',
            '-opt/ros/' + self.rosversion + '/.rosinstall',
            '-opt/ros/' + self.rosversion + '/setup.sh',
            '-opt/ros/' + self.rosversion + '/_setup_util.py'
        ])

    def snap(self, config={}):
        if 'services' not in config.data:
            config.data['services'] = {}

        rosserv = {
            'start': os.path.join('bin', self.name + '-rosmaster-service'),
            'description': 'ROS Master service',
            'ports': {
                'internal': {
                    'rosmaster': {
                        'port': '11311/tcp',
                        'negotiable': False
                    }
                }
            }
        }

        config.data['services'][self.name + '-rosmaster'] = rosserv
        return True
