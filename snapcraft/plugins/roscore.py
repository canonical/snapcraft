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

    _PLUGIN_STAGE_SOURCES = '''
deb http://packages.ros.org/ros/ubuntu/ trusty main
deb http://${prefix}.ubuntu.com/${suffix}/ trusty main universe
deb http://${prefix}.ubuntu.com/${suffix}/ trusty-updates main universe
deb http://${prefix}.ubuntu.com/${suffix}/ trusty-security main universe
deb http://${security}.ubuntu.com/${suffix} trusty-security main universe
'''

    @classmethod
    def schema(cls):
        return {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'type': 'object',
            'properties': {
                'rosversion': {
                    'type': 'string',
                    'default': 'indigo'
                },
            }
        }

    def __init__(self, name, options):
        super().__init__(name, options)
        self.stage_packages.append(
            'ros-' + self.options.rosversion + '-ros-core'
        )

    def build(self):
        os.makedirs(os.path.join(self.installdir, 'bin'), exist_ok=True)
        ros_dir = os.path.join(
            '$SNAP_APP_PATH',
            'opt',
            'ros',
            self.options.rosversion)
        service_wrapper = os.path.join(
            self.installdir, 'bin', '{}-rosmaster-service'.format(self.name))

        with open(os.path.join(service_wrapper), 'w') as f:
            f.write('#!/bin/bash\n')
            f.write('_CATKIN_SETUP_DIR={}\n'.format(ros_dir))
            f.write('source {}/setup.bash\n'.format(ros_dir))
            f.write('export PYTHONPATH={}:$PYTHONPATH\n'.format(
                os.path.join(ros_dir, 'lib', 'python2.7', 'dist-packages')))
            f.write('exec {}/bin/rosmaster\n'.format(ros_dir))

        os.chmod(service_wrapper, 0o755)

    def snap_fileset(self):
        rospath = os.path.join('opt', 'ros', self.options.rosversion)
        return ([
            os.path.join('bin', self.name + '-rosmaster-service'),
            os.path.join(rospath, 'bin', '*'),
            os.path.join(rospath, 'lib', '*'),
            '-' + os.path.join(rospath, 'share', '*', 'cmake', '*'),
            '-' + os.path.join(rospath, 'include'),
            '-' + os.path.join(rospath, '.catkin'),
            '-' + os.path.join(rospath, '.rosinstall'),
            '-' + os.path.join(rospath, 'setup.sh'),
            '-' + os.path.join(rospath, '_setup_util.py')
        ])
