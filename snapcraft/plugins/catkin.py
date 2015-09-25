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

import os
import tempfile
import snapcraft

class CatkinPlugin (snapcraft.BasePlugin):
    _PLUGIN_STAGE_PACKAGES = [
    ]

    _PLUGIN_STAGE_SOURCES = ('deb http://packages.ros.org/ros/ubuntu/ vivid main\n'
                             'deb http://archive.ubuntu.com/ubuntu/ vivid main universe\n'
                             'deb http://archive.ubuntu.com/ubuntu/ vivid-updates main universe\n'
                             'deb http://archive.ubuntu.com/ubuntu/ vivid-security main universe\n')

    def __init__ (self, name, options):
        self.rosversion = options.rosversion or 'jade'
        self.package = options.catkin_pkg or 'jade'
        self._PLUGIN_STAGE_PACKAGES.append('ros-' + self.rosversion + '-ros-core')
        super().__init__(name, options)

    def env(self, root):
        return [
            'PYTHONPATH={0}'.format(os.path.join(self.installdir, 'usr', 'lib', self.python_version, 'dist-packages')),
            'DESTDIR={0}'.format(self.installdir),
            'CPPFLAGS="-std=c++11 $CPPFLAGS"', # ROS needs it but doesn't set it :-/
        ]

    @property
    def python_version(self):
        return self.run_output(['pyversions', '-i'])

    @property
    def rosdir(self):
        return os.path.join(self.installdir, 'opt', 'ros', self.rosversion)


    def pull(self):
        if not self.handle_source_options():
            return False

        # Look for a package definition

        return True

    def build(self):
        # Fixup ROS Cmake files that have hardcoded paths in them
        if not self.run([
            'find', self.rosdir, '-name', '*.cmake',
            '-exec', 'sed', '-i', '-e', 's|\\(\W\\)/usr/lib/|\\1{0}/usr/lib/|g'.format(self.installdir), '{}', ';'
        ]):
            return False

        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('set -ex\n')
            f.write('_CATKIN_SETUP_DIR=' + os.path.join(self.installdir, 'opt', 'ros', self.rosversion) + '\n')
            f.write('source ' + os.path.join(self.installdir, 'opt', 'ros', self.rosversion, 'setup.bash') + '\n')
            f.write('env | sort | grep FLAG\n')
            f.write('export VERBOSE=1\n')
            f.write(' '.join([
                'catkin_make',
                '--pkg', self.package,
                '--directory', self.builddir, 
                '--cmake-args',
                '-DCMAKE_C_FLAGS="$CFLAGS"',
                '-DCMAKE_CXX_FLAGS="$CPPFLAGS"',
                '-DCMAKE_LD_FLAGS="$LDFLAGS"',
                '-DCATKIN_DEVEL_PREFIX={}'.format(os.path.join(self.installdir, 'opt', 'ros', self.rosversion)),
                '-DCMAKE_INSTALL_PREFIX={}'.format(self.installdir),
                '-Dcatkin_DIR={0}'.format(os.path.join(self.installdir, 'opt', 'ros', self.rosversion, 'share', 'catkin', 'cmake')),
                '-Droscpp_DIR={0}'.format(os.path.join(self.installdir, 'opt', 'ros', self.rosversion, 'share', 'roscpp', 'cmake')),
                '-Drospy_DIR={0}'.format(os.path.join(self.installdir, 'opt', 'ros', self.rosversion, 'share', 'rospy', 'cmake')),
                '-Dgenmsg_DIR={0}'.format(os.path.join(self.installdir, 'opt', 'ros', self.rosversion, 'share', 'genmsg', 'cmake')),
                '\n'
            ]))
            f.write('catkin_make install\n')
            f.flush()

            return self.run(['/bin/bash', f.name], cwd=self.builddir)
