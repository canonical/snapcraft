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
        'gcc',
        'g++',
        'libstdc++-4.8-dev',
    ]

    _PLUGIN_STAGE_SOURCES = ('deb http://packages.ros.org/ros/ubuntu/ trusty main\n'
                             'deb http://archive.ubuntu.com/ubuntu/ trusty main universe\n'
                             'deb http://archive.ubuntu.com/ubuntu/ trusty-updates main universe\n'
                             'deb http://archive.ubuntu.com/ubuntu/ trusty-security main universe\n')

    def __init__(self, name, options):
        self.rosversion = options.rosversion or 'jade'
        self.packages = options.catkin_packages
        self.dependencies = []
        self._PLUGIN_STAGE_PACKAGES.extend(['ros-' + self.rosversion + '-ros-core', ])
        super().__init__(name, options)

    def env(self, root):
        return [
            'PYTHONPATH={0}'.format(os.path.join(self.installdir, 'usr', 'lib', self.python_version, 'dist-packages')),
            'DESTDIR={0}'.format(self.installdir),
            # ROS needs it but doesn't set it :-/
            'CPPFLAGS="-std=c++11 $CPPFLAGS -I{0} -I{1}"'.format(
                os.path.join(root, 'usr', 'include', 'c++', '4.8'),
                os.path.join(root, 'usr', 'include',
                             snapcraft.common.get_arch_triplet(), 'c++', '4.8')),
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

    def rosrun(self, commandlist, cwd=None):
        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('set -ex\n')
            f.write('_CATKIN_SETUP_DIR=' + os.path.join(self.installdir, 'opt', 'ros', self.rosversion) + '\n')
            f.write('source ' + os.path.join(self.installdir, 'opt', 'ros', self.rosversion, 'setup.bash') + '\n')
            f.write('exec {}\n'.format(' '.join(commandlist)))
            f.flush()

            return self.run(['/bin/bash', f.name], cwd=cwd)

    def build(self):
        # Fixup ROS Cmake files that have hardcoded paths in them
        if not self.run([
            'find', self.rosdir, '-name', '*.cmake',
            '-exec', 'sed', '-i', '-e', 's|\\(\W\\)/usr/lib/|\\1{0}/usr/lib/|g'.format(self.installdir), '{}', ';'
        ]):
            return False

        catkincmd = ['catkin_make']

        for pkg in self.packages:
            catkincmd.append('--pkg')
            catkincmd.append(pkg)

        # Define the location
        catkincmd.extend(['--directory', self.builddir])

        # Start the CMake Commands
        catkincmd.append('--cmake-args')

        # CMake directories
        catkincmd.append('-DCATKIN_DEVEL_PREFIX={}'.format(self.rosdir))
        catkincmd.append('-DCMAKE_INSTALL_PREFIX={}'.format(self.installdir))

        # Dep CMake files
        for dep in ['catkin', 'roscpp', 'rospy', 'genmsg'] + self.dependencies:
            catkincmd.append('-D{0}_DIR={1}'.format(dep, os.path.join(self.rosdir, 'share', dep, 'cmake')))

        # Compiler fun
        catkincmd.extend([
            '-DCMAKE_C_FLAGS="$CFLAGS"',
            '-DCMAKE_CXX_FLAGS="$CPPFLAGS"',
            '-DCMAKE_LD_FLAGS="$LDFLAGS"',
            '-DCMAKE_C_COMPILER={}'.format(os.path.join(self.installdir, 'usr', 'bin', 'gcc')),
            '-DCMAKE_CXX_COMPILER={}'.format(os.path.join(self.installdir, 'usr', 'bin', 'g++'))
        ])

        if not self.rosrun(catkincmd):
            return False

        if not self.rosrun(['catkin_make', 'install']):
            return False

        if not self.run(['find', self.installdir, '-name', '*.cmake', '-delete']):
            return False

        os.remove(os.path.join(self.installdir, 'usr/bin/xml2-config'))
        if not self.run(['rm', '-f', 'opt/ros/' + self.rosversion + '/.catkin', 'opt/ros/' + self.rosversion + '/.rosinstall', 'opt/ros/' + self.rosversion + '/setup.sh', 'opt/ros/' + self.rosversion + '/_setup_util.py'], cwd=self.installdir):
            return False

        return True
