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

import lxml.etree
import os
import shutil
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
        self.rosversion = options.rosversion if options.rosversion else 'indigo'
        self.packages = options.catkin_packages
        self.dependencies = ['ros-core']
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
            'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{0}/opt/ros/{1}/lib'.format(root, self.rosversion),
            '_CATKIN_SETUP_DIR=' + os.path.join(root, 'opt', 'ros', self.rosversion),
            'if test -e {0}; then\nsource {0}\nfi'.format(os.path.join(root, 'opt', 'ros', self.rosversion, 'setup.sh'))
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

        # Look for a package definition and pull deps if there are any
        for pkg in self.packages:
            try:
                with open(os.path.join(self.builddir, 'src', pkg, 'package.xml'), 'r') as f:
                    tree = lxml.etree.parse(f)

                    for deptype in ['buildtool_depend', 'build_depend', 'run_depend']:
                        for dep in tree.xpath('/package/' + deptype):
                            self.dependencies.append(dep.text)
            except:
                pass

        # Make sure we get the ROS package for our dependencies
        for dep in self.dependencies:
            self._PLUGIN_STAGE_PACKAGES.append('ros-' + self.rosversion + '-' + dep.replace('_', '-'))

            if dep == 'roscpp':
                self._PLUGIN_STAGE_PACKAGES.extend(['g++', 'libstdc++-4.8-dev'])

        return True

    def rosrun(self, commandlist, cwd=None):
        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('set -ex\n')
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
        for dep in self.dependencies:
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

        # the hacks
        if not self.run(['find', self.installdir, '-name', '*.cmake', '-delete']):
            return False

        if not self.run(
            ['rm', '-f', 'opt/ros/' +
             self.rosversion + '/.catkin', 'opt/ros/' +
             self.rosversion + '/.rosinstall', 'opt/ros/' + self.rosversion +
             '/setup.sh', 'opt/ros/' + self.rosversion +
             '/_setup_util.py'], cwd=self.installdir):
            return False

        shutil.rmtree(os.path.join(self.installdir, 'home'))
        os.remove(os.path.join(self.installdir, 'usr/bin/xml2-config'))

        return True
