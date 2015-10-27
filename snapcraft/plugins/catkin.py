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

"""The catkin plugin is useful for building ROS parts.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin specific keywords:

    - catkin-packages:
      (list of strings)
      List of catkin packages to build.
"""

import lxml.etree
import os
import tempfile
import logging

import snapcraft
import snapcraft.repo

logger = logging.getLogger(__name__)


class CatkinPlugin (snapcraft.BasePlugin):

    _PLUGIN_STAGE_SOURCES = '''
deb http://packages.ros.org/ros/ubuntu/ trusty main
deb http://${prefix}.ubuntu.com/${suffix}/ trusty main universe
deb http://${prefix}.ubuntu.com/${suffix}/ trusty-updates main universe
deb http://${prefix}.ubuntu.com/${suffix}/ trusty-security main universe
deb http://${security}.ubuntu.com/${suffix} trusty-security main universe
'''

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['rosversion'] = {
            'type': 'string',
            'default': 'indigo'
        }
        schema['properties']['catkin-packages'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': [],
        }

        schema['required'].append('catkin-packages')

        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.packages = set(options.catkin_packages)
        self.dependencies = ['ros-core']
        self.package_deps_found = False
        self.package_local_deps = {}
        self._deb_packages = []

    def pull(self):
        super().pull()
        self._setup_deb_packages()

    def env(self, root):
        return [
            'PYTHONPATH={0}'.format(
                os.path.join(
                    self.installdir,
                    'usr',
                    'lib',
                    self.python_version,
                    'dist-packages')),
            'DESTDIR={0}'.format(self.installdir),
            # ROS needs it but doesn't set it :-/
            'CPPFLAGS="-std=c++11 $CPPFLAGS -I{0} -I{1}"'.format(
                os.path.join(root, 'usr', 'include', 'c++', self.gcc_version),
                os.path.join(root,
                             'usr',
                             'include',
                             snapcraft.common.get_arch_triplet(),
                             'c++',
                             self.gcc_version)),
            'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{0}/opt/ros/{1}/lib'.format(
                root,
                self.options.rosversion),
            'ROS_MASTER_URI=http://localhost:11311',
            '_CATKIN_SETUP_DIR={}'.format(os.path.join(
                root, 'opt', 'ros', self.options.rosversion)),
            'echo FOO=BAR\nif `test -e {0}` ; then\n. {0} ;\nfi\n'.format(
                os.path.join(
                    root, 'opt', 'ros', self.options.rosversion, 'setup.sh'))
        ]

    @property
    def python_version(self):
        return self.run_output(['pyversions', '-i'])

    @property
    def gcc_version(self):
        return self.run_output(['gcc', '-dumpversion'])

    @property
    def rosdir(self):
        return os.path.join(self.installdir,
                            'opt', 'ros', self.options.rosversion)

    def _deps_from_packagesxml(self, f, pkg):
        try:
            tree = lxml.etree.parse(f)
        except lxml.etree.ParseError:
            logger.warning("Unable to read packages.xml file for '{}'".format(
                pkg))
            return

        for deptype in ('buildtool_depend', 'build_depend', 'run_depend'):
            for xmldep in tree.xpath('/package/' + deptype):
                dep = xmldep.text

                self.dependencies.append(dep)

                # Make sure we're not providing the dep ourselves
                if dep in self.packages:
                    self.package_local_deps[pkg].add(dep)
                    continue

                # If we're already getting this through a deb package,
                # we don't need it
                if (dep in self._deb_packages or
                        dep.replace('_', '-') in self._deb_packages):
                    continue

                # Get the ROS package for it
                self._deb_packages.append(
                    'ros-'+self.options.rosversion+'-'+dep.replace('_', '-'))

                if dep == 'roscpp':
                    self._deb_packages.append('g++')

    def _find_package_deps(self):
        if self.package_deps_found:
            return

        # Look for a package definition and pull deps if there are any
        for pkg in self.packages:
            if pkg not in self.package_local_deps:
                self.package_local_deps[pkg] = set()

            try:
                filename = os.path.join(
                    self.builddir, 'src', pkg, 'package.xml')
                with open(filename, 'r') as f:
                    self._deps_from_packagesxml(f, pkg)
            except os.FileNotFound:
                logger.warning("Unable to find packages.xml for '" + pkg + "'")
                pass

        self.package_deps_found = True

    def _setup_deb_packages(self):
        self._find_package_deps()

        if self._deb_packages:
            ubuntu = snapcraft.repo.Ubuntu(
                self.ubuntudir, sources=self.PLUGIN_STAGE_SOURCES)
            ubuntu.get(self._deb_packages)
            ubuntu.unpack(self.installdir)

    def _rosrun(self, commandlist, cwd=None):
        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('set -ex\n')
            f.write('exec {}\n'.format(' '.join(commandlist)))
            f.flush()

            self.run(['/bin/bash', f.name], cwd=cwd)

    def build(self):
        # Fixup ROS Cmake files that have hardcoded paths in them
        self.run([
            'find', self.rosdir, '-name', '*.cmake',
            '-exec', 'sed', '-i', '-e',
            r's|\(\W\)/usr/lib/|\1{0}/usr/lib/|g'.format(self.installdir),
            '{}', ';'
        ])

        self._find_package_deps()
        self._build_packages_deps()

        # the hacks
        findcmd = ['find', self.installdir, '-name', '*.cmake', '-delete']
        self.run(findcmd)

        self.run(
            ['rm', '-f',
             'opt/ros/' + self.options.rosversion + '/.catkin',
             'opt/ros/' + self.options.rosversion + '/.rosinstall',
             'opt/ros/' + self.options.rosversion + '/setup.sh',
             'opt/ros/' + self.options.rosversion + '/_setup_util.py'],
            cwd=self.installdir)

        os.remove(os.path.join(self.installdir, 'usr/bin/xml2-config'))

    def _build_packages_deps(self):
        # Ugly dependency resolution, just loop through until we can
        # find something to build. When we do, build it. Loop until we
        # either can't build anything or we built everything.
        built = set()
        built_pkg = True

        while len(built) < len(self.packages) and built_pkg:
            built_pkg = False
            for pkg in self.packages - built:
                if len(self.package_local_deps[pkg] - built) > 0:
                    continue
                self._handle_package(pkg)

                built.add(pkg)
                built_pkg = True

        if not built_pkg:
            raise RuntimeError('some packages failed to build')

    def _handle_package(self, pkg):
        catkincmd = ['catkin_make_isolated']

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
            catkincmd.append('-D{0}_DIR={1}'.format(
                dep.replace('-', '_'),
                os.path.join(self.rosdir, 'share', dep, 'cmake')))

        # Compiler fun
        catkincmd.extend([
            '-DCMAKE_C_FLAGS="$CFLAGS"',
            '-DCMAKE_CXX_FLAGS="$CPPFLAGS"',
            '-DCMAKE_LD_FLAGS="$LDFLAGS"',
            '-DCMAKE_C_COMPILER={}'.format(
                os.path.join(self.installdir, 'usr', 'bin', 'gcc')),
            '-DCMAKE_CXX_COMPILER={}'.format(
                os.path.join(self.installdir, 'usr', 'bin', 'g++'))
        ])

        self._rosrun(catkincmd)
