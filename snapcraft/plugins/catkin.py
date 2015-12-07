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

Additionally, this plugin uses the following plugin-specific keywords:

    - catkin-packages:
      (list of strings)
      List of catkin packages to build.
"""

import lxml.etree
import os
import tempfile
import logging
import shutil
import re

import snapcraft
from snapcraft import repo

logger = logging.getLogger(__name__)


class CatkinPlugin(snapcraft.BasePlugin):

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

            # Various ROS tools (e.g. rospack) keep a cache, and they determine
            # where the cache goes using $ROS_HOME.
            'ROS_HOME=$SNAP_APP_USER_DATA_PATH/.ros',
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
        tree = lxml.etree.parse(f)

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

        source_subdir = getattr(self.options, 'source_subdir', None)
        if source_subdir:
            sourcedir = os.path.join(self.sourcedir, source_subdir)
        else:
            sourcedir = self.sourcedir

        # catkin expects packages to be in 'src' but most repos
        # keep there catkin-packages in plain sight without a
        # 'src' directory as the top level.
        if os.path.exists(os.path.join(sourcedir, 'src')):
            basedir = os.path.join(sourcedir, 'src')
        else:
            basedir = sourcedir

        # Look for a package definition and pull deps if there are any
        for pkg in self.packages:
            if pkg not in self.package_local_deps:
                self.package_local_deps[pkg] = set()

            filename = os.path.join(basedir, pkg, 'package.xml')
            try:
                with open(filename, 'r') as f:
                    self._deps_from_packagesxml(f, pkg)
            except IOError as e:
                if e.errno is os.errno.ENOENT:
                    raise FileNotFoundError(
                        'unable to find "package.xml" for "{}"'.format(pkg))
                else:
                    raise e

        self.package_deps_found = True

    def _setup_deb_packages(self):
        self._find_package_deps()

        ubuntudir = os.path.join(self.partdir, 'ubuntu')
        os.makedirs(ubuntudir, exist_ok=True)
        if self._deb_packages:
            ubuntu = repo.Ubuntu(ubuntudir, sources=self.PLUGIN_STAGE_SOURCES)
            ubuntu.get(self._deb_packages)
            ubuntu.unpack(self.installdir)

    def _rosrun(self, commandlist, cwd=None):
        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('set -ex\n')
            f.write('exec {}\n'.format(' '.join(commandlist)))
            f.flush()

            self.run(['/bin/bash', f.name], cwd=cwd)

    def _provision_builddir(self):
        if os.path.exists(self.builddir):
            shutil.rmtree(self.builddir)
        dst = os.path.join(self.builddir, 'src')

        source_subdir = getattr(self.options, 'source_subdir', None)
        if source_subdir:
            sourcedir = os.path.join(self.sourcedir, source_subdir)
        else:
            sourcedir = self.sourcedir

        shutil.copytree(
            sourcedir, dst, symlinks=True,
            ignore=lambda d, s: snapcraft.common.SNAPCRAFT_FILES
            if d is self.sourcedir else [])

    def build(self):
        if os.path.exists(os.path.join(self.sourcedir, 'src')):
            super().build()
        else:
            self._provision_builddir()

        # Fixup ROS Cmake files that have hardcoded paths in them
        self.run([
            'find', self.rosdir, '-name', '*.cmake',
            '-exec', 'sed', '-i', '-e',
            r's|\(\W\)/usr/lib/|\1{0}/usr/lib/|g'.format(self.installdir),
            '{}', ';'
        ])

        self._find_package_deps()
        self._build_packages_deps()

        # Fix the shebang in _setup_util.py to be portable
        with open('{}/opt/ros/{}/_setup_util.py'.format(
                  self.installdir, self.options.rosversion), 'r+') as f:
            pattern = re.compile(r'#!.*python')
            replaced = pattern.sub(r'#!/usr/bin/env python', f.read())
            f.seek(0)
            f.truncate()
            f.write(replaced)

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

        # Install the package
        catkincmd.append('--install')

        # Specify the package to be built
        catkincmd.append('--pkg')
        catkincmd.append(pkg)

        # Don't clutter the real ROS workspace-- use the Snapcraft build
        # directory
        catkincmd.extend(['--directory', self.builddir])

        # Specify that the package should be installed along with the rest of
        # the ROS distro.
        catkincmd.extend(['--install-space', self.rosdir])

        # Start the CMake Commands
        catkincmd.append('--cmake-args')

        # Make sure all ROS dependencies can be found, even without the
        # workspace setup
        for dep in self.dependencies:
            catkincmd.append('-D{0}_DIR={1}'.format(
                dep.replace('-', '_'),
                os.path.join(self.rosdir, 'share', dep, 'cmake')))

        # Make sure we're using the compilers included in this .snap
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
