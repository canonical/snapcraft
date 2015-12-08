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

import os
import tempfile
import logging
import shutil
import re
import subprocess

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
        schema['properties']['rosdistro'] = {
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
        self.package_deps_found = False
        self.package_local_deps = {}
        self._deb_packages = []
        self.dependencies = []

        source_subdir = self.options.source_subdir
        if not source_subdir:
            source_subdir = 'src'

        self._ros_package_path = os.path.join(self.sourcedir, source_subdir)

        # Ensure the `source` key is set to the root of a Catkin workspace,
        # where the Catkin source space can be remapped via the source-subdir
        # key. We're this strict because `source` CANNOT be set to the Catkin
        # source space (likewise source_subdir cannot be set to the root of a
        # Catkin workspace), as it will result in a circular symlink upon
        # pull() which will break rosdep.
        if os.path.abspath(self.sourcedir) == os.path.abspath(
                self._ros_package_path):
            raise RuntimeError(
                'source-subdir cannot be the root of the Catkin workspace')

        self._rosdep = _Rosdep(self.options.rosdistro, self._ros_package_path,
                               os.path.join(self.partdir, 'rosdep'),
                               self.PLUGIN_STAGE_SOURCES)

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
                self.options.rosdistro),
            'ROS_MASTER_URI=http://localhost:11311',

            # Various ROS tools (e.g. rospack) keep a cache, and they determine
            # where the cache goes using $ROS_HOME.
            'ROS_HOME=$SNAP_APP_USER_DATA_PATH/.ros',
            '_CATKIN_SETUP_DIR={}'.format(os.path.join(
                root, 'opt', 'ros', self.options.rosdistro)),
            'echo FOO=BAR\nif `test -e {0}` ; then\n. {0} ;\nfi\n'.format(
                os.path.join(
                    root, 'opt', 'ros', self.options.rosdistro, 'setup.sh'))
        ]

    def pull(self):
        super().pull()

        # Make sure the package path exists before continuing
        if not os.path.exists(self._ros_package_path):
            raise FileNotFoundError(
                'Unable to find package path: "{}"'.format(
                    self._ros_package_path))

        self._rosdep.setup()
        self._setup_deb_packages()

    @property
    def python_version(self):
        return self.run_output(['pyversions', '-i'])

    @property
    def gcc_version(self):
        return self.run_output(['gcc', '-dumpversion'])

    @property
    def rosdir(self):
        return os.path.join(self.installdir,
                            'opt', 'ros', self.options.rosdistro)

    def _find_dependencies(self):
        if self.package_deps_found:
            return

        for package in self.packages:
            self._find_package_dependencies(package)

        self.package_deps_found = True

    def _find_package_dependencies(self, package):
        if package not in self.package_local_deps:
            self.package_local_deps[package] = set()

        # Query rosdep for the list of dependencies for this package
        dependencies = self._rosdep.get_dependencies(package)

        for dependency in dependencies:
            if dependency in self.packages:
                self.package_local_deps[package].add(dependency)
            else:
                self._resolve_system_dependency(dependency)

    def _resolve_system_dependency(self, dependency):
        system_dependency = self._rosdep.resolve_dependency(
            dependency)

        if not system_dependency:
            raise RuntimeError(
                'Package "{}" isn\'t a valid system dependency. '
                'Did you forget to add it to catkin-packages? If '
                'not, add the Ubuntu package containing it to '
                'stage-packages until you can get it into the '
                'rosdep database.'.format(dependency))

        self._deb_packages.append(system_dependency)

        # TODO: Not sure why this isn't pulled in by roscpp. Can it
        # be compiled by clang, etc.? If so, perhaps this should be
        # left up to the developer.
        if dependency == 'roscpp':
            self._deb_packages.append('g++')

    def _setup_deb_packages(self):
        self._find_dependencies()

        if self._deb_packages:
            logger.info('Preparing to fetch package dependencies...')
            ubuntudir = os.path.join(self.partdir, 'ubuntu')
            os.makedirs(ubuntudir, exist_ok=True)
            ubuntu = repo.Ubuntu(ubuntudir, sources=self.PLUGIN_STAGE_SOURCES)
            logger.info('Fetching package dependencies...')
            ubuntu.get(self._deb_packages)
            logger.info('Installing package dependencies...')
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

        self._find_dependencies()
        self._build_packages_deps()

        # Fix the shebang in _setup_util.py to be portable
        with open(os.path.join(self.rosdir, '_setup_util.py'), 'r+') as f:
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
        logger.info('Installing Catkin package: {}...'.format(pkg))

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


class _Rosdep:
    def __init__(self, ros_distro, ros_package_path, rosdep_path,
                 ubuntu_sources):
        self._ros_distro = ros_distro
        self._ros_package_path = ros_package_path
        self._ubuntu_sources = ubuntu_sources
        self._rosdep_path = rosdep_path
        self._rosdep_install_path = os.path.join(self._rosdep_path, 'install')
        self._rosdep_sources_path = os.path.join(self._rosdep_path,
                                                 'sources.list.d')
        self._rosdep_cache_path = os.path.join(self._rosdep_path, 'cache')

    def setup(self):
        os.makedirs(self._rosdep_install_path, exist_ok=True)
        os.makedirs(self._rosdep_sources_path, exist_ok=True)
        os.makedirs(self._rosdep_cache_path, exist_ok=True)

        logger.info('Preparing to fetch rosdep...')

        # Prepare to download Ubuntu .deb packages from the ROS repositories.
        # We need to do this here because the package dependencies are not
        # necessarily known at pull-time-- we need to interrogate the Catkin
        # packages first. Then we pull them down ourselves.
        ubuntu = repo.Ubuntu(self._rosdep_path, sources=self._ubuntu_sources)

        # rosdep is used for analyzing the dependencies specified within each
        # project's package.xml. But it introduces a complication, in that it's
        # not a dependency of the project, and we don't want to bloat the .snap
        # more than necessary. So we'll unpack it somewhere else, and use it
        # from there.
        logger.info('Fetching rosdep...')
        ubuntu.get(['python-rosdep'])
        ubuntu.unpack(self._rosdep_install_path)

        logger.info('Initializing rosdep database...')
        try:
            self._run(['init'])
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                'Error initializing rosdep database: {}'.format(e.output))

        logger.info('Updating rosdep database...')
        try:
            self._run(['update'])
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                'Error updating rosdep database: {}'.format(e.output))

    def get_dependencies(self, package_name):
        try:
            output = self._run(['keys', package_name]).strip()
            if output:
                return output.split('\n')
            else:
                return []
        except subprocess.CalledProcessError:
            raise FileNotFoundError(
                'Unable to find Catkin package "{}"'.format(package_name))

    def resolve_dependency(self, dependency_name):
        try:
            output = self._run(['resolve', dependency_name, '--rosdistro',
                                self._ros_distro])
        except subprocess.CalledProcessError:
            return None

        # `rosdep resolve` returns output like:
        # #apt
        # ros-indigo-package
        #
        # We're obviously only interested in the second line.
        resolved = output.split('\n')

        if len(resolved) < 2:
            raise RuntimeError(
                'Unexpected rosdep resolve output:\n{}'.format(output))

        return resolved[1]

    def _run(self, arguments):
        env = os.environ.copy()

        # We want to make sure we use our own rosdep (which is python)
        env['PATH'] = os.path.join(self._rosdep_install_path, 'usr', 'bin')
        env['PYTHONPATH'] = os.path.join(self._rosdep_install_path, 'usr',
                                         'lib', 'python2.7', 'dist-packages')

        # By default, rosdep uses /etc/ros/rosdep to hold its sources list. We
        # don't want that here since we don't want to touch the host machine
        # (not to mention it would require sudo), so we can redirect it via
        # this environment variable
        env['ROSDEP_SOURCE_PATH'] = self._rosdep_sources_path

        # By default, rosdep saves its cache in $HOME/.ros, which we shouldn't
        # access here, so we'll redirect it with this environment variable.
        env['ROS_HOME'] = self._rosdep_cache_path

        # This environment variable tells rosdep which directory to recursively
        # search for packages.
        env['ROS_PACKAGE_PATH'] = self._ros_package_path

        return subprocess.check_output(['rosdep'] + arguments,
                                       env=env).decode('utf8').strip()
