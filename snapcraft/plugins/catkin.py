# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
    - source-space:
      (string)
      The source space containing Catkin packages. By default this is 'src'.
    - rosdistro:
      (string)
      The ROS distro required by this system. Defaults to 'indigo'.
    - include-roscore:
      (boolean)
      Whether or not to include roscore with the part. Defaults to true.
    - rosinstall-files:
      (list of strings)
      List of rosinstall files to merge while pulling. Paths are relative to
      the source.
    - catkin-cmake-args:
      (list of strings)
      Configure flags to pass onto the cmake invocation from catkin.
    - underlay:
      (object)
      Used to inform Snapcraft that this snap isn't standalone, and is actually
      overlaying a workspace from another snap via content sharing. Made up of
      two properties:
      - build-path:
        (string)
        Build-time path to existing workspace to underlay the one being built,
        for example '$SNAPCRAFT_STAGE/opt/ros/kinetic'.
      - run-path:
        (string)
        Run-time path of the underlay workspace (e.g. a subdirectory of the
        content interface's 'target' attribute.)
"""

import contextlib
import glob
import os
import tempfile
import logging
import re
import shutil
import subprocess
import textwrap

import snapcraft
from snapcraft.plugins import _ros
from snapcraft import (
    common,
    file_utils,
    formatting_utils,
    repo,
)
from snapcraft.internal import errors

logger = logging.getLogger(__name__)

# Map ROS releases to Ubuntu releases
_ROS_RELEASE_MAP = {
    'indigo': 'trusty',
    'jade': 'trusty',
    'kinetic': 'xenial',
    'lunar': 'xenial'
}

_SUPPORTED_DEPENDENCY_TYPES = {
    'apt',
}


class CatkinInvalidSystemDependencyError(errors.SnapcraftError):
    fmt = (
        "Package {dependency!r} isn't a valid system dependency. Did you "
        'forget to add it to catkin-packages? If not, add the Ubuntu package '
        'containing it to stage-packages until you can get it into the rosdep '
        'database.'
    )

    def __init__(self, dependency):
        super().__init__(dependency=dependency)


class CatkinUnsupportedDependencyTypeError(errors.SnapcraftError):
    fmt = (
        'Package {dependency!r} resolved to an unsupported type of '
        'dependency: {dependency_type!r}'
    )

    def __init__(self, dependency_type, dependency):
        super().__init__(dependency_type=dependency_type,
                         dependency=dependency)


class CatkinPlugin(snapcraft.BasePlugin):

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
        schema['properties']['source-space'] = {
            'type': 'string',
            'default': 'src',
        }

        # The default is true since we expect most Catkin packages to be ROS
        # packages. The only reason one wouldn't want to include ROS in the
        # snap is if library snaps exist, which will still likely be the
        # minority.
        schema['properties']['include-roscore'] = {
            'type': 'boolean',
            'default': True,
        }

        schema['properties']['underlay'] = {
            'type': 'object',
            'properties': {
                'build-path': {
                    'type': 'string',
                },
                'run-path': {
                    'type': 'string',
                }
            },
            'required': ['build-path', 'run-path'],
        }

        schema['properties']['rosinstall-files'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': [],
        }

        schema['properties']['catkin-cmake-args'] = {
            'type': 'array',
            'minitems': 1,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        schema['required'].append('catkin-packages')

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ['rosdistro', 'catkin-packages', 'source-space',
                'include-roscore', 'underlay', 'rosinstall-files']

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['build-attributes', 'catkin-cmake-args']

    @property
    def PLUGIN_STAGE_SOURCES(self):
        return """
deb http://packages.ros.org/ros/ubuntu/ {0} main
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0} main universe
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0}-updates main universe
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0}-security main universe
deb http://${{security}}.ubuntu.com/${{suffix}} {0}-security main universe
""".format(_ROS_RELEASE_MAP[self.options.rosdistro])

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(['libc6-dev', 'make'])

        # roslib is the base requiremet to actually create a workspace with
        # setup.sh and the necessary hooks.
        self.stage_packages.append(
            'ros-{}-roslib'.format(self.options.rosdistro))

        # Get a unique set of packages
        self.catkin_packages = set(options.catkin_packages)
        self._rosdep_path = os.path.join(self.partdir, 'rosdep')
        self._compilers_path = os.path.join(self.partdir, 'compilers')
        self._catkin_path = os.path.join(self.partdir, 'catkin')
        self._wstool_path = os.path.join(self.partdir, 'wstool')

        # The path created via the `source` key (or a combination of `source`
        # and `source-subdir` keys) needs to point to a valid Catkin workspace
        # containing another subdirectory called the "source space." By
        # default, this is a directory named "src," but it can be remapped via
        # the `source-space` key. It's important that the source space is not
        # the root of the Catkin workspace, since Catkin won't work that way
        # and it'll create a circular link that causes rosdep to hang.
        if self.options.source_subdir:
            self._ros_package_path = os.path.join(self.sourcedir,
                                                  self.options.source_subdir,
                                                  self.options.source_space)
        else:
            self._ros_package_path = os.path.join(self.sourcedir,
                                                  self.options.source_space)

        if os.path.abspath(self.sourcedir) == os.path.abspath(
                self._ros_package_path):
            raise RuntimeError(
                'source-space cannot be the root of the Catkin workspace')

        # Validate selected ROS distro
        if self.options.rosdistro not in _ROS_RELEASE_MAP:
            raise RuntimeError(
                'Unsupported rosdistro: {!r}. The supported ROS distributions '
                'are {}'.format(
                    self.options.rosdistro,
                    formatting_utils.humanize_list(
                        _ROS_RELEASE_MAP.keys(), 'and')))

    def env(self, root):
        """Runtime environment for ROS binaries and services."""

        paths = common.get_library_paths(root, self.project.arch_triplet)
        ld_library_path = formatting_utils.combine_paths(
            paths, prepend='', separator=':')

        env = [
            # This environment variable tells ROS nodes where to find ROS
            # master. It does not affect ROS master, however-- this is just the
            # default URI.
            'ROS_MASTER_URI=http://localhost:11311',

            # Various ROS tools (e.g. rospack, roscore) keep a cache or a log,
            # and use $ROS_HOME to determine where to put them.
            'ROS_HOME=${SNAP_USER_DATA:-/tmp}/ros',

            # FIXME: LP: #1576411 breaks ROS snaps on the desktop, so we'll
            # temporarily work around that bug by forcing the locale to
            # C.UTF-8.
            'LC_ALL=C.UTF-8',

            # The Snapcraft Core will ensure that we get a good LD_LIBRARY_PATH
            # overall, but it defines it after this function runs. Some ROS
            # tools will cause binaries to be run when we source the setup.sh,
            # below, so we need to have a sensible LD_LIBRARY_PATH before then.
            'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{}'.format(ld_library_path),
        ]

        # There's a chicken and egg problem here, everything run get's an
        # env built, even package installation, so the first runs for these
        # will likely fail.
        try:
            # The ROS packaging system tools (e.g. rospkg, etc.) don't go
            # into the ROS install path (/opt/ros/$distro), so we need the
            # PYTHONPATH to include the dist-packages in /usr/lib as well.
            env.append('PYTHONPATH={0}:$PYTHONPATH'.format(
                common.get_python2_path(root)))
        except errors.SnapcraftEnvironmentError as e:
            logger.debug(e)

        # The setup.sh we source below requires the in-snap python. Here we
        # make sure it's in the PATH before it's run.
        env.append('PATH=$PATH:{}/usr/bin'.format(root))

        if self.options.underlay:
            script = '. {}'.format(os.path.join(
                self.rosdir, 'snapcraft-setup.sh'))
        else:
            script = self._source_setup_sh(root, None)

        # Each of these lines is prepended with an `export` when the
        # environment is actually generated. In order to inject real shell code
        # we have to hack it in by appending it on the end of an item already
        # in the environment. FIXME: There should be a better way to do this.
        env[-1] = env[-1] + '\n\n' + script

        return env

    def pull(self):
        """Copy source into build directory and fetch dependencies.

        Catkin packages can specify their system dependencies in their
        package.xml. In order to support that, the Catkin packages are
        interrogated for their dependencies here. Since `stage-packages` are
        already installed by the time this function is run, the dependencies
        from the package.xml are pulled down explicitly.
        """

        super().pull()

        # There may be nothing contained within the source but a rosinstall
        # file. We need to use it to flesh out the workspace before continuing
        # with the pull.
        if self.options.rosinstall_files:
            wstool = _Wstool(
                self._ros_package_path, self._wstool_path,
                self.PLUGIN_STAGE_SOURCES, self.project)
            wstool.setup()

            source_path = self.sourcedir
            if self.options.source_subdir:
                source_path = os.path.join(self.sourcedir,
                                           self.options.source_subdir)
            _handle_rosinstall_files(
                wstool,  source_path, self.options.rosinstall_files)

        # Make sure the package path exists before continuing
        if self.catkin_packages and not os.path.exists(self._ros_package_path):
            raise FileNotFoundError(
                'Unable to find package path: "{}"'.format(
                    self._ros_package_path))

        # Validate the underlay. Note that this validation can't happen in
        # __init__ as the underlay will probably only be valid once a
        # dependency has been staged.
        catkin = None
        underlay_build_path = None
        if self.options.underlay:
            underlay_build_path = self.options.underlay['build-path']
        if underlay_build_path:
            if not os.path.isdir(underlay_build_path):
                raise errors.SnapcraftEnvironmentError(
                    'Requested underlay ({!r}) does not point to a valid '
                    'directory'.format(underlay_build_path))

            if not os.path.isfile(os.path.join(underlay_build_path,
                                               'setup.sh')):
                raise errors.SnapcraftEnvironmentError(
                    'Requested underlay ({!r}) does not contain a '
                    'setup.sh'.format(underlay_build_path))

            # Use catkin_find to discover dependencies already in the underlay
            catkin = _Catkin(
                self.options.rosdistro, underlay_build_path, self._catkin_path,
                self.PLUGIN_STAGE_SOURCES, self.project)
            catkin.setup()

            self._generate_snapcraft_setup_sh(
                self.installdir, underlay_build_path)

        # Pull our own compilers so we use ones that match up with the version
        # of ROS we're using.
        compilers = _Compilers(
            self._compilers_path, self.PLUGIN_STAGE_SOURCES, self.project)
        compilers.setup()

        # Use rosdep for dependency detection and resolution
        rosdep = _ros.rosdep.Rosdep(
            ros_distro=self.options.rosdistro,
            ros_package_path=self._ros_package_path,
            rosdep_path=self._rosdep_path,
            ubuntu_distro=_ROS_RELEASE_MAP[self.options.rosdistro],
            ubuntu_sources=self.PLUGIN_STAGE_SOURCES,
            project=self.project)
        rosdep.setup()

        self._setup_dependencies(rosdep, catkin)

    def _setup_dependencies(self, rosdep, catkin):
        # Parse the Catkin packages to pull out their system dependencies
        system_dependencies = _find_system_dependencies(
            self.catkin_packages, rosdep, catkin)

        # If the package requires roscore, resolve it into a system dependency
        # as well.
        if self.options.include_roscore:
            roscore = rosdep.resolve_dependency('ros_core')
            if roscore:
                for dependency_type, dependencies in roscore.items():
                    if dependency_type not in system_dependencies:
                        system_dependencies[dependency_type] = set()
                    system_dependencies[dependency_type] |= dependencies
            else:
                raise RuntimeError(
                    'Unable to determine system dependency for roscore')

        # Pull down and install any apt dependencies that were discovered
        apt_dependencies = system_dependencies.get('apt')
        if apt_dependencies:
            ubuntudir = os.path.join(self.partdir, 'ubuntu')
            os.makedirs(ubuntudir, exist_ok=True)

            logger.info('Preparing to fetch package dependencies...')
            ubuntu = repo.Ubuntu(ubuntudir,
                                 sources=self.PLUGIN_STAGE_SOURCES,
                                 project_options=self.project)

            logger.info('Fetching package dependencies...')
            try:
                ubuntu.get(apt_dependencies)
            except repo.errors.PackageNotFoundError as e:
                raise RuntimeError(
                    'Failed to fetch system dependencies: {}'.format(
                        e.message))

            logger.info('Installing package dependencies...')
            ubuntu.unpack(self.installdir)

    def clean_pull(self):
        super().clean_pull()

        # Remove the rosdep path, if any
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._rosdep_path)

        # Remove the compilers path, if any
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._compilers_path)

        # Remove the catkin path, if any
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._catkin_path)

    def _source_setup_sh(self, root, underlay_path):
        rosdir = os.path.join(root, 'opt', 'ros', self.options.rosdistro)
        if underlay_path:
            source_script = textwrap.dedent('''
                if [ -f {underlay_setup} ]; then
                    _CATKIN_SETUP_DIR={underlay} . {underlay_setup}
                    if [ -f {rosdir_setup} ]; then
                        set -- --extend
                        _CATKIN_SETUP_DIR={rosdir} . {rosdir_setup}
                    fi
                fi
            ''').format(
                underlay=underlay_path,
                underlay_setup=os.path.join(underlay_path, 'setup.sh'),
                rosdir=rosdir,
                rosdir_setup=os.path.join(rosdir, 'setup.sh'))
        else:
            source_script = textwrap.dedent('''
                if [ -f {rosdir_setup} ]; then
                    _CATKIN_SETUP_DIR={rosdir} . {rosdir_setup}
                fi
            ''').format(
                rosdir=rosdir,
                rosdir_setup=os.path.join(rosdir, 'setup.sh'))

        # We need to source ROS's setup.sh at this point. However, it accepts
        # arguments (thus will parse $@), and we really don't want it to, since
        # $@ in this context will be meant for the app being launched
        # (LP: #1660852). So we'll backup all args, source the setup.sh, then
        # restore all args for the wrapper's `exec` line.
        return textwrap.dedent('''
            # Shell quote arbitrary string by replacing every occurrence of '
            # with '\\'', then put ' at the beginning and end of the string.
            # Prepare yourself, fun regex ahead.
            quote()
            {{
                for i; do
                    printf %s\\\\n "$i" | sed "s/\'/\'\\\\\\\\\'\'/g;1s/^/\'/;\$s/\$/\' \\\\\\\\/"
                done
                echo " "
            }}

            BACKUP_ARGS=$(quote "$@")
            set --
            {}
            eval "set -- $BACKUP_ARGS"
        ''').format(source_script)  # noqa

    def _generate_snapcraft_setup_sh(self, root, underlay_path):
        script = self._source_setup_sh(root, underlay_path)
        os.makedirs(self.rosdir, exist_ok=True)
        with open(os.path.join(self.rosdir, 'snapcraft-setup.sh'), 'w') as f:
            f.write(script)

    @property
    def rosdir(self):
        return os.path.join(self.installdir, 'opt', 'ros',
                            self.options.rosdistro)

    def _run_in_bash(self, commandlist, cwd=None, env=None):
        with tempfile.NamedTemporaryFile(mode='w') as f:
            f.write('set -e\n')
            f.write('exec {}\n'.format(' '.join(commandlist)))
            f.flush()

            self.run(['/bin/bash', f.name], cwd=cwd, env=env)

    def build(self):
        """Build Catkin packages.

        This function runs some pre-build steps to prepare the sources for
        building in the Snapcraft environment, builds the packages via
        catkin_make_isolated, and finally runs some post-build clean steps
        to prepare the newly-minted install to be packaged as a .snap.
        """

        super().build()

        logger.info('Preparing to build Catkin packages...')
        self._prepare_build()

        logger.info('Building Catkin packages...')
        self._build_catkin_packages()

        logger.info('Cleaning up newly installed Catkin packages...')
        self._finish_build()

    def _prepare_build(self):
        self._use_in_snap_python()

        # Each Catkin package distributes .cmake files so they can be found via
        # find_package(). However, the Ubuntu packages pulled down as
        # dependencies contain .cmake files pointing to system paths (e.g.
        # /usr/lib, /usr/include, etc.). They need to be rewritten to point to
        # the install directory.
        def _new_path(path):
            if not path.startswith(self.installdir):
                # Not using os.path.join here as `path` is absolute.
                return self.installdir + path
            return path

        self._rewrite_cmake_paths(_new_path)

    def _rewrite_cmake_paths(self, new_path_callable):
        def _rewrite_paths(match):
            paths = match.group(1).strip().split(';')
            for i, path in enumerate(paths):
                # Offer the opportunity to rewrite this path if it's absolute.
                if os.path.isabs(path):
                    paths[i] = new_path_callable(path)

            return '"' + ';'.join(paths) + '"'

        # Looking for any path-like string
        file_utils.replace_in_file(self.rosdir, re.compile(r'.*Config.cmake$'),
                                   re.compile(r'"(.*?/.*?)"'),
                                   _rewrite_paths)

    def _finish_build(self):
        self._use_in_snap_python()

        # We've finished the build, but we need to make sure we turn the cmake
        # files back into something that doesn't include our installdir. This
        # way it's usable from the staging area, and won't clash with the same
        # file coming from other parts.
        pattern = re.compile(r'^{}'.format(self.installdir))

        def _new_path(path):
            return pattern.sub('$ENV{SNAPCRAFT_STAGE}', path)
        self._rewrite_cmake_paths(_new_path)

        # Replace the CMAKE_PREFIX_PATH in _setup_util.sh
        setup_util_file = os.path.join(self.rosdir, '_setup_util.py')
        if os.path.isfile(setup_util_file):
            with open(setup_util_file, 'r+') as f:
                pattern = re.compile(r"CMAKE_PREFIX_PATH = '.*/opt/ros.*")
                replaced = pattern.sub('CMAKE_PREFIX_PATH = []', f.read())
                f.seek(0)
                f.truncate()
                f.write(replaced)

        # Set the _CATKIN_SETUP_DIR in setup.sh to a sensible default, removing
        # our installdir (this way it doesn't clash with a setup.sh coming
        # from another part).
        setup_sh_file = os.path.join(self.rosdir, 'setup.sh')
        if os.path.isfile(setup_sh_file):
            with open(setup_sh_file, 'r+') as f:
                pattern = re.compile(r"\${_CATKIN_SETUP_DIR:=.*}")
                replaced = pattern.sub(
                    '${{_CATKIN_SETUP_DIR:=$SNAP/opt/ros/{}}}'.format(
                        self.options.rosdistro), f.read())
                f.seek(0)
                f.truncate()
                f.write(replaced)

        if self.options.underlay:
            underlay_run_path = self.options.underlay['run-path']
            self._generate_snapcraft_setup_sh('$SNAP', underlay_run_path)

    def _use_in_snap_python(self):
        # Fix all shebangs to use the in-snap python.
        file_utils.replace_in_file(self.rosdir, re.compile(r''),
                                   re.compile(r'^#!.*python'),
                                   r'#!/usr/bin/env python')

        # Also replace the python usage in 10.ros.sh to use the in-snap python.
        ros10_file = os.path.join(self.rosdir,
                                  'etc/catkin/profile.d/10.ros.sh')
        if os.path.isfile(ros10_file):
            with open(ros10_file, 'r+') as f:
                pattern = re.compile(r'/usr/bin/python')
                replaced = pattern.sub(r'python', f.read())
                f.seek(0)
                f.truncate()
                f.write(replaced)

    def _build_catkin_packages(self):
        # Nothing to do if no packages were specified
        if not self.catkin_packages:
            return

        catkincmd = ['catkin_make_isolated']

        # Install the package
        catkincmd.append('--install')

        # Specify the packages to be built
        catkincmd.append('--pkg')
        catkincmd.extend(self.catkin_packages)

        # Don't clutter the real ROS workspace-- use the Snapcraft build
        # directory
        catkincmd.extend(['--directory', self.builddir])

        # Account for a non-default source space by always specifying it
        catkincmd.extend(['--source-space', os.path.join(
            self.builddir, self.options.source_space)])

        # Specify that the package should be installed along with the rest of
        # the ROS distro.
        catkincmd.extend(['--install-space', self.rosdir])

        # All the arguments that follow are meant for CMake
        catkincmd.append('--cmake-args')

        # Make sure we're using our own compilers (the one on the system may
        # be the wrong version).
        compilers = _Compilers(
            self._compilers_path, self.PLUGIN_STAGE_SOURCES, self.project)
        build_type = 'Release'
        if 'debug' in self.options.build_attributes:
            build_type = 'Debug'
        catkincmd.extend([
            '-DCMAKE_C_FLAGS="$CFLAGS {}"'.format(compilers.cflags),
            '-DCMAKE_CXX_FLAGS="$CPPFLAGS {}"'.format(compilers.cxxflags),
            '-DCMAKE_LD_FLAGS="$LDFLAGS {}"'.format(compilers.ldflags),
            '-DCMAKE_C_COMPILER={}'.format(compilers.c_compiler_path),
            '-DCMAKE_CXX_COMPILER={}'.format(compilers.cxx_compiler_path),
            '-DCMAKE_BUILD_TYPE={}'.format(build_type),
        ])

        # Finally, add any cmake-args requested from the plugin options
        catkincmd.extend(self.options.catkin_cmake_args)

        # This command must run in bash due to a bug in Catkin that causes it
        # to explode if there are spaces in the cmake args (which there are).
        # This has been fixed in Catkin Tools... perhaps we should be using
        # that instead.
        self._run_in_bash(catkincmd, env=compilers.environment)

    def snap_fileset(self):
        """Filter useless files out of the snap.

        - opt/ros/<rosdistro>/.rosinstall points to the part installdir, and
          isn't useful from the snap anyway.
        """

        fileset = super().snap_fileset()
        fileset.append('-{}'.format(
            os.path.join('opt', 'ros', self.options.rosdistro, '.rosinstall')))
        return fileset


def _find_system_dependencies(catkin_packages, rosdep, catkin):
    """Find system dependencies for a given set of Catkin packages."""

    resolved_dependencies = {}

    logger.info('Determining system dependencies for Catkin packages...')
    for package in catkin_packages:
        # Query rosdep for the list of dependencies for this package
        dependencies = rosdep.get_dependencies(package)

        for dependency in dependencies:
            _resolve_package_dependencies(
                catkin_packages, dependency, catkin, rosdep,
                resolved_dependencies)

    # We currently have nested dict structure of:
    #    dependency name -> package type -> package names
    #
    # We want to return a flattened dict of package type -> package names.
    flattened_dependencies = {}
    for dependency_types in resolved_dependencies.values():
        for key, value in dependency_types.items():
            if key not in flattened_dependencies:
                flattened_dependencies[key] = set()
            flattened_dependencies[key] |= value

    # Finally, return that dict of dependencies
    return flattened_dependencies


def _resolve_package_dependencies(catkin_packages, dependency, catkin, rosdep,
                                  resolved_dependencies):
    # No need to resolve this dependency if we know it's local, or if
    # we've already resolved it into a system dependency
    if (dependency in catkin_packages or
            dependency in resolved_dependencies):
        return

    if _dependency_is_in_underlay(catkin, dependency):
        # Package was found-- don't pull anything extra to satisfy
        # this dependency.
        logger.debug(
            'Satisfied dependency {!r} in underlay'.format(
                dependency))
        return

    # In this situation, the package depends on something that we
    # weren't instructed to build. It's probably a system dependency,
    # but the developer could have also forgotten to tell us to build
    # it.
    try:
        these_dependencies = rosdep.resolve_dependency(dependency)
    except _ros.rosdep.RosdepDependencyNotFoundError:
        raise CatkinInvalidSystemDependencyError(dependency)

    for key, value in these_dependencies.items():
        if key not in _SUPPORTED_DEPENDENCY_TYPES:
            raise CatkinUnsupportedDependencyTypeError(key, dependency)

        resolved_dependencies[dependency] = {key: value}


def _dependency_is_in_underlay(catkin, dependency):
    if catkin:
        # Before trying to resolve this dependency into a system
        # dependency, see if it's already in the underlay.
        try:
            catkin.find(dependency)
        except CatkinPackageNotFoundError:
            # No package by that name is available
            pass
        else:
            return True
    return False


def _handle_rosinstall_files(wstool, source_path, rosinstall_files):
    """Merge given rosinstall files into our workspace."""

    for rosinstall_file in rosinstall_files:
        logger.info('Merging {}'.format(rosinstall_file))
        wstool.merge(os.path.join(source_path, rosinstall_file))

    logger.info('Updating workspace...')
    wstool.update()


class CatkinPackageNotFoundError(errors.SnapcraftError):
    fmt = 'Unable to find Catkin package {package_name!r}'

    def __init__(self, package_name):
        super().__init__(package_name=package_name)


class _Compilers:
    def __init__(self, compilers_path, ubuntu_sources, project):
        self._compilers_path = compilers_path
        self._ubuntu_sources = ubuntu_sources
        self._project = project

        self._compilers_install_path = os.path.join(
            self._compilers_path, 'install')
        self.__gcc_version = None

    def setup(self):
        os.makedirs(self._compilers_install_path, exist_ok=True)

        # Since we support building older ROS distros we need to make sure we
        # use the corresponding compiler versions, so they can't be
        # build-packages. We'll just download them to another place and use
        # them from there.
        logger.info('Preparing to fetch compilers...')
        ubuntu = repo.Ubuntu(
            self._compilers_path, sources=self._ubuntu_sources,
            project_options=self._project)

        logger.info('Fetching compilers...')
        ubuntu.get(['gcc', 'g++'])

        logger.info('Installing compilers...')
        ubuntu.unpack(self._compilers_install_path)

    @property
    def environment(self):
        env = os.environ.copy()

        paths = common.get_library_paths(
            self._compilers_install_path, self._project.arch_triplet)
        ld_library_path = formatting_utils.combine_paths(
            paths, prepend='', separator=':')

        env['LD_LIBRARY_PATH'] = (
            env.get('LD_LIBRARY_PATH', '') + ':' + ld_library_path)

        env['PATH'] = env.get('PATH', '') + ':' + os.path.join(
            self._compilers_install_path, 'usr', 'bin')

        return env

    @property
    def c_compiler_path(self):
        return os.path.join(self._compilers_install_path, 'usr', 'bin', 'gcc')

    @property
    def cxx_compiler_path(self):
        return os.path.join(self._compilers_install_path, 'usr', 'bin', 'g++')

    @property
    def cflags(self):
        return ''

    @property
    def cxxflags(self):
        paths = set(common.get_include_paths(
            self._compilers_install_path, self._project.arch_triplet))

        try:
            paths.add(_get_highest_version_path(os.path.join(
                self._compilers_install_path, 'usr', 'include', 'c++')))
            paths.add(_get_highest_version_path(os.path.join(
                self._compilers_install_path, 'usr', 'include',
                self._project.arch_triplet, 'c++')))
        except RuntimeError as e:
            raise RuntimeError('Unable to determine gcc version: {}'.format(
                str(e)))

        return formatting_utils.combine_paths(
            paths, prepend='-I', separator=' ')

    @property
    def ldflags(self):
        paths = common.get_library_paths(
            self._compilers_install_path, self._project.arch_triplet)
        return formatting_utils.combine_paths(
            paths, prepend='-L', separator=' ')


class _Catkin:
    def __init__(self, ros_distro, workspace, catkin_path, ubuntu_sources,
                 project):
        self._ros_distro = ros_distro
        self._workspace = workspace
        self._catkin_path = catkin_path
        self._ubuntu_sources = ubuntu_sources
        self._project = project
        self._catkin_install_path = os.path.join(self._catkin_path, 'install')

    def setup(self):
        os.makedirs(self._catkin_install_path, exist_ok=True)

        # With the introduction of an underlay, we no longer know where Catkin
        # is. Let's just fetch/unpack our own, and use it.
        logger.info('Preparing to fetch catkin...')
        ubuntu = repo.Ubuntu(self._catkin_path, sources=self._ubuntu_sources,
                             project_options=self._project)
        logger.info('Fetching catkin...')
        ubuntu.get(['ros-{}-catkin'.format(self._ros_distro)])

        logger.info('Installing catkin...')
        ubuntu.unpack(self._catkin_install_path)

    def find(self, package_name):
        try:
            return self._run(['--first-only', package_name]).strip()
        except subprocess.CalledProcessError:
            raise CatkinPackageNotFoundError(package_name)

    def _run(self, arguments):
        with tempfile.NamedTemporaryFile(mode='w+') as f:
            lines = ['export PYTHONPATH={}'.format(os.path.join(
                self._catkin_install_path, 'usr', 'lib', 'python2.7',
                'dist-packages'))]

            ros_path = os.path.join(
                self._catkin_install_path, 'opt', 'ros', self._ros_distro)
            bin_paths = (
                os.path.join(ros_path, 'bin'),
                os.path.join(self._catkin_install_path, 'usr', 'bin'))
            lines.append('export {}'.format(
                formatting_utils.format_path_variable(
                    'PATH', bin_paths, prepend='', separator=':')))

            lines.append('export _CATKIN_SETUP_DIR={}'.format(self._workspace))
            lines.append('source {}'.format(os.path.join(
                self._workspace, 'setup.sh')))
            lines.append('exec "$@"')
            f.write('\n'.join(lines))
            f.flush()
            return subprocess.check_output(
                ['/bin/bash', f.name, 'catkin_find'] + arguments,
                stderr=subprocess.STDOUT).decode('utf8').strip()


def _get_highest_version_path(path):
    paths = sorted(glob.glob(os.path.join(path, '*')))
    if not paths:
        raise RuntimeError('nothing found in {!r}'.format(path))

    return paths[-1]


class _Wstool:
    def __init__(self, ros_package_path, wstool_path, ubuntu_sources, project):
        self._ros_package_path = ros_package_path
        self._ubuntu_sources = ubuntu_sources
        self._wstool_path = wstool_path
        self._wstool_install_path = os.path.join(wstool_path, 'install')
        self._project = project

    def setup(self):
        os.makedirs(self._wstool_install_path, exist_ok=True)

        # wstool isn't a dependency of the project, so we'll unpack it
        # somewhere else, and use it from there.
        logger.info('Preparing to fetch wstool...')
        ubuntu = repo.Ubuntu(self._wstool_path, sources=self._ubuntu_sources,
                             project_options=self._project)
        logger.info('Fetching wstool...')
        ubuntu.get(['python-wstool'])

        logger.info('Installing wstool...')
        ubuntu.unpack(self._wstool_install_path)

        logger.info('Initializing workspace (if necessary)...')
        try:
            self._run(['init', self._ros_package_path, '-j{}'.format(
                self._project.parallel_build_count)])
        except subprocess.CalledProcessError as e:
            output = e.output.decode('utf8').strip()
            if 'already is a workspace' not in output:
                raise RuntimeError(
                    'Error initializing workspace:\n{}'.format(output))

    def merge(self, rosinstall_file):
        return self._run(
            ['merge', rosinstall_file, '--confirm-all', '-t{}'.format(
                self._ros_package_path)]).strip()

    def update(self):
        return self._run(
            ['update', '-j{}'.format(self._project.parallel_build_count),
             '-t{}'.format(self._ros_package_path)]).strip()

    def _run(self, arguments):
        env = os.environ.copy()

        env['PATH'] += ':' + os.path.join(self._wstool_install_path, 'usr',
                                          'bin')
        # The execution path of python doesn't seem to cause these packages to
        # be picked up, so put them on the PYTHONPATH manually.
        env['PYTHONPATH'] = os.path.join(self._wstool_install_path, 'usr',
                                         'lib', 'python2.7', 'dist-packages')

        return subprocess.check_output(['wstool'] + arguments,
                                       env=env).decode('utf8').strip()
