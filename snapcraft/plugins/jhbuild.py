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

"""The JHBuild plugin is used for GNOME-based parts.

JHBuild is a tool used to build components of the GNOME ecosystem from version
control. See the JHBuild manual for more details:
https://developer.gnome.org/jhbuild/stable/.

The plugin can be customized with the following keys:

  - modules: (required) a list of modules to include in the part, modules that
             should be skipped should be prefixed with '-', e.g. '-WebKit'
  - module-set: (default: gnome-world) the module set JHBuild should use
  - module-set-dir: the directory where the module set is located
               Either leave unset to use the default moduleset definitions, or
             set to `.` to reference your project folder.
  - jhbuild-archive: the source tarball directory on the build host
  - jhbuild-mirror: the DVCS repository directory on the build host

Advice:

  - You only need to specify a single module in most cases. JHBuild knows the
    module dependency graph.

  - The desktop-gtk3 helper is incompatible with GTK built by jhbuild. A custom
    helper is currently required.
    See github.com/diddledan/corebird for example.

  - Building WebKit requires a lot of time and computing power. If WebKit is a
    dependency of your JHBuild module, it can be skipped by adding '-WebKit' to
    the list of modules, and adding 'libwebkit2gtk-4.0-dev' to the
    build-packages and stage-packages.

  - Specify directories on the build host for jhbuild-archive and
    jhbuild-mirror to prevent repeated downloading of the JHBuild module
    sources. It's best to reserve common directories on your local machine that
    can be reused by all snaps you might want to build.
"""

import glob
import logging
import os
import subprocess
import snapcraft
from snapcraft.internal import common

HOME = os.path.expanduser('~') if os.geteuid() else os.path.join('/', 'root')

BUILD_PACKAGES = {
    'apt-file',
    'autoconf',
    'automake',
    'autopoint',
    'autotools-dev',
    'bison',
    'build-essential',
    'ca-certificates',
    'cvs',
    'docbook',
    'docbook-xml',
    'docbook-xsl',
    'flex',
    'gettext',
    'git',
    'intltool',
    'iso-codes',
    'libtext-csv-perl',
    'libtool',
    'libxml-parser-perl',
    'make',
    'ninja-build',
    'pkg-config',
    'pkgconf',
    'python-dev',
    'python3-dev',
    'subversion',
    'symlinks',
    'yelp-tools',
    'zlib1g-dev',
}


LOG = logging.getLogger(__name__)


def _get_jhbuild_user():
    myuid = os.getuid()
    if myuid != 0:
        return myuid

    try:
        uid = common.run_output(['id', '-u', 'jhbuild'])
    except subprocess.CalledProcessError:
        uid = None
    return uid


def _get_jhbuild_group():
    mygid = os.getgid()
    myuid = os.getuid()
    if myuid != 0:
        return mygid

    try:
        gid = common.run_output(['id', '-g', 'jhbuild'])
    except subprocess.CalledProcessError:
        gid = 65534
    return gid


class JHBuildPlugin(snapcraft.BasePlugin):
    """
    JHBuildPlugin provides jhbuild functionality to snapcraft
    """

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties'] = {
            'modules': {
                'type': 'array',
                'items': {
                    'type': 'string',
                },
                'minItems': 1,
                'uniqueItems': True,
            },
            'module-set': {
                'type': 'string',
                'default': 'gnome-world',
            },
            'module-set-dir': {
                'type': 'string',
                'default': '',
            },
            'jhbuild-archive': {
                'type': 'string',
                'default': '',
            },
            'jhbuild-mirror': {
                'type': 'string',
                'default': '',
            },
            'cflags': {
                'type': 'string',
                'default': '',
            },
        }

        schema['required'] = ['modules']

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return [
            'modules',
            'module-set',
            'module-set-dir',
            'jhbuild-archive',
            'jhbuild-mirror',
        ]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return [
            'cflags',
        ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.modules = [
            module for module in self.options.modules if module[0] != '-']

        self.skip_modules = [
            module[1:] for module in self.options.modules if module[0] == '-']

        self.build_packages += list(BUILD_PACKAGES)

        self.jhbuild_src = os.path.join(self.partdir, 'jhbuild', 'src')
        self.jhbuild_program = os.path.join(
            self.partdir, 'jhbuild', 'usr', 'bin', 'jhbuild')
        self.jhbuildrc_path = os.path.join(self.partdir, 'jhbuildrc')

    def enable_cross_compilation(self):
        """
        We don't support cross compilation (yet?)
        """
        raise NotImplementedError(
            'The {!s} plugin does not support cross compilation'.format(
                self.name))

    def run(self, cmd, cwd=None, **kwargs):
        """Run a command.

        Use run_output() if you need to capture or suppress output.

        :param list cmd: command arguments, first is the executable
        :param str cwd: working directory
        """
        return super().run(cmd, cwd=cwd, **kwargs)

    def run_output(self, cmd, cwd=None, **kwargs):
        """Run a command, capturing its output.

        Use run() if you don't need the output and prefer it to be piped to
        stdout.

        :param list cmd: command arguments, first is the executable
        :param str cwd: working directory
        :return: the output of the command
        :rtype: str
        """
        return super().run_output(cmd, cwd=cwd, **kwargs)

    def jhbuild(self, args, output=True, **kwargs):
        """Run JHBuild in the build stage.

        :param list args: command arguments without executable
        :return: the output of the command if captured
        :rtype: str
        """

        cwd = kwargs.pop('cwd', os.getcwd())
        cmd = []
        if os.getuid() == 0:
            cmd = ['sudo', '-H', '-u', 'jhbuild', '--']

        cmd = cmd + [self.jhbuild_program,
                     '--no-interact', '-f', self.jhbuildrc_path]

        run = self.run_output if output else self.run
        return run(cmd + args, cwd=cwd, **kwargs)

    def _create_jhbuild_user(self):
        common.run(['adduser', '--shell', '/bin/bash', '--disabled-password',
                    '--system', '--quiet', '--home', os.sep +
                    os.path.join(self.partdir, 'jhbuild', 'home'), 'jhbuild'])
        return _get_jhbuild_user()

    def _set_jhbuild_ownership(self, paths):
        jhbuild_user = int(_get_jhbuild_user() or self._create_jhbuild_user())
        jhbuild_group = int(_get_jhbuild_group())
        for path in paths:
            for filename in glob.iglob('%s/**' % path, recursive=True):
                if not os.path.exists(filename):
                    continue
                os.chown(filename, jhbuild_user, jhbuild_group)

    def _pull_jhbuild(self):
        LOG.info('Pulling JHBuild')

        repository = 'https://git.gnome.org/browse/jhbuild'

        if self.options.jhbuild_mirror:
            repository = self.options.jhbuild_mirror

        if os.path.isdir(self.jhbuild_src):
            self.run(['git', 'pull'], cwd=self.jhbuild_src)
        else:
            env = {'https_proxy': os.getenv('https_proxy') or ''}
            self.run(['git', 'clone', repository, self.jhbuild_src], env=env)

    def _setup_jhbuild(self):
        if not os.path.isfile(self.jhbuildrc_path):
            self._write_jhbuildrc()

        if not os.path.exists(self.jhbuild_program):
            LOG.info('Building JHBuild')

            self.run(['./autogen.sh', '--prefix=%s' % os.sep +
                      os.path.join(self.partdir, 'jhbuild', 'usr')],
                     cwd=self.jhbuild_src)

            self.run(['make', '-j%d' % self.parallel_build_count],
                     cwd=self.jhbuild_src)

            self.run(['make', '-j%d' % self.parallel_build_count,
                      'install'], cwd=self.jhbuild_src)

        archive_path = os.path.join(self.partdir, 'jhbuild', 'packages')
        mirror_path = os.path.join(self.partdir, 'jhbuild', 'mirror')
        unpacked_path = os.path.join(self.partdir, 'jhbuild', 'unpacked')

        make_paths = [self.builddir, self.installdir,
                      archive_path, mirror_path, unpacked_path]

        for folder in make_paths:
            os.makedirs(folder, exist_ok=True)

        self._set_jhbuild_ownership(make_paths)

    def _write_jhbuildrc(self):
        """
        _write_jhbuildrc() - write out the jhbuildrc file for build dependency
        specification
        """
        with open(self.jhbuildrc_path, 'w') as jhbuildrc_file:
            config = '''
moduleset = {module_set!r}
modulesets_dir = {modulesets_dir!r}
tarballdir = {tarballdir!r}
dvcs_mirror_dir = {dvcs_mirror_dir!r}
checkoutroot = {checkoutroot!r}
prefix = {prefix!r}
buildroot = {buildroot!r}
use_local_modulesets = True
skip = [{skip!s}]
module_autogenargs['gdk-pixbuf'] = '--disable-gio-sniffing'
extra_prefixes = [{extra_prefixes!s}]
cflags = {cflags!r}
'''

            extra_prefixes = [os.path.join(self.project.stage_dir, 'usr'),
                              os.path.join(self.project.stage_dir, 'usr',
                                           'local')]

            jhbuildrc_file.write(
                config.format(module_set=self.options.module_set,
                              modulesets_dir=self.options.module_set_dir,
                              tarballdir=self.options.jhbuild_archive or
                              os.path.join(self.partdir, 'jhbuild',
                                           'packages'),
                              dvcs_mirror_dir=os.path.join(
                                  self.partdir, 'jhbuild', 'mirror'),
                              checkoutroot=os.path.join(
                                  self.partdir, 'jhbuild', 'unpacked'),
                              buildroot=os.path.join(self.builddir, 'jhbuild'),
                              prefix=os.path.join(self.installdir, 'usr'),
                              skip=', '.join(
                                  ['\'%s\'' % module for module in
                                   self.skip_modules]),
                              extra_prefixes=', '.join(
                                  ['\'%s\'' % prefix for prefix in
                                   extra_prefixes]),
                              cflags=self.options.cflags,
                              ))

    def pull(self):
        self._pull_jhbuild()
        self._setup_jhbuild()

        self.jhbuild(['sanitycheck'], output=False)

        modules = self.jhbuild(['list'] + self.modules,
                               output=True).splitlines()

        if 'gtk+' in modules or 'gtk+-3' in modules:
            self.modules.append('adwaita-icon-theme')
            self.stage_packages.append('ttf-ubuntu-font-family')

        LOG.info('Pulling modules')
        self.jhbuild(['update'] + self.modules, output=False)

    def build(self):
        self._setup_jhbuild()

        LOG.info('Building modules')
        self.jhbuild(['build'] + self.modules,
                     cwd=self.builddir, output=False)

        LOG.info('Fixing symbolic links')
        self.run(['symlinks', '-c', '-d', '-r', '-s', '-v', self.installdir])
