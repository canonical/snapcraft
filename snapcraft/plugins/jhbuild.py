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

  - modules (required): a list of modules to include in the part, modules that
                        should be skipped should be prefixed with '-'
  - moduleset: the module set JHBuild should use (default: gnome-world)
  - snap-name (required): the name of the snap this part is included in
  - snap-revision: the revision of the snap this part is included in
                   (default: current)
  - container-name: the name of the LXC container used for building
                    (default: snapcraft-<snap name>-<part name>)
  - distribution: the distribution the LXC container is based off of
                  (default: host distribution)
  - release: the release of the distribution the LXC container is based off of
             (default: host release)
  - architecture: the architecture of the LXC container
                  (default: host architecture)
  - container-packages: a list of build packages the LXC container should be
                        bootstrapped with
  - debmirror: the mirror to use for downloading .deb packages
  - jhtarballs: the directory on the host containing source tarballs
  - jhmirror: the directory on the host containing DVCS mirror repositories
  - jhcheckout: the directory on the host container DVCS checkout repositories
  - ccache: the directory on the host to use for ccache caching
  - disable-pull: True to skip the pull stage (default: False)
  - disable-build: True to skip the build stage (default: False)
  - disable-jhupdate: True to skip updating jhtarballs, jhmirror and jhcheckout
                      (default: False)
  - disable-parallel: True to disable parallel building (default: False)

Advice:

  - The command for launching an app from a JHBuild part is:
    jhbuild -f $SNAP/etc/jhbuildrc run <app>

    So for a glade snap, the app would look like:

    apps:
      glade:
        command: jhbuild -f $SNAP/etc/jhbuildrc run glade
        plugs: [x11]

  - Building WebKit requires a lot of time and computing power. If WebKit is a
    dependency of your JHBuild module, it can be skipped by adding '-WebKit' to
    the list of modules, and adding 'libwebkit2gtk-4.0-dev' to the list of
    container-packages.

  - Using the debmirror option in combination with a local apt-cacher-ng
    installation can prevent repeated downloading of .deb packages. Use the IP
    address of your lxcbr0 interface with the port number apt-cacher-ng uses.
    For example, debmirror: http://10.0.3.1:3142

  - Specify directories on your host for jhtarballs and jhmirror to prevent
    repeated downloading of the JHBuild module sources. It is best to reserve
    common directories on your local machine that can be used for all snaps you
    might want to build.

  - If you are debugging your snap, but don't need to rebuild the sources (for
    example, to determine the list of plugs required under confinement), use
    the disable-pull, disable-build, and disable-jhupdate options.

  - To add a shell for debugging your snap, add an extra app whose command is:
    jhbuild -f $SNAP/etc/jhbuildrc shell. You might also want to add gdb,
    strace, etc. to your list of stage-packages for debugging purposes.
"""

import logging
import os
import shlex
import platform
import snapcraft
import time  # TODO: wait for networking

logger = logging.getLogger(__name__)

BUILD_PACKAGES = [
    'ca-certificates',
    'git',
    'make',
    'autoconf',
    'automake',
    'gettext',
    'pkg-config',
    'yelp-tools',
    'ccache',
    'libtool',
    'docbook-xsl',
    'libxml-parser-perl',
    'cvs',
    'subversion',
    'flex',
    'bison',
    'apt-file',
]

STAGE_PACKAGES = [
    'python',
    'ttf-ubuntu-font-family',
    'adwaita-icon-theme-full',
]

# build dependencies that JHBuild doesn't know about
MODULE_EXCEPTIONS = {
    'vala': ['valac'],
    'gst-plugins-bad': ['libgl1-mesa-dev'],
}

# build dependencies that cannot be uniquely determined via apt-file
SYSDEP_EXCEPTIONS = {
    'c_include:boost/variant.hpp': ['libboost-dev'],
    'c_include:jpeglib.h': ['libjpeg-dev'],
    'c_include:readline/readline.h': ['libreadline-dev'],
    'c_include:tiff.h': ['libtiff-dev'],
    'path:automake': ['automake'],
    'path:bison': ['bison'],
    'path:bogofilter': ['bogofilter-bdb'],
    'path:c++': ['g++'],
    'path:cc': ['gcc'],
    'path:flex': ['flex'],
    'path:gdb': ['gdb'],
    'path:krb5-config': ['libkrb5-dev'],
    'path:llvm-config': ['llvm'],
    'path:make': ['make'],
    'path:pkg-config': ['pkg-config'],
    'path:rapper': ['raptor2-utils'],
    'path:ruby': ['ruby'],
    'pkgconfig:dbus-1': ['libdbus-1-dev'],
    'pkgconfig:egl': ['libegl1-mesa-dev'],
    'pkgconfig:glesv2': ['libgles2-mesa-dev'],
    'pkgconfig:libpng': ['libpng-dev'],
    'pkgconfig:neon': ['libneon27-gnutls-dev'],
    'pkgconfig:zlib': ['zlib1g-dev'],
    'xml:-//OASIS//DTD DocBook XML V4.3//EN': ['docbook-xml'],
    'xml:http://docbook.sourceforge.net/release/xsl/current/': ['docbook-xsl'],
}


class JHBuildPlugin(snapcraft.BasePlugin):

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
            'moduleset': {
                'type': 'string',
                'default': 'gnome-world',
            },
            'snap-name': {
                'type': 'string',
            },
            'snap-revision': {
                'type': 'string',
                'default': 'current',
            },
            'container-name': {
                'type': 'string',
            },
            'distribution': {
                'type': 'string',
                'default': platform.linux_distribution()[0].lower(),
            },
            'release': {
                'type': 'string',
                'default': platform.linux_distribution()[2].lower(),
            },
            'architecture': {
                'type': 'string',
                'default': {
                    'amd64': 'amd64',
                    'x86_64': 'amd64',
                    'i386': 'i386',
                    'i686': 'i386',
                }[platform.machine()],
            },
            'container-packages': {
                'type': 'array',
                'items': {
                    'type': 'string',
                },
                'uniqueItems': True,
            },
            'debmirror': {
                'type': 'string',
            },
            'jhtarballs': {
                'type': 'string',
            },
            'jhmirror': {
                'type': 'string',
            },
            'jhcheckout': {
                'type': 'string',
            },
            'ccache': {
                'type': 'string',
            },
            'disable-pull': {
                'type': 'boolean',
                'default': False,
            },
            'disable-build': {
                'type': 'boolean',
                'default': False,
            },
            'disable-jhupdate': {
                'type': 'boolean',
                'default': False,
            },
            'disable-parallel': schema['properties']['disable-parallel'],
        }

        schema['required'] = [
            'modules',
            'snap-name',
        ]

        schema['pull-properties'] = [
            'modules',
            'moduleset',
            'snap-name',
            'snap-revision',
            'container-name',
            'distribution',
            'release',
            'architecture',
            'container-packages',
            'debmirror',
            'jhtarballs',
            'jhmirror',
            'jhcheckout',
            'ccache',
            'disable-pull',
            'disable-jhupdate',
        ]

        schema['build-properties'] = [
            'disable-build',
            'disable-parallel',
        ]

        return schema

    def __init__(self, name, options, project=None):
        super().__init__(name, options, project)

        self.modules = [module for module in self.options.modules
                        if not module.startswith('-')]
        self.skipped_modules = [module[1:] for module in self.options.modules
                                if module.startswith('-')]
        self.container_name = (self.options.container_name
                               or 'snapcraft-%s-%s'
                               % (self.options.snap_name, self.name))
        self.src_jhtarballs = (self.options.jhtarballs
                               or os.path.join(self.sourcedir, 'jhtarballs'))
        self.src_jhmirror = (self.options.jhmirror
                             or os.path.join(self.sourcedir, 'jhmirror'))
        self.src_jhcheckout = (self.options.jhcheckout
                               or os.path.join(self.sourcedir, 'jhcheckout'))
        self.src_ccache = (self.options.ccache
                           or os.path.join(self.sourcedir, 'ccache'))
        self.def_lxc_conf = os.path.join('/', 'etc', 'lxc', 'default.conf')
        self.lxc_conf = os.path.join(self.sourcedir, 'lxc.conf')
        self.sources_list = os.path.join('/', 'etc', 'apt', 'sources.list')
        self.rel_prefix = os.path.join('snap',
                                       self.options.snap_name,
                                       self.options.snap_revision)
        self.prefix = os.path.join('/', self.rel_prefix)
        self.jhsource = os.path.join(self.prefix, 'usr', 'src', 'jhbuild')
        self.rel_host = os.path.join('host')
        self.rel_jhtarballs = os.path.join(self.rel_host, 'jhtarballs')
        self.rel_jhmirror = os.path.join(self.rel_host, 'jhmirror')
        self.rel_jhcheckout = os.path.join(self.rel_host, 'jhcheckout')
        self.rel_ccache = os.path.join(self.rel_host, 'ccache')
        self.jhtarballs = os.path.join('/', self.rel_jhtarballs)
        self.jhmirror = os.path.join('/', self.rel_jhmirror)
        self.jhcheckout = os.path.join('/', self.rel_jhcheckout)
        self.ccache = os.path.join('/', self.rel_ccache)
        self.jhbuildrc = os.path.join(self.prefix, 'etc', 'jhbuildrc')
        # TODO: use custom /etc/fonts/fonts.conf
        self.xdgdatahome = os.path.join(self.prefix, 'usr', 'share')
        self.stage_packages += STAGE_PACKAGES

    @property
    def user(self):
        """Get the container's unprivileged user."""
        if not hasattr(self, '_user') or not self._user:
            self._user = self.lxc_run([
                'ls',
                '-ld',
                self.prefix,
            ], root=True, output=True).split()[2]

            if self._user == '1000':
                self.lxc_run([
                    'useradd',
                    '-m',
                    '-u',
                    '1000',
                    'user',
                ], root=True)

                self._user = self.lxc_run([
                    'ls',
                    '-ld',
                    self.prefix,
                ], root=True, output=True).split()[2]

        return self._user

    def lxc_containers(self, state=''):
        """Get a list of all LXC containers."""
        if state:
            lines = self.run_output(['lxc-ls', '-1', state]).splitlines()
        else:
            lines = self.run_output(['lxc-ls', '-1']).splitlines()

        return [line.strip() for line in lines]

    def all_containers(self):
        """Get a list of all LXC containers."""
        return self.lxc_containers()

    def stopped_containers(self):
        """Get a list of stopped LXC containers."""
        return self.lxc_containers('--stopped')

    def frozen_containers(self):
        """Get a list of frozen LXC containers."""
        return self.lxc_containers('--frozen')

    def lxc_create(self):
        """Create the LXC build container."""
        uid, gid = os.geteuid(), os.getegid()

        if uid == 0:
            raise RuntimeError('Do not run as root')

        if not self.options.jhtarballs:
            os.makedirs(self.src_jhtarballs, exist_ok=True)
        if not self.options.jhmirror:
            os.makedirs(self.src_jhmirror, exist_ok=True)
        if not self.options.jhcheckout:
            os.makedirs(self.src_jhcheckout, exist_ok=True)
        if not self.options.ccache:
            os.makedirs(self.src_ccache, exist_ok=True)

        if self.container_name not in self.all_containers():
            if not os.path.exists(self.lxc_conf):
                with open(self.lxc_conf, 'w') as f:
                    f.write('lxc.include = %s\n' % self.def_lxc_conf)

                    f.write('lxc.id_map = u 0 100000 1000\n')
                    f.write('lxc.id_map = u 1000 %d 1\n' % uid)
                    f.write('lxc.id_map = u 1001 101001 64535\n')

                    f.write('lxc.id_map = g 0 100000 1000\n')
                    f.write('lxc.id_map = g 1000 %d 1\n' % gid)
                    f.write('lxc.id_map = g 1001 101001 64535\n')

                    f.write('lxc.mount.entry = ')
                    f.write(self.installdir)
                    f.write(' ')
                    f.write(self.rel_prefix)
                    f.write(' none bind,optional,create=dir\n')

                    f.write('lxc.mount.entry = ')
                    f.write(self.src_jhtarballs)
                    f.write(' ')
                    f.write(self.rel_jhtarballs)
                    f.write(' none bind,optional,create=dir\n')

                    f.write('lxc.mount.entry = ')
                    f.write(self.src_jhmirror)
                    f.write(' ')
                    f.write(self.rel_jhmirror)
                    f.write(' none bind,optional,create=dir\n')

                    f.write('lxc.mount.entry = ')
                    f.write(self.src_jhcheckout)
                    f.write(' ')
                    f.write(self.rel_jhcheckout)
                    f.write(' none bind,optional,create=dir\n')

                    f.write('lxc.mount.entry = ')
                    f.write(self.src_ccache)
                    f.write(' ')
                    f.write(self.rel_ccache)
                    f.write(' none bind,optional,create=dir\n')

            self.run([
                'lxc-create',
                '-n',
                self.container_name,
                '-f',
                self.lxc_conf,
                '-t',
                'download',
                '--',
                '-d',
                self.options.distribution,
                '-r',
                self.options.release,
                '-a',
                self.options.architecture,
            ])

            self.lxc_start()

            if self.options.debmirror:
                mirror = self.options.debmirror

                self.lxc_run([
                    'sed',
                    '-ie',
                    's/https\\?:\\/\\//%s\\//' % mirror.replace('/', '\\/'),
                    self.sources_list,
                ], root=True)
        else:
            self.lxc_start()

    def lxc_start(self):
        """Start the LXC build container."""
        if self.container_name in self.stopped_containers():
            logger.info('Starting container \'%s\'' % self.container_name)

            self.run([
                'lxc-start',
                '-n',
                self.container_name,
            ])
        elif self.container_name in self.frozen_containers():
            logger.info('Thawing container \'%s\'' % self.container_name)

            self.run([
                'lxc-unfreeze',
                '-n',
                self.container_name,
            ])

        self.run([
            'lxc-wait',
            '-n',
            self.container_name,
            '-s',
            'RUNNING',
        ])

        time.sleep(5)  # TODO: wait for networking

    def lxc_stop(self):
        """Stop the LXC build container."""
        try:
            self.run([
                'lxc-stop',
                '-n',
                self.container_name,
            ])
        except:
            pass

        self.run([
            'lxc-wait',
            '-n',
            self.container_name,
            '-s',
            'STOPPED',
        ])

    def lxc_run(self, args, cwd='', root=False, stdin=None, output=False):
        """Run a command in the LXC build container.

        :param list args: the command as a list of tokens
        :param str cwd: the directory to run the command from (default: '')
        :param bool root: True to run as root in the container (default: False)
        :param int stdin: a file descriptor for standard input (default: None)
        :param bool output: True to return the output of the command (default:
                            False)
        """
        if root:
            home = os.path.join('/', 'root')
        else:
            home = os.path.join('/', 'home', self.user)

        env = {
            'HOME': home,
            'PATH': ':'.join([
                os.path.join(self.prefix, 'bin'),
                os.path.join('/', 'usr', 'lib', 'ccache'),
                os.path.join('/', 'usr', 'bin'),
                os.path.join('/', 'bin'),
                os.path.join('/', 'usr', 'sbin'),
                os.path.join('/', 'sbin'),
            ]),
            'CCACHE_DIR': self.ccache,
        }

        env_list = ['%s=%s' % (shlex.quote(var), shlex.quote(env[var]))
                    for var in env]

        cmd = [
            'lxc-attach',
            '-n',
            self.container_name,
            '--clear-env',
        ]

        for var in env_list:
            cmd += ['-v', var]

        cmd.append('--')

        if root:
            if cwd:
                cmd += [
                    'sh',
                    '-c',
                    'mkdir -p %s ; cd %s ; %s' % (
                        shlex.quote(cwd),
                        shlex.quote(cwd),
                        ' '.join(map(shlex.quote, args)),
                    ),
                ]
            else:
                cmd += args
        else:
            if cwd:
                cmd += [
                    'su',
                    '-',
                    self.user,
                    '-c',
                    'env - %s sh -c %s' % (
                        ' '.join(env_list),
                        shlex.quote('mkdir -p %s ; cd %s ; %s' % (
                            shlex.quote(cwd),
                            shlex.quote(cwd),
                            ' '.join(map(shlex.quote, args)),
                        )),
                    ),
                ]
            else:
                cmd += [
                    'su',
                    '-',
                    self.user,
                    '-c',
                    'env - %s sh -c %s' % (
                        ' '.join(env_list),
                        shlex.quote(' '.join(map(shlex.quote, args))),
                    ),
                ]

        if output:
            return self.run_output(cmd, stdin=stdin)
        else:
            return self.run(cmd, stdin=stdin)

    def lxc_write(self, path, data, cwd='', root=False):
        """Write to a file in the LXC build container.

        :param str path: the file to write to
        :param str data: the data to write
        :param str cwd: the directory that path is relative to (default: '')
        :param bool root: True to write as root in the container (default:
                          False)
        """
        stdin, stdout = os.pipe()
        os.write(stdout, data.encode('utf-8'))
        os.close(stdout)

        self.lxc_run([
            'mkdir',
            '-p',
            os.path.dirname(path),
        ], cwd=cwd, root=root)

        return self.lxc_run([
            'dd',
            'of=%s' % shlex.quote(path),
        ], cwd=cwd, root=root, stdin=stdin)

    def jhbuild_run(self, args, output=False):
        """Run a JHBuild command in the LXC build container.

        :param list args: the JHBuild command as a list of tokens
        :param bool output: True to return the output of the command (default:
                            False)
        """
        return self.lxc_run([
            'jhbuild',
            '-f',
            self.jhbuildrc,
        ] + args, output=output)

    def lookup_providers(self, package):
        """Get a list of all packages that provide a virtual package.

        :param str package: the virtual package to query
        """
        lines = self.lxc_run([
            'apt-cache',
            'showpkg',
            package,
        ], output=True).splitlines()

        providers = []
        processing = False

        for line in lines:
            if not processing and line.strip() == 'Reverse Provides:':
                processing = True
            elif processing:
                providers.append(line.split()[0])

        return providers

    def lookup_stagedeps(self, builddeps):
        """Get a list of all runtime packages needed for a list of build packages.

        :param list builddeps: the list of build packages to query
        """
        builddeps = builddeps.copy()
        processed = set(builddeps)
        stagedeps = []

        while builddeps:
            builddep = builddeps.pop()

            # TODO: should we check if it's in libdevel instead?
            if builddep.endswith('-dev'):
                lines = self.lxc_run([
                    'apt-cache',
                    'depends',
                    '-i',
                    builddep,
                ], output=True).splitlines()

                lines = [line.strip() for line in lines
                         if line.find('Depends:') != -1]
                skip = False

                for line in lines:
                    alternative = skip
                    skip = line.startswith('|')

                    # skip alternative dependencies
                    if alternative:
                        continue

                    package = line.split(':', 1)[1].strip()
                    virtual = False

                    if package.startswith('<') and package.endswith('>'):
                        package = package[1:-1]
                        virtual = True

                    # trim the architecture if any
                    package = package.split(':')[0]

                    if virtual:
                        providers = self.lookup_providers(package)

                        if providers:
                            package = providers[0]
                            virtual = False
                        else:
                            logger.warning('No package found for \'%s\''
                                           % package)
                            skip = False

                    if package not in processed:
                        builddeps.append(package)
                        processed.add(package)
            else:
                stagedeps.append(builddep)

        return list(set(stagedeps))

    def lookup_sysdep_deps(self, sysdep):
        """Get all build packages and runtime packages needed to satisfy a
        system dependency.

        Return a pair of lists. The first is the list of build packages. The
        second is the list of runtime packages.

        :param str sysdep: the system dependency to satisfy
        """
        builddeps = None

        if sysdep in SYSDEP_EXCEPTIONS:
            builddeps = SYSDEP_EXCEPTIONS[sysdep]
        else:
            type, name = sysdep.split(':', 1)
            name = name.lstrip('./')

            if type == 'c_include':
                lines = self.lxc_run([
                    'apt-file',
                    'search',
                    '/usr/include/%s' % name,
                ], root=True, output=True).splitlines()

                builddeps = [line[:line.index(':')] for line in lines
                             if line.endswith('/usr/include/%s' % name)]
            elif type == 'path':
                lines = self.lxc_run([
                    'apt-file',
                    'search',
                    'bin/%s' % name,
                ], root=True, output=True).splitlines()

                builddeps = [line[:line.index(':')] for line in lines
                             if line.endswith('bin/%s' % name)]
            elif type == 'pkgconfig':
                lines = self.lxc_run([
                    'apt-file',
                    'search',
                    '/%s.pc' % name,
                ], root=True, output=True).splitlines()

                builddeps = [line[:line.index(':')] for line in lines
                             if line.endswith('/%s.pc' % name)]
            elif type == 'python2':
                builddeps = ['python-%s' % name]
            elif type == 'xml':
                pass
            else:
                raise KeyError('Unknown dependency type \'%s\'' % sysdep)

            n = len(builddeps or [])

            if n == 0:
                logger.warning('No package found for \'%s\'' % sysdep)
            elif n > 1:
                quoted_builddeps = ['\'%s\'' % builddep
                                    for builddep in builddeps]
                logger.warning('Multiple packages found for \'%s\': %s'
                               % (sysdep, ', '.join(quoted_builddeps)))

        if builddeps is None:
            raise KeyError('No package found for \'%s\'' % sysdep)

        builddeps = list(set(builddeps))
        stagedeps = self.lookup_stagedeps(builddeps)

        return builddeps, stagedeps

    def lookup_module_deps(self, modules):
        """Get all build packages and runtime packages needed to satisfy a list
        of JHBuild modules.

        Return a pair of lists. The first is the list of build packages. The
        second is the list of runtime packages.

        :param list modules: the list of JHBuild modules to query
        """
        build_packages = []
        stage_packages = []

        self.lxc_run([
            'apt-file',
            'update',
        ], root=True)

        lines = self.jhbuild_run([
            'sysdeps',
            '--dump-all',
        ] + modules, output=True).splitlines()

        for line in lines:
            found = False

            for sysdep in line.split(','):
                logger.info('Finding dependencies for \'%s\'' % sysdep)

                try:
                    builddeps, stagedeps = self.lookup_sysdep_deps(sysdep)

                    if builddeps:
                        build_packages += builddeps
                    if stagedeps:
                        stage_packages += stagedeps

                    found = True
                    break
                except KeyError as e:
                    logger.warning(str(e))

            if not found:
                raise KeyError('No package found for \'%s\'' % line)

        return build_packages, stage_packages

    def lxc_bootstrap(self):
        """Install the necessary build dependencies in the LXC container and
        update the list of required runtime dependencies."""
        build_packages = []

        # add explicit dependencies from snapcraft.yaml
        if self.options.container_packages:
            packages = ['\'%s\'' % package
                        for package in self.options.container_packages]
            logger.info('Finding dependencies for %s' % ', '.join(packages))

            build_packages += self.options.container_packages
            stagedeps = self.lookup_stagedeps(self.options.container_packages)
            self.stage_packages += stagedeps

        # add dependencies that JHBuild doesn't know about
        lines = self.jhbuild_run([
            'list',
        ] + self.modules, output=True).splitlines()

        for line in lines:
            if line in MODULE_EXCEPTIONS:
                logger.info('Finding dependencies for \'%s\'' % line)

                builddeps = MODULE_EXCEPTIONS[line]
                stagedeps = self.lookup_stagedeps(builddeps)

                build_packages += builddeps
                self.stage_packages += stagedeps

        # add dependencies that JHBuild knows about
        builddeps, stagedeps = self.lookup_module_deps(self.modules)
        build_packages = list(set(build_packages + builddeps))
        self.stage_packages = list(set(self.stage_packages + stagedeps))

        self.lxc_run([
            'apt-get',
            'install',
            '-y',
        ] + build_packages, root=True)

    def pull(self):
        if self.options.disable_pull:
            return

        logger.info('Creating container \'%s\'' % self.container_name)

        self.lxc_create()

        logger.info('Updating container \'%s\'' % self.container_name)

        self.lxc_run([
            'apt-get',
            'update',
        ], root=True)

        self.lxc_run([
            'apt-get',
            'install',
            '-y',
        ] + BUILD_PACKAGES, root=True)

        logger.info('Setting up JHBuild')

        try:
            self.lxc_run([
                'git',
                'clone',
                'https://git.gnome.org/browse/jhbuild',
                self.jhsource,
            ])
        except:
            self.lxc_run([
                'git',
                'pull',
            ], cwd=self.jhsource)

        self.lxc_run([
            os.path.join('.', 'autogen.sh'),
            '--prefix=%s' % self.prefix,
        ], cwd=self.jhsource)

        self.lxc_run([
            'make',
        ], cwd=self.jhsource)

        self.lxc_run([
            'make',
            'install',
        ], cwd=self.jhsource)

        if self.skipped_modules:
            skipped = ['\'%s\'' % module for module in self.skipped_modules]
            skip = 'skip = [%s]\n' % ', '.join(skipped)
        else:
            skip = ''

        self.lxc_write(self.jhbuildrc, (
            'moduleset = \'%s\'\n' % self.options.moduleset +
            'tarballdir = \'%s\'\n' % self.jhtarballs +
            'dvcs_mirror_dir = \'%s\'\n' % self.jhmirror +
            'checkoutroot = \'%s\'\n' % self.jhcheckout +
            'prefix = \'%s\'\n' % self.prefix +
            'use_local_modulesets = True\n' +
            'nonetwork = True\n' +
            skip +
            # TODO: use custom /etc/fonts/fonts.conf
            'os.environ[\'XDG_DATA_HOME\'] = \'%s\'\n' % self.xdgdatahome +
            'module_autogenargs[\'gdk-pixbuf\'] = \'--disable-gio-sniffing\'\n'
        ))

        logger.info('Installing dependencies')

        self.lxc_bootstrap()

        if not self.options.disable_jhupdate:
            logger.info('Downloading modules')

            self.jhbuild_run([
                'update',
            ] + self.modules)

        self.lxc_stop()

    def clean_pull(self):
        logger.info('Destroying container \'%s\'' % self.container_name)

        self.lxc_stop()

        self.run([
            'lxc-destroy',
            '-n',
            self.container_name,
            '-s',
        ])

        super().clean_pull()

    def build(self):
        if self.options.disable_build:
            return

        self.lxc_start()

        logger.info('Building modules')

        self.jhbuild_run([
            'build',
        ] + self.modules)

        self.lxc_stop()

    def clean_build(self):
        pass

    def snap_fileset(self):
        return os.listdir(self.installdir)
