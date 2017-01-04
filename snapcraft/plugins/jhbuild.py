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
  - snap-name: (required) the name of the snap this part is included in
  - snap-revision: the revision of the snap this part is included in
  - jhbuild-archive: the source tarball directory on the build host
  - jhbuild-mirror: the DVCS repository directory on the build host
  - ccache-cache: the ccache directory on the build host

Advice:

  - You only need to specify a single module in most cases. JHBuild knows the
    module dependency graph.

  - The command for launching an app from a JHBuild part is:
    jhbuild -f $SNAP/etc/jhbuildrc run <app>

    So for a glade snap, the app would look like:

    apps:
      glade:
        command: jhbuild -f $SNAP/etc/jhbuildrc run glade
        plugs:
          - unity7

  - Building WebKit requires a lot of time and computing power. If WebKit is a
    dependency of your JHBuild module, it can be skipped by adding '-WebKit' to
    the list of modules, and adding 'libwebkit2gtk-4.0-dev' to the
    build-packages and stage-packages.

  - Specify directories on the build host for jhbuild-archive and
    jhbuild-mirror to prevent repeated downloading of the JHBuild module
    sources. It's best to reserve common directories on your local machine that
    can be reused by all snaps you might want to build.

  - If you find yourself re-building the same modules over-and-over again, you
    can specify a local ccache directory on the build host using ccache-cache.

  - To add a shell for debugging your snap, add an extra app whose command is:
    jhbuild -f $SNAP/etc/jhbuildrc shell. You might also want to add gdb,
    strace, etc. to your list of stage-packages for debugging purposes.
"""

import logging
import os
import json
import shlex
import shutil
import snapcraft

HOME = os.path.expanduser('~') if os.geteuid() else os.path.join('/', 'root')

BUILD_PACKAGES = {
    'git',
    'ca-certificates',
    'ccache',
    'make',
    'autoconf',
    'automake',
    'gettext',
    'pkg-config',
    'yelp-tools',
    'autopoint',
    'python',
    'libtool',
    'docbook-xsl',
    'libxml-parser-perl',
    'cvs',
    'subversion',
    'flex',
    'bison',
    'apt-file',
    'ninja-build',
    'bubblewrap',
    'symlinks',
}

STAGE_PACKAGES = {
    'python',
}

SYSDEP_EXCEPTIONS = {
    'c_include:boost/variant.hpp': {'libboost-dev'},
    'c_include:jpeglib.h': {'libjpeg-dev'},
    'c_include:readline/readline.h': {'libreadline-dev'},
    'c_include:tiff.h': {'libtiff-dev'},
    'path:automake': {'automake'},
    'path:bison': {'bison'},
    'path:bogofilter': {'bogofilter-bdb'},
    'path:c++': {'g++'},
    'path:cc': {'gcc'},
    'path:flex': {'flex'},
    'path:iptables': {'iptables'},
    'path:krb5-config': {'libkrb5-dev'},
    'path:make': {'make'},
    'path:pkg-config': {'pkg-config'},
    'path:rapper': {'raptor2-utils'},
    'pkgconfig:dbus-1': {'libdbus-1-dev'},
    'pkgconfig:neon': {'libneon27-gnutls-dev'},
    'pkgconfig:zlib': {'zlib1g-dev'},
    'xml:-//OASIS//DTD DocBook XML V4.3//EN': {'docbook-xml'},
    'xml:http://docbook.sourceforge.net/release/xsl/current/': {'docbook-xsl'},
}

VIRTUAL_PACKAGE_EXCEPTIONS = {
    'python': 'python',
    'python3': 'python3',
}

ROOT_PATCH = """\
diff --git a/jhbuild/main.py b/jhbuild/main.py
index a5cf99b..bab8136 100644
--- a/jhbuild/main.py
+++ b/jhbuild/main.py
@@ -94,10 +94,6 @@ def main(args):
         localedir = None
     gettext.install('jhbuild', localedir=localedir, unicode=True)
 \n\
-    if hasattr(os, 'getuid') and os.getuid() == 0:
-        sys.stderr.write(_('You should not run jhbuild as root.\\n').encode(\
_encoding, 'replace'))
-        sys.exit(1)
-
     logging.getLogger().setLevel(logging.INFO)
     logging_handler = logging.StreamHandler()
     logging_handler.setFormatter(LoggingFormatter())
"""

logger = logging.getLogger(__name__)


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
            'module-set': {
                'type': 'string',
                'default': 'gnome-world',
            },
            'module-set-dir': {
                'type': 'string',
            },
            'snap-name': {
                'type': 'string',
            },
            'snap-revision': {
                'type': 'string',
                'default': 'current',
            },
            'jhbuild-archive': {
                'type': 'string',
            },
            'jhbuild-mirror': {
                'type': 'string',
            },
            'ccache-cache': {
                'type': 'string',
            },
        }

        schema['required'] = [
            'modules',
            'snap-name',
        ]

        return schema

    @classmethod
    def get_pull_properties(cls):
        return [
            'modules',
            'module-set',
            'module-set-dir',
            'snap-name',
            'snap-revision',
            'jhbuild-archive',
            'jhbuild-mirror',
        ]

    @classmethod
    def get_build_properties(cls):
        return [
            'ccache-cache',
        ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.modules = [
            module
            for module in self.options.modules
            if module[0] != '-'
        ]

        self.skip_modules = [
            module[1:]
            for module in self.options.modules
            if module[0] == '-'
        ]

        self.build_packages += list(BUILD_PACKAGES)
        self.stage_packages += list(STAGE_PACKAGES)

    def run(self, args, env={}, input=None, **kwargs):
        """Run a command.

        Use run_output() if you need to capture or suppress output.

        :param list args: command arguments, first is the executable
        :param dict env: environment variables and their values
        :param str input: text to pipe into stdin
        """
        env['HOME'] = HOME
        env['PATH'] = '/usr/lib/ccache:/usr/bin:/usr/sbin:/bin:/sbin'

        if input:
            kwargs['stdin'], stdout = os.pipe()
            os.write(stdout, input.encode())
            os.close(stdout)

        status = super().run([
            'env',
            '-',
        ] + [
            '%s=%s' % (shlex.quote(var), shlex.quote(str(env[var])))
            for var in env
            if env[var] is not None
        ] + args, **kwargs)

        if input:
            os.close(kwargs['stdin'])

        return status

    def run_output(self, args, env={}, input=None, **kwargs):
        """Run a command, capturing its output.

        Use run() if you don't need the output and prefer it to be piped to
        stdout.

        :param list args: command arguments, first is the executable
        :param dict env: environment variables and their values
        :param str input: text to pipe into stdin
        :return: the output of the command
        :rtype: str
        """
        env['HOME'] = HOME
        env['PATH'] = '/usr/lib/ccache:/usr/bin:/usr/sbin:/bin:/sbin'

        if input:
            kwargs['stdin'], stdout = os.pipe()
            os.write(stdout, input.encode())
            os.close(stdout)

        output = super().run_output([
            'env',
            '-',
        ] + [
            '%s=%s' % (shlex.quote(var), shlex.quote(str(env[var])))
            for var in env
            if env[var] is not None
        ] + args, **kwargs)

        if input:
            os.close(kwargs['stdin'])

        return output

    def jhpull(self, args, output=False, **kwargs):
        """Run JHBuild in the pull stage.

        :param list args: command arguments without executable
        :param bool output: True to capture and suppress output
        :return: the output of the command if captured
        :rtype: str
        """
        return (self.run_output if output else self.run)([
            self.jhpull_program,
            '-f',
            os.path.join(
                self.sourcedir,
                'jhbuildrc',
            ),
        ] + args, **kwargs)

    def bwrap(self, args, cwd='', output=False, **kwargs):
        """Run a command inside bubblewrap.

        :param list args: command arguments, first is the executable
        :param str cwd: current working directory
        :param bool output: True to capture and suppress output
        :return: the output of the command if captured
        :rtype: str
        """
        if not hasattr(self, 'bwrap_args'):
            mounts = [
                (os.path.join('/', dir), os.path.join('/', dir))
                for dir in os.listdir('/')
                if dir != 'snap'
            ] + [
                (
                    self.installdir,
                    os.path.join(
                        '/',
                        'snap',
                        self.options.snap_name,
                        self.options.snap_revision,
                    )
                ),
                (
                    (
                        self.options.jhbuild_archive or
                        os.path.join(self.sourcedir, 'archive')
                    ),
                    os.path.join(
                        HOME,
                        'snap',
                        self.options.snap_name,
                        'common',
                        '.cache',
                        'archive',
                    )
                ),
                (
                    (
                        self.options.jhbuild_mirror or
                        os.path.join(self.sourcedir, 'mirror')
                    ),
                    os.path.join(
                        HOME,
                        'snap',
                        self.options.snap_name,
                        'common',
                        '.cache',
                        'mirror',
                    )
                ),
                (
                    os.path.join(self.sourcedir, 'source'),
                    os.path.join(
                        HOME,
                        'snap',
                        self.options.snap_name,
                        'common',
                        '.cache',
                        'source',
                    )
                ),
                (
                    self.builddir,
                    os.path.join(
                        HOME,
                        'snap',
                        self.options.snap_name,
                        'common',
                        '.cache',
                        'build',
                    )
                ),
                (
                    (
                        self.options.ccache_cache or
                        os.path.join(self.builddir, '.cache')
                    ),
                    os.path.join(
                        HOME,
                        'snap',
                        self.options.snap_name,
                        'common',
                        '.cache',
                        'cache',
                    )
                ),
            ]

            self.bwrap_args = ['bwrap']

            for mount in mounts:
                self.bwrap_args += [
                    '--dev-bind',
                    mount[0],
                    mount[1],
                ]

        env = kwargs.get('env', {})
        env['CCACHE_DIR'] = os.path.join(
            HOME,
            'snap',
            self.options.snap_name,
            'common',
            '.cache',
            'cache',
        )
        kwargs['env'] = env

        if cwd:
            args = [
                'sh',
                '-c',
                'mkdir -p %s ; cd %s ; %s' % (
                    shlex.quote(cwd),
                    shlex.quote(cwd),
                    ' '.join(map(shlex.quote, args))
                ),
            ]

        return (self.run_output if output else self.run)(
            self.bwrap_args + args,
            **kwargs
        )

    def jhbuild(self, args, **kwargs):
        """Run JHBuild in the build stage.

        :param list args: command arguments without executable
        :return: the output of the command if captured
        :rtype: str
        """
        return self.bwrap([
            self.jhbuild_program,
            '-f',
            os.path.join(
                '/',
                'snap',
                self.options.snap_name,
                self.options.snap_revision,
                'etc',
                'jhbuildrc',
            ),
        ] + args, **kwargs)

    def find_packages(self, query):
        """Find packages that provide a file.

        :param str query: the file to find
        :return: the packages that provide the file
        :rtype: set
        """
        if not getattr(self, 'apt_file_updated', False):
            logger.info('Updating apt-file, please wait...')

            self.run_output([
                'apt-file',
                'update',
            ])

            self.apt_file_updated = True

        return {
            line.split(':', 1)[0]

            for line in self.run_output([
                'apt-file',
                'search',
                query,
            ]).splitlines()

            if line.endswith(query)
        }

    def find_build_packages(self, sysdep):
        """Find packages satisfying a sysdep.

        :param str sysdep: the sysdep to satisfy
        :return: the packages that satisfy sysdep
        :rtype: set
        """
        if not hasattr(self, 'apt_file_cache'):
            self.apt_file_cache = {}

        if sysdep not in self.apt_file_cache:
            type, name = sysdep.split(':', 1)

            if sysdep in SYSDEP_EXCEPTIONS:
                packages = SYSDEP_EXCEPTIONS[sysdep] or None
            elif type == 'c_include':
                packages = self.find_packages('/usr/include/%s' % name)
            elif type == 'path':
                packages = self.find_packages('/usr/bin/%s' % name)
            elif type == 'pkgconfig':
                packages = self.find_packages('/%s.pc' % name) - {'emscripten'}
            elif type == 'python2':
                packages = {'python-%s' % name}
            elif type == 'xml':
                packages = set()
            else:
                raise KeyError('Unknown dependency type \'%s\'' % sysdep)

            if packages:
                self.apt_file_cache[sysdep] = list(sorted(packages))
            else:
                self.apt_file_cache[sysdep] = []

        return set(self.apt_file_cache[sysdep])

    def is_virtual_package(self, package):
        """Check if a package is virtual.

        :param str package: the package to check
        :return: True if package is virtual
        :rtype: bool
        """
        return package[0] == '<' and package[-1] == '>'

    def is_build_package(self, package):
        """Check if a package is used for development.

        :param str package: the package to check
        :return: True if package is used for development
        :rtype: bool
        """
        return package.endswith('-dev')

    def find_provider(self, package):
        """Find a provider for a virtual package.

        :param str package: the virtual package to find a provider for
        :return: a provider for the virtual package
        :rtype: str
        """
        if package in VIRTUAL_PACKAGE_EXCEPTIONS:
            return VIRTUAL_PACKAGE_EXCEPTIONS[package]

        lines = [line.strip() for line in self.run_output([
            'apt-cache',
            'showpkg',
            package,
        ]).splitlines()]

        lines = lines[lines.index('Reverse Provides:') + 1:]

        if not lines:
            raise KeyError('No provider for virtual package \'%s\'' % package)

        if len(lines) > 1:
            logger.warning(
                'Multiple providers for virtual package \'%s\'' % package
            )

        return lines[0].split(maxsplit=1)[0]

    def find_dependencies(self, package, recommends=True, suggests=False):
        """Find the dependencies of a package.

        :param str package: the package whose dependencies to find
        :param bool recommends: True to also find recommended packages
        :param bool suggests: True to also find suggested packages
        :return: the dependencies of package
        :rtype: set
        """
        packages = set()

        lines = self.run_output([
            'apt-cache',
            'depends',
            package,
        ]).splitlines()

        skip = False

        for line in lines:
            line = line.strip()

            relevant = (
                line.startswith('PreDepends: ') or
                line.startswith('Depends: ') or
                (recommends and line.startswith('Recommends: ')) or
                (suggests and line.startswith('Suggests: '))
            )

            if not relevant:
                continue

            prev_skip = skip
            skip = line[0] == '|'

            if prev_skip:
                continue

            packages.add(line.split(': ', 1)[1])

        return packages

    def find_stage_packages(self, build_package):
        """Find the packages that should be staged for a build package.

        :param str build_package: the package whose stage packages to find
        :return: the packages that should be staged
        :rtype: set
        """
        build_packages = {build_package}
        stage_packages = set()
        finished = set()

        while build_packages:
            package = build_packages.pop()

            if package in finished:
                continue

            finished.add(package)

            if self.is_virtual_package(package):
                package = package[1:-1].split(':', 1)[0]
                build_packages.add(self.find_provider(package))
            elif self.is_build_package(package):
                build_packages |= self.find_dependencies(package)
            else:
                stage_packages.add(package)

        return stage_packages

    def scan_apt_file(self, sysdeps):
        """Find the set of build packages satisfying a set of sysdeps.

        :param list sysdeps: the sysdeps to satisfy
        :return: the build packages satisfying sysdeps
        :rtype: set
        """
        build_packages = set()

        for line in sysdeps:
            for sysdep in line.split(','):
                logger.info('Looking for \'%s\'' % sysdep)

                try:
                    packages = self.find_build_packages(sysdep)

                    if packages is None:
                        break

                    if packages:
                        build_packages |= packages
                        break
                except KeyError as e:
                    logger.warning(str(e))
            else:
                logger.warning('No package found for \'%s\'' % line)

        return build_packages

    def find_module_dependencies(self, modules):
        """Find the build and stage packages for a set of modules.

        :param list modules: the modules to find dependencies for
        """
        os.makedirs(os.path.join(self.sourcedir, '.cache'), exist_ok=True)

        build_packages = set()
        build_packages_file = os.path.join(
            self.sourcedir,
            '.cache',
            'build-packages',
        )

        sysdeps = self.jhpull([
            'sysdeps',
            '--dump-all',
        ] + modules, output=True).splitlines()

        if os.path.isfile(build_packages_file):
            with open(build_packages_file, 'r') as f:
                self.apt_file_cache = json.load(f)

        build_packages = self.scan_apt_file(sysdeps)

        with open(build_packages_file, 'w') as f:
            json.dump(self.apt_file_cache, f, indent=4, sort_keys=True)

        stage_packages = set()
        stage_packages_file = os.path.join(
            self.sourcedir,
            '.cache',
            'stage-packages',
        )

        if os.path.isfile(stage_packages_file):
            with open(stage_packages_file, 'r') as f:
                apt_depends_cache = json.load(f)
        else:
            apt_depends_cache = {}

        for build_package in build_packages:
            if build_package not in apt_depends_cache:
                packages = self.find_stage_packages(build_package)
                apt_depends_cache[build_package] = list(sorted(packages))

            stage_packages |= set(apt_depends_cache[build_package])

        with open(stage_packages_file, 'w') as f:
            json.dump(apt_depends_cache, f, indent=4, sort_keys=True)

        self.build_packages += list(build_packages)
        self.stage_packages += list(stage_packages)

    def _pull_jhpull(self):
        logger.info('Pulling JHBuild')

        repository = 'https://git.gnome.org/browse/jhbuild'
        updated = False

        if self.options.jhbuild_mirror:
            checkout = os.path.join(
                self.options.jhbuild_mirror,
                'jhbuild.git',
            )

            if os.path.isdir(checkout):
                repository = checkout
                updated = True

                self.run([
                    'git',
                    'fetch',
                ], cwd=repository)

        if os.path.isdir(os.path.join(self.sourcedir, 'jhbuild')):
            self.run([
                'git',
                'pull',
                '--rebase',
                '--autostash',
            ], cwd=os.path.join(self.sourcedir, 'jhbuild'))
        else:
            self.run([
                'git',
                'clone',
                repository,
                os.path.join(self.sourcedir, 'jhbuild'),
            ], env={'https_proxy': os.getenv('https_proxy')})

            self.run([
                'git',
                'apply',
            ], cwd=os.path.join(
                self.sourcedir,
                'jhbuild',
            ), input=ROOT_PATCH)

        logger.info('Building JHBuild')

        env = {
            'CCACHE_DIR': (
                self.options.ccache_cache or
                os.path.join(self.sourcedir, '.cache', 'cache')
            ),
        }

        self.run([
            './autogen.sh',
            '--prefix=%s' % os.path.join(
                self.sourcedir,
                '.cache',
                'install',
            ),
        ], env=env, cwd=os.path.join(self.sourcedir, 'jhbuild'))

        self.run([
            'make',
            '-j%d' % self.parallel_build_count,
        ], env=env, cwd=os.path.join(self.sourcedir, 'jhbuild'))

        self.run([
            'make',
            '-j%d' % self.parallel_build_count,
            'install',
        ], env=env, cwd=os.path.join(self.sourcedir, 'jhbuild'))

        self.jhpull_program = os.path.join(
            self.sourcedir,
            '.cache',
            'install',
            'bin',
            'jhbuild',
        )

        if not updated:
            try:
                self.jhpull([
                    'updateone',
                    'jhbuild',
                ])
            except:
                pass

    def pull(self):
        self._pull_jhpull()

        if not os.path.isfile(os.path.join(self.sourcedir, 'jhbuildrc')):
            if not self.options.jhbuild_archive:
                os.makedirs(
                    os.path.join(self.sourcedir, 'archive'),
                    exist_ok=True
                )
            if not self.options.jhbuild_mirror:
                os.makedirs(
                    os.path.join(self.sourcedir, 'mirror'),
                    exist_ok=True
                )

            with open(os.path.join(self.sourcedir, 'jhbuildrc'), 'w') as f:
                f.write('moduleset = \'%s\'\n' % self.options.module_set)
                if self.options.module_set_dir:
                    f.write('modulesets_dir = \'%s\'\n' % (
                        self.options.module_set_dir
                    ))
                f.write('tarballdir = \'%s\'\n' % (
                    self.options.jhbuild_archive or
                    os.path.join(self.sourcedir, 'archive')
                ))
                f.write('dvcs_mirror_dir = \'%s\'\n' % (
                    self.options.jhbuild_mirror or
                    os.path.join(self.sourcedir, 'mirror')
                ))
                f.write('checkoutroot = \'%s\'\n' % (
                    os.path.join(self.sourcedir, 'source')
                ))
                f.write('buildroot = \'%s\'\n' % (
                    os.path.join(self.sourcedir, '.cache', 'build')
                ))
                f.write('prefix = \'%s\'\n' % (
                    os.path.join(self.sourcedir, '.cache', 'install')
                ))
                f.write('nonetwork = True\n')
                f.write('use_local_modulesets = True\n')
                f.write('skip = [%s]\n' % ', '.join([
                    '\'%s\'' % module for module in self.skip_modules
                ]))
                f.write(
                    'module_autogenargs[\'gdk-pixbuf\'] = '
                    '\'--disable-gio-sniffing\'\n'
                )
                f.write(
                    'os.environ[\'XDG_DATA_HOME\'] = \'%s\'\n' % (
                        os.path.join(
                            '/',
                            'snap',
                            self.options.snap_name,
                            self.options.snap_revision,
                            'usr',
                            'share',
                        )
                    )
                )

        self.jhpull([
            'sanitycheck',
        ])

        modules = self.jhpull([
            'list',
        ] + self.modules, output=True).splitlines()

        if 'gtk+' in modules or 'gtk+-3' in modules:
            self.modules.append('adwaita-icon-theme')
            self.stage_packages.append('ttf-ubuntu-font-family')

        logger.info('Finding dependencies')

        self.find_module_dependencies(self.modules)

        ubuntu = snapcraft.internal.repo.Ubuntu(
            os.path.join(self.partdir, 'ubuntu'),
            sources=self.PLUGIN_STAGE_SOURCES
        )

        ubuntu.get(self.stage_packages)
        ubuntu.unpack(self.installdir)

        logger.info('Pulling modules')

        self.jhpull([
            'update',
        ] + self.modules)

    def build(self):
        checkout = os.path.join(
            self.installdir,
            'usr',
            'src',
            'jhbuild',
        )

        if os.path.isdir(checkout):
            if shutil.rmtree.avoids_symlink_attacks:
                shutil.rmtree(checkout)

        shutil.copytree(os.path.join(self.sourcedir, 'jhbuild'), checkout)

        self.run([
            'git',
            'clean',
            '-d',
            '-f',
            '-x',
        ], cwd=checkout)

        logger.info('Building JHBuild')

        env = {
            'CCACHE_DIR': (
                self.options.ccache_cache or
                os.path.join(self.builddir, '.cache')
            ),
        }

        self.run([
            './autogen.sh',
            '--prefix=%s' % os.path.join(
                '/',
                'snap',
                self.options.snap_name,
                self.options.snap_revision,
            ),
        ], env=env, cwd=os.path.join(
            self.installdir,
            'usr',
            'src',
            'jhbuild',
        ))

        self.run([
            'make',
            '-j%d' % self.parallel_build_count,
        ], env=env, cwd=os.path.join(
            self.installdir,
            'usr',
            'src',
            'jhbuild',
        ))

        self.bwrap([
            'make',
            '-j%d' % self.parallel_build_count,
            'install',
        ], env=env, cwd=os.path.join(
            '/',
            'snap',
            self.options.snap_name,
            self.options.snap_revision,
            'usr',
            'src',
            'jhbuild',
        ))

        self.jhbuild_program = os.path.join(
            '/',
            'snap',
            self.options.snap_name,
            self.options.snap_revision,
            'bin',
            'jhbuild',
        )

        jhbuildrc = os.path.join(self.installdir, 'etc', 'jhbuildrc')

        if not os.path.isfile(jhbuildrc):
            os.makedirs(os.path.join(self.installdir, 'etc'), exist_ok=True)

            with open(jhbuildrc, 'w') as f:
                f.write('moduleset = \'%s\'\n' % self.options.module_set)
                if self.options.module_set_dir:
                    f.write('modulesets_dir = \'%s\'\n' % (
                        self.options.module_set_dir
                    ))
                f.write('tarballdir = \'%s\'\n' % os.path.join(
                    '~',
                    'snap',
                    self.options.snap_name,
                    'common',
                    '.cache',
                    'archive',
                ))
                f.write('dvcs_mirror_dir = \'%s\'\n' % os.path.join(
                    '~',
                    'snap',
                    self.options.snap_name,
                    'common',
                    '.cache',
                    'mirror',
                ))
                f.write('checkoutroot = \'%s\'\n' % os.path.join(
                    '~',
                    'snap',
                    self.options.snap_name,
                    'common',
                    '.cache',
                    'source',
                ))
                f.write('buildroot = \'%s\'\n' % os.path.join(
                    '~',
                    'snap',
                    self.options.snap_name,
                    'common',
                    '.cache',
                    'build',
                ))
                f.write('prefix = \'%s\'\n' % os.path.join(
                    '/',
                    'snap',
                    self.options.snap_name,
                    self.options.snap_revision,
                ))
                f.write('nonetwork = True\n')
                f.write('use_local_modulesets = True\n')
                f.write('skip = [%s]\n' % ', '.join([
                    '\'%s\'' % module
                    for module in self.skip_modules
                ]))
                f.write(
                    'module_autogenargs[\'gdk-pixbuf\'] = '
                    '\'--disable-gio-sniffing\'\n'
                )
                f.write(
                    'os.environ[\'XDG_DATA_HOME\'] = \'%s\'\n' % (
                        os.path.join(
                            '/',
                            'snap',
                            self.options.snap_name,
                            self.options.snap_revision,
                            'usr',
                            'share',
                        )
                    )
                )

        logger.info('Building modules')

        self.jhbuild([
            'build',
        ] + self.modules)

        logger.info('Fixing symbolic links')

        self.bwrap([
            'symlinks',
            '-c',
            '-d',
            '-r',
            '-s',
            '-v',
            os.path.join(
                '/',
                'snap',
                self.options.snap_name,
                self.options.snap_revision,
            ),
        ])
