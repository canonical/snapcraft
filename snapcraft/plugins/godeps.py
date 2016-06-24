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

"""The godeps plugin can be used for godeps-enabled go projects.

These projects have a file containing information about the project's
dependencies. This file is typically called "dependencies.tsv," but may be
named anything.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - godeps-file:
      (string)
      Path to the godeps dependencies file contained within the source
      (default: dependencies.tsv)

    - go-importpath:
      (string)
      This entry tells the checked out `source` to live within a certain path
      within `GOPATH`. This is not required, but is useful if absolute imports
      or import path checking is used.
"""

import logging
import os
import shutil

import snapcraft


logger = logging.getLogger(__name__)


class GodepsPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['godeps-file'] = {
            'type': 'string',
            'default': 'dependencies.tsv'
        }
        schema['properties']['go-importpath'] = {
            'type': 'string',
            'default': ''
        }

        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        schema['pull-properties'].extend(['godeps-file', 'go-importpath'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('golang-go')
        self._gopath = os.path.join(self.partdir, 'go')
        self._gopath_src = os.path.join(self._gopath, 'src')
        self._gopath_bin = os.path.join(self._gopath, 'bin')
        self._gopath_pkg = os.path.join(self._gopath, 'pkg')

    def pull(self):
        super().pull()

        if self.options.go_importpath:
            go_package = self.options.go_importpath
        else:
            logger.warning(
                'Please consider setting `go-importpath` for the {!r} '
                'part'.format(self.name))
            go_package = os.path.basename(
                os.path.abspath(self.options.source))

        path_in_gopath = os.path.join(self._gopath_src, go_package)
        os.makedirs(os.path.dirname(path_in_gopath), exist_ok=True)
        if os.path.islink(path_in_gopath):
            os.unlink(path_in_gopath)
        os.symlink(self.sourcedir, path_in_gopath)

        # Fetch and run godeps
        logger.info('Fetching godeps...')
        self._run(['go', 'get', 'github.com/rogpeppe/godeps'])

        logger.info('Obtaining project dependencies...')
        self._run(['godeps', '-t', '-u', os.path.join(
            self.sourcedir, self.options.godeps_file)])

    def clean_pull(self):
        super().clean_pull()

        # Remove the gopath (if present)
        if os.path.exists(self._gopath):
            shutil.rmtree(self._gopath)

    def build(self):
        super().build()

        if self.options.go_importpath:
            go_package = self.options.go_importpath
        else:
            go_package = os.path.basename(os.path.abspath(self.options.source))
        self._run(['go', 'install', './{}/...'.format(go_package)])

        install_bin_path = os.path.join(self.installdir, 'bin')
        os.makedirs(install_bin_path, exist_ok=True)
        os.makedirs(self._gopath_bin, exist_ok=True)
        for binary in os.listdir(os.path.join(self._gopath_bin)):
            # Skip godeps. It serves no purpose in production.
            if binary == 'godeps':
                continue

            binary_path = os.path.join(self._gopath_bin, binary)
            shutil.copy2(binary_path, install_bin_path)

    def clean_build(self):
        super().clean_build()

        if os.path.isdir(self._gopath_bin):
            shutil.rmtree(self._gopath_bin)

        if os.path.isdir(self._gopath_pkg):
            shutil.rmtree(self._gopath_pkg)

    def _run(self, cmd, **kwargs):
        env = self._build_environment()
        return self.run(cmd, cwd=self._gopath_src, env=env, **kwargs)

    def _build_environment(self):
        env = os.environ.copy()
        env['GOPATH'] = self._gopath
        env['PATH'] = '{}:{}'.format(
            os.path.join(self._gopath, 'bin'), env.get('PATH', ''))
        linker_flags = ' '.join([
            '-L{0}/lib',
            '-L{0}/usr/lib',
            '-L{0}/lib/{1}',
            '-L{0}/usr/lib/{1}']).format(self.installdir,
                                         self.project.arch_triplet)
        env['CGO_LDFLAGS'] = '{} {} {}'.format(
            env.get('CGO_LDFLAGS', ''), linker_flags, env.get('LDFLAGS', ''))

        return env
