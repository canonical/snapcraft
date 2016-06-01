# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

"""The go plugin can be used for go projects using `go get`.

This plugin uses the common plugin keywords, for more information check the
'plugins' topic.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - go-packages:
      (list of strings)
      Go packages to fetch, these must be a "main" package. Dependencies
      are pulled in automatically by `go get`.
      Packages that are not "main" will not cause an error, but would
      not be useful either.

    - go-importpath:
      (string)
      This entry tells the checked out `source` to live within a certain path
      within `GOPATH`.
      This is not needed for `go-packages`.
"""

import os
import shutil

import snapcraft


class GoPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['go-packages'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }
        schema['properties']['go-importpath'] = {
            'type': 'string',
            'default': ''
        }

        if 'required' in schema:
            del schema['required']

        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        schema['pull-properties'].append('go-packages')

        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['build-properties'].extend(['source', 'go-packages'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('golang-go')
        self._gopath = os.path.join(self.partdir, 'go')
        self._gopath_src = os.path.join(self._gopath, 'src')
        self._gopath_bin = os.path.join(self._gopath, 'bin')
        self._gopath_pkg = os.path.join(self._gopath, 'pkg')

        if self.options.source and self.options.go_importpath:
            self.sourcedir = os.path.join(self._gopath_src,
                                          self.options.go_importpath)

    def env(self, root):
        # usr/lib/go/bin on newer Ubuntus, usr/bin on trusty
        env = [
            'GOPATH={}/go'.format(root),
            'CGO_LDFLAGS="$CGO_LDFLAGS ' + ' '.join([
                '-L{0}/lib',
                '-L{0}/usr/lib',
                '-L{0}/lib/{1}',
                '-L{0}/usr/lib/{1}',
                '$LDFLAGS']).format(root, self.project.arch_triplet) + '"',
        ]
        return env

    def pull(self):
        # use -d to only download (build will happen later)
        # use -t to also get the test-deps
        super().pull()
        os.makedirs(self._gopath_src, exist_ok=True)
        if self.options.source is not None:
            self._local_pull()
        self._remote_pull()

    def clean_pull(self):
        super().clean_pull()

        # Remove the gopath (if present)
        if os.path.exists(self._gopath):
            shutil.rmtree(self._gopath)

    def _local_pull(self):
        if self.options.go_importpath:
            go_package = self.options.go_importpath
        else:
            go_package = os.path.basename(
               os.path.abspath(self.options.source))
            local_path = os.path.join(self._gopath_src, go_package)
            if os.path.islink(local_path):
                os.unlink(local_path)
            os.symlink(self.sourcedir, local_path)
        self._run(['go', 'get', '-t', '-d', './{}/...'.format(go_package)])

    def _remote_pull(self):
        for go_package in self.options.go_packages:
            self._run(['go', 'get', '-t', '-d', go_package])

    def build(self):
        super().build()
        if self.options.source is not None:
            self._local_build()
        self._remote_build()

        install_bin_path = os.path.join(self.installdir, 'bin')
        os.makedirs(install_bin_path, exist_ok=True)
        os.makedirs(self._gopath_bin, exist_ok=True)
        for binary in os.listdir(os.path.join(self._gopath_bin)):
            binary_path = os.path.join(self._gopath_bin, binary)
            shutil.copy2(binary_path, install_bin_path)

    def clean_build(self):
        super().clean_build()

        if os.path.isdir(self._gopath_bin):
            shutil.rmtree(self._gopath_bin)

        if os.path.isdir(self._gopath_pkg):
            shutil.rmtree(self._gopath_pkg)

    def _local_build(self):
        if self.options.go_importpath:
            go_package = self.options.go_importpath
        else:
            go_package = os.path.basename(os.path.abspath(self.options.source))
        self._run(['go', 'install', './{}/...'.format(go_package)])

    def _remote_build(self):
        for go_package in self.options.go_packages:
            self._run(['go', 'install', go_package])

    def _run(self, cmd, **kwargs):
        cmd = ['env', 'GOPATH={}'.format(self._gopath)] + cmd
        return self.run(cmd, cwd=self._gopath_src, **kwargs)
