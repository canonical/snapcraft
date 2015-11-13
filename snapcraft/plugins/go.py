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
"""

import os
import shutil

import snapcraft


class GoPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('golang-go')
        self._gopath = os.path.join(self.partdir, 'go')
        self._gopath_src = os.path.join(self._gopath, 'src')
        self._gopath_bin = os.path.join(self._gopath, 'bin')

    def env(self, root):
        # usr/lib/go/bin on newer Ubuntus, usr/bin on trusty
        env = [
            'GOPATH={}/go'.format(root),
            'CGO_LDFLAGS=$CGO_LDFLAGS"' + ' '.join([
                '-L{0}/lib',
                '-L{0}/usr/lib',
                '-L{0}/lib/{1}',
                '-L{0}/usr/lib/{1}',
                '$LDFLAGS'
            ]).format(root, snapcraft.common.get_arch_triplet()) + '"',
        ]
        return env

    def pull(self):
        # use -d to only download (build will happen later)
        # use -t to also get the test-deps
        super().pull()
        os.makedirs(self._gopath_src, exist_ok=True)
        if self.options.source is not None:
            self._local_pull()

    def _local_pull(self):
        go_package = os.path.basename(os.path.abspath(self.options.source))
        local_path = os.path.join(self._gopath_src, go_package)
        if os.path.islink(local_path):
            os.unlink(local_path)
        os.symlink(self.sourcedir, local_path)
        self._run(['go', 'get', '-t', '-d', './{}/...'.format(go_package)])

    def build(self):
        super().build()
        if self.options.source is not None:
            self._local_build()

        install_bin_path = os.path.join(self.installdir, 'bin')
        os.makedirs(install_bin_path, exist_ok=True)
        os.makedirs(self._gopath_bin, exist_ok=True)
        for binary in os.listdir(os.path.join(self._gopath_bin)):
            binary_path = os.path.join(self._gopath_bin, binary)
            shutil.copy2(binary_path, install_bin_path)

    def _local_build(self):
        go_package = os.path.basename(os.path.abspath(self.options.source))
        self._run(['go', 'install', './{}/...'.format(go_package)])

    def _run(self, cmd, **kwargs):
        cmd = ['env', 'GOPATH={}'.format(self._gopath)] + cmd
        return self.run(cmd, cwd=self._gopath_src, **kwargs)
