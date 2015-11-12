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

Additionally, this plugin uses the following plugin specific keywords:

    - source:
      (string)
      A path to some source tree to build in the form of something `go get`
      understands.
"""

import os
import snapcraft


class GoPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            'properties': {
                'source': {
                    'type': 'string',
                },
            },
            'required': [
                'source',
            ]
        }

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('golang-go')

        if self.options.source.startswith('lp:'):
            self.fullname = self.options.source.split(':~')[1]
        else:
            self.fullname = self.options.source.split('://')[1]

    def env(self, root):
        # usr/lib/go/bin on newer Ubuntus, usr/bin on trusty
        return [
            'GOPATH={}/go'.format(root),
            'CGO_LDFLAGS=$CGO_LDFLAGS"' + ' '.join([
                '-L{0}/lib',
                '-L{0}/usr/lib',
                '-L{0}/lib/{1}',
                '-L{0}/usr/lib/{1}',
                '$LDFLAGS'
            ]).format(root, snapcraft.common.get_arch_triplet()) + '"',
        ]

    def pull(self):
        # use -d to only download (build will happen later)
        # use -t to also get the test-deps
        self._run(['go', 'get', '-t', '-d', self.fullname])

    def build(self):
        self._run(['go', 'build', self.fullname])
        self._run(['go', 'install', self.fullname])
        self._run(['cp', '-a', os.path.join(self.builddir, 'bin'),
                  self.installdir])

    def _run(self, cmd, **kwargs):
        cmd = ['env', 'GOPATH=' + self.builddir] + cmd
        return self.run(cmd, **kwargs)
