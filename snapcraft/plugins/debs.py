# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 struktur AG
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

import os
import urllib.parse
import snapcraft

from snapcraft.repo import _fix_contents
from snapcraft.common import isurl


class DebsPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            'properties': {
                'debs': {
                    'type': 'array',
                },
            },
            'required': [
                'debs',
            ]
        }

    def __init__(self, name, options):
        super().__init__(name, options)
        packages = set()
        debs = set()
        for source in sorted(self.options.debs):
            if isurl(source):
                filename = os.path.split(urllib.parse.urlparse(source).path)[1]
            else:
                # Local file.
                filename = source
            packages.add(filename)
            debs.add(source)
        self.packages = sorted(packages)
        self.debs = sorted(debs)

    def pull(self):
        for source in self.debs:
            self.pull_file(source, destdir=self.sourcedir)

    def build(self):
        self.unpack_debs(self.packages, self.installdir, debdir=self.sourcedir)
        _fix_contents(debdir=self.installdir)

    def pull_file(self, source, destdir=None):
        if isurl(source):
            self.run(['wget', '-q', '-c', source], cwd=destdir)
        else:
            self.run(['cp', os.path.join(os.getcwd(), source), destdir])

    def unpack_debs(self, pkgs, targetDir, debdir=None):
        debdir = debdir or self.builddir

        for p in pkgs:
            self.run(['dpkg-deb', '--extract', p, targetDir], cwd=debdir)
