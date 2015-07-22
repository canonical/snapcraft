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

import os
import snapcraft.common


class BasePlugin:

    def __init__(self, name, options):
        self.name = name
        self.options = options
        self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
        self.installdir = os.path.join(os.getcwd(), "parts", self.name, "install")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
    def pull(self):
        return True

    def build(self):
        return True

    def snap_files(self):
        """Returns two iteratables of globs:
            - the first is the set of files/dirs to include
            - the second is the set of files/dirs to exclude
           For example: (['bin', 'lib'], ['lib/*.a'])"""
        return (['*'], [])

    def env(self, root):
        return []

    # Helpers
    def run(self, cmd, cwd=None, **kwargs):
        if cwd is None:
            cwd = self.builddir
        if True:
            print(' '.join(cmd))
        return snapcraft.common.run(cmd, cwd=cwd, **kwargs)

    def pull_bzr(self, url):
        if os.path.exists(os.path.join(self.sourcedir, ".bzr")):
            return self.run(['bzr', 'pull', url], cwd=self.sourcedir)
        else:
            os.rmdir(self.sourcedir)
            return self.run(['bzr', 'branch', url, self.sourcedir])

    def pull_git(self, url):
        if os.path.exists(os.path.join(self.sourcedir, ".git")):
            return self.run(['git', 'pull'], cwd=self.sourcedir)
        else:
            return self.run(['git', 'clone', url, '.'], cwd=self.sourcedir)

    def get_source(self, source):
        branchType = None
        if source.startswith("bzr:") or source.startswith("lp:"):
            branchType = 'bzr'
        elif source.startswith("git:"):
            branchType = 'git'
        elif ':' in source:
            raise Exception("Did not recognize branch url: " + source)
        # Local branch
        elif os.path.isdir(os.path.join(source, '.bzr')):
            branchType = 'bzr'
            source = os.path.abspath(source)
        elif os.path.isdir(os.path.join(source, '.git')):
            branchType = 'git'
            source = os.path.abspath(source)

        if branchType == 'bzr':
            if not self.pull_bzr(source):
                return False
            if not self.run(['cp', '-Trfa', self.sourcedir, self.builddir]):
                return False
        elif branchType == "git":
            if not self.pull_git(source):
                return False
            if not self.run(['cp', '-Trfa', self.sourcedir, self.builddir]):
                return False
        else:
            # local branch
            path = os.path.abspath(source)
            if os.path.isdir(self.builddir):
                os.rmdir(self.builddir)
            else:
                os.remove(self.builddir)
            os.symlink(path, self.builddir)

        return True

    def handle_source_options(self):
        return self.get_source(self.options.source)

    def makedirs(self, d):
        try:
            os.makedirs(d)
        except FileExistsError:
            pass
