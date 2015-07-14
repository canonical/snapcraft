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

    def __init__(self, name, config, options):
        self.name = name
        self.config = config
        self.options = options
        self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
        self.installdir = os.path.join(os.getcwd(), "parts", self.name, "install")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
    def pull(self):
        if self.config.get('accepts-source-options', False):
            return self._handle_source_options()
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

    def makedirs(self, d):
        try:
            os.makedirs(d)
        except FileExistsError:
            pass

    def get_source(self, source, source_type=None, source_tag=None, source_branch=None):
        if source_type is None:
            if source.startswith("bzr:") or source.startswith("lp:"):
                source_type = 'bzr'
            elif source.startswith("git:"):
                source_type = 'git'
            elif ':' in source:
                snapcraft.common.fatal("Unrecognized source '%s' for part '%s'.  Try specifying 'source-type'." % (source, self.name))

        if source_type == 'bzr':
            if source_branch:
                snapcraft.common.fatal("You can't specify source-branch for a bzr source (part '%s')." % self.name)
            if not self._pull_bzr(source, source_tag):
                return False
            if not self.run(['cp', '-Trfa', self.sourcedir, self.builddir]):
                return False
        elif source_type == 'git':
            if not self._pull_git(source, source_tag, source_branch):
                return False
            if not self.run(['cp', '-Trfa', self.sourcedir, self.builddir]):
                return False
        else:
            # local source dir
            path = os.path.abspath(source)
            if os.path.isdir(self.builddir):
                os.rmdir(self.builddir)
            else:
                os.remove(self.builddir)
            os.symlink(path, self.builddir)

        return True

    # Private helpers
    def _pull_bzr(self, source, source_tag):
        tag_opts = []
        if source_tag:
            tag_opts = ['-r', 'tag:' + source_tag]
        if os.path.exists(os.path.join(self.sourcedir, ".bzr")):
            return self.run(['bzr', 'pull'] + tag_opts + [source, '-d', self.sourcedir], cwd=os.getcwd())
        else:
            os.rmdir(self.sourcedir)
            return self.run(['bzr', 'branch'] + tag_opts + [source, self.sourcedir], cwd=os.getcwd())

    def _pull_git(self, source, source_tag, source_branch):
        if source_tag and source_branch:
            snapcraft.common.fatal("You can't specify both source-tag and source-branch for a git source (part '%s')." % self.name)

        if os.path.exists(os.path.join(self.sourcedir, ".git")):
            refspec = 'HEAD'
            if source_branch:
                refspec = 'refs/heads/' + source_branch
            elif source_tag:
                refspec = 'refs/tags/' + source_tag
            return self.run(['git', '-C', self.sourcedir, 'pull', source, refspec], cwd=os.getcwd())
        else:
            branch_opts = []
            if source_tag or source_branch:
                branch_opts = ['--branch', source_tag or source_branch]
            return self.run(['git', 'clone'] + branch_opts + [source, self.sourcedir], cwd=os.getcwd())

    def _handle_source_options(self):
        return self.get_source(self.options.source,
                               source_type=self.options.source_type,
                               source_tag=self.options.source_tag,
                               source_branch=self.options.source_branch)
