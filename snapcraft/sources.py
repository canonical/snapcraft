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

import logging
import os
import os.path
import shutil
import tarfile
import re

import snapcraft.common


logger = logging.getLogger(__name__)


class IncompatibleOptionsError(Exception):

    def __init__(self, message):
        self.message = message


class Base:

    def __init__(self, source, source_dir, source_tag=None,
                 source_branch=None):
        self.source = source
        self.source_dir = source_dir
        self.source_tag = source_tag
        self.source_branch = source_branch

    def pull(self):
        raise NotImplementedError('this is just a base class')

    def provision(self, dst):
        return snapcraft.common.run(['cp', '-Trfa', self.source_dir, dst],
                                    cwd=os.getcwd())


class Bazaar(Base):

    def __init__(self, source, source_dir, source_tag=None,
                 source_branch=None):
        super().__init__(source, source_dir, source_tag, source_branch)
        if source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a bzr source')

    def pull(self):
        tag_opts = []
        if self.source_tag:
            tag_opts = ['-r', 'tag:' + self.source_tag]
        if os.path.exists(os.path.join(self.source_dir, ".bzr")):
            cmd = ['bzr', 'pull'] + tag_opts + \
                  [self.source, '-d', self.source_dir]
        else:
            os.rmdir(self.source_dir)
            cmd = ['bzr', 'branch'] + tag_opts + \
                  [self.source, self.source_dir]

        return snapcraft.common.run(cmd, cwd=os.getcwd())


class Git(Base):

    def __init__(self, source, source_dir, source_tag=None,
                 source_branch=None):
        super().__init__(source, source_dir, source_tag, source_branch)
        if source_tag and source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for '
                'a git source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, ".git")):
            refspec = 'HEAD'
            if self.source_branch:
                refspec = 'refs/heads/' + self.source_branch
            elif self.source_tag:
                refspec = 'refs/tags/' + self.source_tag
            cmd = ['git', '-C', self.source_dir, 'pull', self.source, refspec]
        else:
            branch_opts = []
            if self.source_tag or self.source_branch:
                branch_opts = ['--branch',
                               self.source_tag or self.source_branch]
            cmd = ['git', 'clone'] + branch_opts + \
                  [self.source, self.source_dir]

        return snapcraft.common.run(cmd, cwd=os.getcwd())


class Mercurial(Base):

    def __init__(self, source, source_dir, source_tag=None,
                 source_branch=None):
        super().__init__(source, source_dir, source_tag, source_branch)
        if source_tag and source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for a '
                'mercurial source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, ".hg")):
            ref = []
            if self.source_tag:
                ref = ['-r', self.source_tag]
            elif self.source_branch:
                ref = ['-b', self.source_branch]
            cmd = ['hg', 'pull'] + ref + [self.source, ]
        else:
            ref = []
            if self.source_tag or self.source_branch:
                ref = ['-u', self.source_tag or self.source_branch]
            cmd = ['hg', 'clone'] + ref + [self.source, self.source_dir]

        return snapcraft.common.run(cmd, cwd=os.getcwd())


class Tar(Base):

    def __init__(self, source, source_dir, source_tag=None,
                 source_branch=None):
        super().__init__(source, source_dir, source_tag, source_branch)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a tar source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a tar source')

    def pull(self):
        if snapcraft.common.isurl(self.source):
            return snapcraft.common.run(['wget', '-q', '-c', self.source],
                                        cwd=self.source_dir)
        else:
            return True

    def provision(self, dst, clean_target=True):
        # TODO add unit tests.
        if snapcraft.common.isurl(self.source):
            tarball = os.path.join(
                self.source_dir,
                os.path.basename(self.source))
        else:
            tarball = os.path.abspath(self.source)

        if clean_target:
            shutil.rmtree(dst)
            os.makedirs(dst)

        return self._extract(tarball, dst)

    def _extract(self, tarball, dst):
        with tarfile.open(tarball) as tar:
            def filter_members(tar):
                """Filters members and member names:
                    - strips common prefix
                    - bans dangerous names"""
                members = tar.getmembers()
                common = os.path.commonprefix([m.name for m in members])

                # commonprefix() works a character at a time and will
                # consider "d/ab" and "d/abc" to have common prefix "d/ab";
                # check all members either start with common dir
                for m in members:
                    if not (m.name.startswith(common + "/") or
                            m.isdir() and m.name == common):
                        # commonprefix() didn't return a dir name; go up one
                        # level
                        common = os.path.dirname(common)
                        break

                for m in members:
                    if m.name == common:
                        continue
                    if m.name.startswith(common + "/"):
                        m.name = m.name[len(common + "/"):]
                    # strip leading "/", "./" or "../" as many times as needed
                    m.name = re.sub(r'^(\.{0,2}/)*', r'', m.name)
                    # We mask all files to be writable to be able to easily
                    # extract on top.
                    m.mode = m.mode | 0o200
                    yield m

            tar.extractall(members=filter_members(tar), path=dst)

        return True


class Local(Base):

    def pull(self):
        return True

    def provision(self, dst):
        path = os.path.abspath(self.source)
        if os.path.islink(dst):
            os.remove(dst)
        elif os.path.isdir(dst):
            os.rmdir(dst)
        else:
            os.remove(dst)
        os.symlink(path, dst)

        return True
