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

"""Common keywords for plugins that use common source options.

A part that uses common source options can have these keyword entries:

    - source:
      (string)
      A path to some source tree to build. It can be either remote or local,
      and either a directory tree or a tarball.
    - source-type:
      (string)
      In some cases the source is not enough to identify the version control
      system or compression algorithim. This hints the system into what to
      do, the valid values are:

                   - bzr
                   - mercurial
                   - hg
                   - git
                   - tar

    - source-branch:
      (string)
      A specific branch from the source tree. This will result in an error
      if used with a bazaar source type.
    - source-tag:
      (string)
      A specific tag from the source tree.
"""


import logging
import os
import os.path
import requests
import shutil
import tarfile
import re

import snapcraft.common


logging.getLogger('urllib3').setLevel(logging.CRITICAL)


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

        snapcraft.common.run(cmd, cwd=os.getcwd())


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

        snapcraft.common.run(cmd, cwd=os.getcwd())


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

        snapcraft.common.run(cmd, cwd=os.getcwd())


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
        if not snapcraft.common.isurl(self.source):
            return

        req = requests.get(self.source, stream=True, allow_redirects=True)
        if req.status_code is not 200:
            raise EnvironmentError('unexpected http status code when '
                                   'downloading %r'.format(req.status_code))

        file = os.path.join(self.source_dir, os.path.basename(self.source))
        with open(file, 'wb') as f:
            for chunk in req.iter_content(1024):
                f.write(chunk)

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

        self._extract(tarball, dst)

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


class Local(Base):

    def pull(self):
        pass

    def provision(self, dst):
        path = os.path.abspath(self.source)
        if os.path.islink(dst):
            os.remove(dst)
        elif os.path.isdir(dst):
            os.rmdir(dst)
        else:
            os.remove(dst)
        os.symlink(path, dst)


def get(sourcedir, builddir, options):
    """Populate sourcedir and builddir from parameters defined in options.

    :param str sourcedir: The source directory to use.
    :param str builddir: The build directory to use.
    :param options: source options.
    """
    source_type = getattr(options, 'source_type', None)
    source_tag = getattr(options, 'source_tag', None)
    source_branch = getattr(options, 'source_branch', None)

    handler_class = _get_source_handler(source_type, options.source)
    handler = handler_class(options.source, sourcedir, source_tag,
                            source_branch)
    handler.pull()
    handler.provision(builddir)


_source_handler = {
    'bzr': Bazaar,
    'git': Git,
    'hg': Mercurial,
    'mercurial': Mercurial,
    'tar': Tar,
}


def _get_source_handler(source_type, source):
    if not source_type:
        source_type = _get_source_type_from_uri(source)

    return _source_handler.get(source_type, Local)


_tar_type_regex = re.compile(r'.*\.((tar\.(xz|gz|bz2))|tgz)$')


def _get_source_type_from_uri(source):
    source_type = ''
    if source.startswith("bzr:") or source.startswith("lp:"):
        source_type = 'bzr'
    elif source.startswith("git:"):
        source_type = 'git'
    elif _tar_type_regex.match(source):
        source_type = 'tar'
    elif snapcraft.common.isurl(source):
        raise ValueError('no handler to manage source')
    elif not os.path.isdir(source):
        raise ValueError('local source is not a directory')

    return source_type
