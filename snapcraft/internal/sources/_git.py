# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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
import subprocess

from . import errors
from ._base import Base


class Git(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None, silent=False):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'git')
        if source_tag and source_branch:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for '
                'a git source')
        if source_tag and source_commit:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-tag and source-commit for '
                'a git source')
        if source_branch and source_commit:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-branch and source-commit for '
                'a git source')
        self.kwargs = {}
        if silent:
            self.kwargs['stdout'] = subprocess.DEVNULL
            self.kwargs['stderr'] = subprocess.DEVNULL

    def _pull_existing(self):
        refspec = 'HEAD'
        if self.source_branch:
            refspec = 'refs/heads/' + self.source_branch
        elif self.source_tag:
            refspec = 'refs/tags/' + self.source_tag
        elif self.source_commit:
            refspec = self.source_commit

        reset_spec = refspec if refspec != 'HEAD' else 'origin/master'

        subprocess.check_call([self.command, '-C', self.source_dir,
                               'fetch', '--prune',
                               '--recurse-submodules=yes'], **self.kwargs)
        subprocess.check_call([self.command, '-C', self.source_dir,
                               'reset', '--hard', reset_spec], **self.kwargs)

        # Merge any updates for the submodules (if any).
        subprocess.check_call([self.command, '-C', self.source_dir,
                              'submodule', 'update', '--recursive',
                               '--remote'], **self.kwargs)

    def _clone_new(self):
        command = [self.command, 'clone', '--recursive']
        if self.source_tag or self.source_branch:
            command.extend([
                '--branch', self.source_tag or self.source_branch])
        if self.source_depth:
            command.extend(['--depth', str(self.source_depth)])
        subprocess.check_call(command + [self.source, self.source_dir],
                              **self.kwargs)

        if self.source_commit:
            subprocess.check_call([self.command, '-C', self.source_dir,
                                  'checkout', self.source_commit],
                                  **self.kwargs)

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.git')):
            self._pull_existing()
        else:
            self._clone_new()
