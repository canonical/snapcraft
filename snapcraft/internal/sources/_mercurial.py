# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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


class Mercurial(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'hg')
        if source_tag and source_branch:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for a '
                'mercurial source')
        if source_tag and source_commit:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-tag and source-commit for a '
                'mercurial source')
        if source_branch and source_commit:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-branch and source-commit for a '
                'mercurial source')
        if source_depth:
            raise errors.IncompatibleOptionsError(
                'can\'t specify source-depth for a mercurial source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.hg')):
            ref = []
            if self.source_tag:
                ref = ['-r', self.source_tag]
            elif self.source_commit:
                ref = ['-r', self.source_commit]
            elif self.source_branch:
                ref = ['-b', self.source_branch]
            cmd = [self.command, 'pull'] + ref + [self.source, ]
        else:
            ref = []
            if self.source_tag or self.source_branch or self.source_commit:
                ref = ['-u', self.source_tag or self.source_branch or
                       self.source_commit]
            cmd = [self.command, 'clone'] + ref + [self.source,
                                                   self.source_dir]

        subprocess.check_call(cmd)
