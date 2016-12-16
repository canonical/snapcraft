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


class Bazaar(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'bzr')
        if source_branch:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-branch for a bzr source')
        if source_depth:
            raise errors.IncompatibleOptionsError(
                'can\'t specify source-depth for a bzr source')
        if source_tag and source_commit:
            raise errors.IncompatibleOptionsError(
                'can\'t specify both source-tag and source-commit for '
                'a bzr source')

    def pull(self):
        tag_opts = []
        if self.source_tag:
            tag_opts = ['-r', 'tag:' + self.source_tag]
        if self.source_commit:
            tag_opts = ['-r', self.source_commit]
        if os.path.exists(os.path.join(self.source_dir, '.bzr')):
            cmd = [self.command, 'pull'] + tag_opts + \
                  [self.source, '-d', self.source_dir]
        else:
            os.rmdir(self.source_dir)
            cmd = [self.command, 'branch'] + tag_opts + \
                  [self.source, self.source_dir]

        subprocess.check_call(cmd)
