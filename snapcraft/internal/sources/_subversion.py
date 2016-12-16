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


class Subversion(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'svn')
        if source_tag:
            if source_branch:
                raise errors.IncompatibleOptionsError(
                    "Can't specify source-tag OR source-branch for a "
                    "Subversion source")
            else:
                raise errors.IncompatibleOptionsError(
                    "Can't specify source-tag for a Subversion source")
        elif source_branch:
            raise errors.IncompatibleOptionsError(
                "Can't specify source-branch for a Subversion source")
        if source_depth:
            raise errors.IncompatibleOptionsError(
                'can\'t specify source-depth for a Subversion source')

    def pull(self):
        opts = []

        if self.source_commit:
            opts = ["-r", self.source_commit]

        if os.path.exists(os.path.join(self.source_dir, '.svn')):
            subprocess.check_call(
                [self.command, 'update'] + opts, cwd=self.source_dir)
        else:
            if os.path.isdir(self.source):
                subprocess.check_call(
                    [self.command, 'checkout',
                     'file://{}'.format(os.path.abspath(self.source)),
                     self.source_dir] + opts)
            else:
                subprocess.check_call(
                    [self.command, 'checkout', self.source, self.source_dir] +
                    opts)
