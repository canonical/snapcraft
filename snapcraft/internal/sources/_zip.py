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
import shutil
import tempfile
import zipfile

from . import errors
from ._base import FileBase


class Zip(FileBase):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)
        if source_tag:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-tag for a zip source')
        elif source_branch:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-branch for a zip source')
        if source_depth:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-depth for a zip source')

    def provision(self, dst, clean_target=True, keep_zip=False):
        zip = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_zip = tempfile.NamedTemporaryFile().name
            shutil.move(zip, tmp_zip)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_zip, zip)

        zipfile.ZipFile(zip).extractall(path=dst)

        if not keep_zip:
            os.remove(zip)
