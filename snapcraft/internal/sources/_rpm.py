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
import libarchive
import shutil
import tempfile

from . import errors
from ._base import FileBase


class Rpm(FileBase):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)
        if source_tag:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-tag for a rpm source')
        elif source_commit:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-commit for a rpm source')
        elif source_branch:
            raise errors.IncompatibleOptionsError(
                'can\'t specify a source-branch for a rpm source')

    def provision(self, dst, clean_target=True, keep_rpm=False):
        rpm_file = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_rpm = tempfile.NamedTemporaryFile().name
            shutil.move(rpm_file, tmp_rpm)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_rpm, rpm_file)

        # Ensure dst does not have trailing slash
        dst = dst.rstrip('/')
        # Open the RPM file and extract it to destination
        with libarchive.file_reader(rpm_file) as rpm:
            for rpm_file_entry in rpm:
                # Binary RPM archive data has paths starting with ./ to support
                # relocation if enabled in the building of RPMs
                rpm_file_entrypath = rpm_file_entry.pathname.lstrip('./')
                rpm_file_entrypath = rpm_file_entrypath.lstrip('/')
                rpm_file_entry.pathname = os.path.join(dst, rpm_file_entrypath)
                # XXX: libarchive frees the entry at the end of loop iterations
                # See https://github.com/Changaco/python-libarchive-c/issues/43
                libarchive.extract.extract_entries([rpm_file_entry])

        if not keep_rpm:
            os.remove(rpm_file)
