# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Tim Süberkrüb
# Copyright (C) 2018 Canonical
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
import subprocess
import tempfile

from . import errors
from ._base import FileBase


class SevenZip(FileBase):
    def __init__(
        self,
        source,
        source_dir,
        source_tag=None,
        source_commit=None,
        source_branch=None,
        source_depth=None,
        source_checksum=None,
    ):
        super().__init__(
            source,
            source_dir,
            source_tag,
            source_commit,
            source_branch,
            source_depth,
            source_checksum,
            "7zip",
        )
        if source_tag:
            raise errors.SnapcraftSourceInvalidOptionError("7z", "source-tag")
        elif source_commit:
            raise errors.SnapcraftSourceInvalidOptionError("7z", "source-commit")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("7z", "source-branch")

    def provision(self, dst, clean_target=True, keep_7z=False, src=None):
        if src:
            seven_zip_file = src
        else:
            seven_zip_file = os.path.join(
                self.source_dir, os.path.basename(self.source)
            )
        seven_zip_file = os.path.realpath(seven_zip_file)

        if clean_target:
            tmp_7z = tempfile.NamedTemporaryFile().name
            shutil.move(seven_zip_file, tmp_7z)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_7z, seven_zip_file)

        extract_command = ["7z", "x", seven_zip_file]
        subprocess.check_output(extract_command, cwd=dst)

        if not keep_7z:
            os.remove(seven_zip_file)
