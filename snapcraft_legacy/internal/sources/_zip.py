# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
        )
        if source_tag:
            raise errors.SnapcraftSourceInvalidOptionError("zip", "source-tag")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("zip", "source-branch")
        if source_depth:
            raise errors.SnapcraftSourceInvalidOptionError("zip", "source-depth")

    def provision(self, dst, clean_target=True, keep_zip=False, src=None):
        if src:
            zip = src
        else:
            zip = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_zip = tempfile.NamedTemporaryFile().name
            shutil.move(zip, tmp_zip)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_zip, zip)

        # Workaround for: https://bugs.python.org/issue15795
        with zipfile.ZipFile(zip, "r") as f:
            for info in f.infolist():
                extracted_file = f.extract(info.filename, path=dst)

                # Extract the mode from the file. Note that external_attr is
                # a four-byte value, where the high two bytes represent UNIX
                # permissions and file type bits, and the low two bytes contain
                # MS-DOS FAT file attributes. Keep the mode to permissions
                # only-- no sticky bit, uid bit, or gid bit.
                mode = info.external_attr >> 16 & 0x1FF

                # If the zip file was created on a non-unix system, it's
                # possible for the mode to end up being zero. That makes it
                # pretty useless, so ignore it if so.
                if mode:
                    os.chmod(extracted_file, mode)

        if not keep_zip:
            os.remove(zip)
