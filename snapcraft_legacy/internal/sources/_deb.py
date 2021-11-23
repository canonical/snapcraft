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
import tarfile
import tempfile

import debian.arfile

from . import errors
from ._base import FileBase


class Deb(FileBase):
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
            raise errors.SnapcraftSourceInvalidOptionError("deb", "source-tag")
        elif source_commit:
            raise errors.SnapcraftSourceInvalidOptionError("deb", "source-commit")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("deb", "source-branch")

    def provision(self, dst, clean_target=True, keep_deb=False, src=None):
        if src:
            deb_file = src
        else:
            deb_file = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_deb = tempfile.NamedTemporaryFile().name
            shutil.move(deb_file, tmp_deb)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_deb, deb_file)

        # Importing DebFile causes LP: #1731478 when snapcraft is
        # run as a snap.
        deb_ar = debian.arfile.ArFile(deb_file)
        try:
            data_member_name = [
                i for i in deb_ar.getnames() if i.startswith("data.tar")
            ][0]
        except IndexError:
            raise errors.InvalidDebError(deb_file=deb_file)
        data_member = deb_ar.getmember(data_member_name)
        with tarfile.open(fileobj=data_member) as tar:
            tar.extractall(dst)

        if not keep_deb:
            os.remove(deb_file)
