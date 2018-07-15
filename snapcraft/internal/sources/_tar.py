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
import re
import shutil
import tarfile
import tempfile

from . import errors
from ._base import FileBase


class Tar(FileBase):
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
            raise errors.SnapcraftSourceInvalidOptionError("tar", "source-tag")
        elif source_commit:
            raise errors.SnapcraftSourceInvalidOptionError("tar", "source-commit")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("tar", "source-branch")
        if source_depth:
            raise errors.SnapcraftSourceInvalidOptionError("tar", "source-depth")

    def provision(self, dst, clean_target=True, keep_tarball=False, src=None):
        # TODO add unit tests.
        if src:
            tarball = src
        else:
            tarball = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_tarball = tempfile.NamedTemporaryFile().name
            shutil.move(tarball, tmp_tarball)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_tarball, tarball)

        self._extract(tarball, dst)

        if not keep_tarball:
            os.remove(tarball)

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
                    if not (
                        m.name.startswith(common + "/")
                        or m.isdir()
                        and m.name == common
                    ):
                        # commonprefix() didn't return a dir name; go up one
                        # level
                        common = os.path.dirname(common)
                        break

                for m in members:
                    if m.name == common:
                        continue
                    self._strip_prefix(common, m)
                    # We mask all files to be writable to be able to easily
                    # extract on top.
                    m.mode = m.mode | 0o200
                    yield m

            tar.extractall(members=filter_members(tar), path=dst)

    def _strip_prefix(self, common, member):
        if member.name.startswith(common + "/"):
            member.name = member.name[len(common + "/") :]
        # strip leading '/', './' or '../' as many times as needed
        member.name = re.sub(r"^(\.{0,2}/)*", r"", member.name)
        # do the same for linkname if this is a hardlink
        if member.islnk() and not member.issym():
            if member.linkname.startswith(common + "/"):
                member.linkname = member.linkname[len(common + "/") :]
            member.linkname = re.sub(r"^(\.{0,2}/)*", r"", member.linkname)
