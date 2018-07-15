# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Neal Gompa
# Copyright (C) 2017-2018 Canonical
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
import shlex
import shutil
import subprocess
import tempfile

from . import errors
from ._base import FileBase


class Rpm(FileBase):
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
            "rpm2cpio",
        )
        if source_tag:
            raise errors.SnapcraftSourceInvalidOptionError("rpm", "source-tag")
        elif source_commit:
            raise errors.SnapcraftSourceInvalidOptionError("rpm", "source-commit")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("rpm", "source-branch")

    def provision(self, dst, clean_target=True, keep_rpm=False, src=None):
        if src:
            rpm_file = src
        else:
            rpm_file = os.path.join(self.source_dir, os.path.basename(self.source))
        rpm_file = os.path.realpath(rpm_file)

        if clean_target:
            tmp_rpm = tempfile.NamedTemporaryFile().name
            shutil.move(rpm_file, tmp_rpm)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_rpm, rpm_file)

        extract_command = "rpm2cpio {} | cpio -idmv".format(shlex.quote(rpm_file))
        subprocess.check_output(extract_command, shell=True, cwd=dst)

        if not keep_rpm:
            os.remove(rpm_file)
