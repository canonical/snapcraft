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
import subprocess

from . import errors
from ._base import Base


class Bazaar(Base):
    def __init__(
        self,
        source,
        source_dir,
        source_tag=None,
        source_commit=None,
        source_branch=None,
        source_depth=None,
        source_checksum=None,
        silent=False,
    ):
        super().__init__(
            source,
            source_dir,
            source_tag,
            source_commit,
            source_branch,
            source_depth,
            source_checksum,
            "bzr",
        )
        if source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("bzr", "source-branch")
        if source_depth:
            raise errors.SnapcraftSourceInvalidOptionError("bzr", "source-depth")
        if source_tag and source_commit:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "bzr", ["source-tag", "source-commit"]
            )
        if source_checksum:
            raise errors.SnapcraftSourceInvalidOptionError("bzr", "source-checksum")

        self._call_kwargs = {}
        if silent:
            self._call_kwargs["stdout"] = subprocess.DEVNULL
            self._call_kwargs["stderr"] = subprocess.DEVNULL

    def pull(self):
        tag_opts = []
        if self.source_tag:
            tag_opts = ["-r", "tag:" + self.source_tag]
        if self.source_commit:
            tag_opts = ["-r", self.source_commit]
        if os.path.exists(os.path.join(self.source_dir, ".bzr")):
            cmd = (
                [self.command, "pull"] + tag_opts + [self.source, "-d", self.source_dir]
            )
        else:
            os.rmdir(self.source_dir)
            cmd = [self.command, "branch"] + tag_opts + [self.source, self.source_dir]

        subprocess.check_call(cmd, **self._call_kwargs)
        self.source_details = self._get_source_details()

    def _get_source_details(self):
        tag = self.source_tag
        commit = self.source_commit

        if not tag:
            if os.path.exists(self.source_dir):
                commit = (
                    subprocess.check_output(["bzr", "revno", self.source_dir])
                    .decode("utf-8")
                    .strip()
                )

        branch = None
        source = self.source

        return {
            "source-commit": commit,
            "source-branch": branch,
            "source": source,
            "source-tag": tag,
        }
