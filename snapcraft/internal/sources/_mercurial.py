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


class Mercurial(Base):
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
            "hg",
        )
        if source_tag and source_branch:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "mercurial", ["source-tag", "source-branch"]
            )
        if source_tag and source_commit:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "mercurial", ["source-tag", "source-commit"]
            )
        if source_branch and source_commit:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "mercurial", ["source-branch", "source-commit"]
            )
        if source_depth:
            raise errors.SnapcraftSourceInvalidOptionError("mercurial", "source-depth")
        if source_checksum:
            raise errors.SnapcraftSourceInvalidOptionError(
                "mercurial", "source-checksum"
            )

        self._call_kwargs = {}
        if silent:
            self._call_kwargs["stdout"] = subprocess.DEVNULL
            self._call_kwargs["stderr"] = subprocess.DEVNULL

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, ".hg")):
            ref = []
            if self.source_tag:
                ref = ["-r", self.source_tag]
            elif self.source_commit:
                ref = ["-r", self.source_commit]
            elif self.source_branch:
                ref = ["-b", self.source_branch]
            cmd = [self.command, "pull"] + ref + [self.source]
        else:
            ref = []
            if self.source_tag or self.source_branch or self.source_commit:
                ref = [
                    "-u",
                    self.source_tag or self.source_branch or self.source_commit,
                ]
            cmd = [self.command, "clone"] + ref + [self.source, self.source_dir]

        subprocess.check_call(cmd, **self._call_kwargs)
        self.source_details = self._get_source_details()

    def _get_source_details(self):
        tag = self.source_tag
        commit = self.source_commit
        branch = self.source_branch
        source = self.source

        if not (tag or commit or branch):
            commit = (
                subprocess.check_output(["hg", "id", self.source_dir])
                .split()[0]
                .decode("utf-8")
                .strip()
            )

        return {
            "source-commit": commit,
            "source-branch": branch,
            "source": source,
            "source-tag": tag,
        }
