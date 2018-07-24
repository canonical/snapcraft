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


class Subversion(Base):
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
            "svn",
        )
        if source_tag:
            raise errors.SnapcraftSourceInvalidOptionError("svn", "source-tag")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("svn", "source-branch")
        if source_depth:
            raise errors.SnapcraftSourceInvalidOptionError("svn", "source-depth")
        if source_checksum:
            raise errors.SnapcraftSourceInvalidOptionError("svn", "source-checksum")

        self._call_kwargs = {}
        if silent:
            self._call_kwargs["stdout"] = subprocess.DEVNULL
            self._call_kwargs["stderr"] = subprocess.DEVNULL

    def pull(self):
        opts = []

        if self.source_commit:
            opts = ["-r", self.source_commit]

        if os.path.exists(os.path.join(self.source_dir, ".svn")):
            subprocess.check_call(
                [self.command, "update"] + opts,
                cwd=self.source_dir,
                **self._call_kwargs
            )
        else:
            if os.path.isdir(self.source):
                subprocess.check_call(
                    [
                        self.command,
                        "checkout",
                        "file://{}".format(os.path.abspath(self.source)),
                        self.source_dir,
                    ]
                    + opts,
                    **self._call_kwargs
                )
            else:
                subprocess.check_call(
                    [self.command, "checkout", self.source, self.source_dir] + opts,
                    **self._call_kwargs
                )

        self.source_details = self._get_source_details()

    def _get_source_details(self):
        branch = None
        tag = None
        source = self.source
        commit = self.source_commit

        if not commit:
            commit = (
                subprocess.check_output(
                    [
                        "svn",
                        "info",
                        "--show-item",
                        "last-changed-revision",
                        "--no-newline",
                        self.source_dir,
                    ]
                )
                .decode("utf-8")
                .strip()
            )

        return {
            "source-commit": commit,
            "source-branch": branch,
            "source": source,
            "source-tag": tag,
        }
