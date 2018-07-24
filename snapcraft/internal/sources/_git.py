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
import subprocess
import sys

from . import errors
from ._base import Base


class Git(Base):
    @classmethod
    def generate_version(cls, *, source_dir=None):
        """Return the latest git tag from PWD or defined source_dir.

        The output depends on the use of annotated tags and will return
        something like: '2.28+git.10.abcdef' where '2.28 is the
        tag, '+git' indicates there are commits ahead of the tag, in
        this case it is '10' and the latest commit hash begins with
        'abcdef'. If there are no tags or the revision cannot be
        determined, this will return 0 as the tag and only the commit
        hash of the latest commit.
        """
        if not source_dir:
            source_dir = os.getcwd()

        encoding = sys.getfilesystemencoding()
        try:
            output = (
                subprocess.check_output(
                    ["git", "-C", source_dir, "describe", "--dirty"],
                    stderr=subprocess.DEVNULL,
                )
                .decode(encoding)
                .strip()
            )
        except subprocess.CalledProcessError:
            # If we fall into this exception it is because the repo is not
            # tagged at all.
            proc = subprocess.Popen(
                ["git", "-C", source_dir, "describe", "--dirty", "--always"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            stdout, stderr = proc.communicate()
            if proc.returncode != 0:
                # This most likely means the project we are in is not driven
                # by git.
                raise errors.VCSError(message=stderr.decode(encoding).strip())
            return "0+git.{}".format(stdout.decode(encoding).strip())

        m = re.search(
            r"^(?P<tag>[a-zA-Z0-9.+~-]+)-"
            r"(?P<revs_ahead>\d+)-"
            r"g(?P<commit>[0-9a-fA-F]+(?:-dirty)?)$",
            output,
        )

        if not m:
            # This means we have a pure tag
            return output

        tag = m.group("tag")
        revs_ahead = m.group("revs_ahead")
        commit = m.group("commit")

        return "{}+git{}.{}".format(tag, revs_ahead, commit)

    def __init__(
        self,
        source,
        source_dir,
        source_tag=None,
        source_commit=None,
        source_branch=None,
        source_depth=None,
        silent=False,
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
            "git",
        )
        if source_tag and source_branch:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "git", ["source-tag", "source-branch"]
            )
        if source_tag and source_commit:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "git", ["source-tag", "source-commit"]
            )
        if source_branch and source_commit:
            raise errors.SnapcraftSourceIncompatibleOptionsError(
                "git", ["source-branch", "source-commit"]
            )
        if source_checksum:
            raise errors.SnapcraftSourceInvalidOptionError("git", "source-checksum")
        self._call_kwargs = {}
        if silent:
            self._call_kwargs["stdout"] = subprocess.DEVNULL
            self._call_kwargs["stderr"] = subprocess.DEVNULL

    def _pull_existing(self):
        refspec = "HEAD"
        if self.source_branch:
            refspec = "refs/heads/" + self.source_branch
        elif self.source_tag:
            refspec = "refs/tags/" + self.source_tag
        elif self.source_commit:
            refspec = self.source_commit

        reset_spec = refspec if refspec != "HEAD" else "origin/master"

        subprocess.check_call(
            [
                self.command,
                "-C",
                self.source_dir,
                "fetch",
                "--prune",
                "--recurse-submodules=yes",
            ],
            **self._call_kwargs
        )
        subprocess.check_call(
            [self.command, "-C", self.source_dir, "reset", "--hard", reset_spec],
            **self._call_kwargs
        )

        # Merge any updates for the submodules (if any).
        subprocess.check_call(
            [
                self.command,
                "-C",
                self.source_dir,
                "submodule",
                "update",
                "--recursive",
                "--force",
            ],
            **self._call_kwargs
        )

    def _clone_new(self):
        command = [self.command, "clone", "--recursive"]
        if self.source_tag or self.source_branch:
            command.extend(["--branch", self.source_tag or self.source_branch])
        if self.source_depth:
            command.extend(["--depth", str(self.source_depth)])
        subprocess.check_call(
            command + [self.source, self.source_dir], **self._call_kwargs
        )

        if self.source_commit:
            subprocess.check_call(
                [self.command, "-C", self.source_dir, "checkout", self.source_commit],
                **self._call_kwargs
            )

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, ".git")):
            self._pull_existing()
        else:
            self._clone_new()
        self.source_details = self._get_source_details()

    def _get_source_details(self):
        tag = self.source_tag
        commit = self.source_commit
        branch = self.source_branch
        source = self.source
        checksum = self.source_checksum

        if not tag and not branch and not commit:
            commit = (
                subprocess.check_output(
                    ["git", "-C", self.source_dir, "rev-parse", "HEAD"]
                )
                .decode("utf-8")
                .strip()
            )

        return {
            "source-commit": commit,
            "source-branch": branch,
            "source": source,
            "source-tag": tag,
            "source-checksum": checksum,
        }
