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
import subprocess
import sys

import requests

import snapcraft.internal.common
from snapcraft.internal.cache import FileCache
from snapcraft.internal.indicators import (
    download_requests_stream,
    download_urllib_source,
)

from . import errors
from ._checksum import split_checksum, verify_checksum


class Base:
    def __init__(
        self,
        source,
        source_dir,
        source_tag=None,
        source_commit=None,
        source_branch=None,
        source_depth=None,
        source_checksum=None,
        command=None,
    ):
        self.source = source
        self.source_dir = source_dir
        self.source_tag = source_tag
        self.source_commit = source_commit
        self.source_branch = source_branch
        self.source_depth = source_depth
        self.source_checksum = source_checksum
        self.source_details = None

        self.command = command
        self._checked = False

    def check(self, target: str):
        """Check if pulled sources have changed since target was created.

        :param str target: Path to target file.
        """
        self._checked = True
        return self._check(target)

    def update(self):
        """Update pulled source.

        :raises RuntimeError: If this function is called before `check()`.
        """
        if not self._checked:
            # This is programmer error
            raise RuntimeError("source must be checked before it's updated")
        self._update()

    def _check(self, target: str):
        """Check if pulled sources have changed since target was created.

        :param str target: Path to target file.
        """
        raise errors.SourceUpdateUnsupportedError(self)

    def _update(self):
        """Update pulled source."""
        raise errors.SourceUpdateUnsupportedError(self)

    def _run(self, command, **kwargs):
        try:
            subprocess.check_call(command, **kwargs)
        except subprocess.CalledProcessError as e:
            raise errors.SnapcraftPullError(command, e.returncode)

    def _run_output(self, command, **kwargs):
        try:
            return (
                subprocess.check_output(command, **kwargs)
                .decode(sys.getfilesystemencoding())
                .strip()
            )
        except subprocess.CalledProcessError as e:
            raise errors.SnapcraftPullError(command, e.returncode)


class FileBase(Base):
    def pull(self):
        source_file = None
        is_source_url = snapcraft.internal.common.isurl(self.source)

        # First check if it is a url and download and if not
        # it is probably locally referenced.
        if is_source_url:
            source_file = self.download()
        else:
            basename = os.path.basename(self.source)
            source_file = os.path.join(self.source_dir, basename)
            # We make this copy as the provisioning logic can delete
            # this file and we don't want that.
            try:
                shutil.copy2(self.source, source_file)
            except FileNotFoundError as exc:
                raise errors.SnapcraftSourceNotFoundError(self.source) from exc

        # Verify before provisioning
        if self.source_checksum:
            verify_checksum(self.source_checksum, source_file)

        # We finally provision, but we don't clean the target so override-pull
        # can actually have meaning when using these sources.
        self.provision(self.source_dir, src=source_file, clean_target=False)

    def download(self, filepath: str = None) -> str:
        if filepath is None:
            self.file = os.path.join(self.source_dir, os.path.basename(self.source))
        else:
            self.file = filepath

        # First check if we already have the source file cached.
        file_cache = FileCache()
        if self.source_checksum:
            algorithm, hash = split_checksum(self.source_checksum)
            cache_file = file_cache.get(algorithm=algorithm, hash=hash)
            if cache_file:
                # We make this copy as the provisioning logic can delete
                # this file and we don't want that.
                shutil.copy2(cache_file, self.file)
                return self.file

        # If not we download and store
        if snapcraft.internal.common.get_url_scheme(self.source) == "ftp":
            download_urllib_source(self.source, self.file)
        else:
            try:
                request = requests.get(self.source, stream=True, allow_redirects=True)
                request.raise_for_status()
            except requests.exceptions.RequestException as e:
                raise errors.SnapcraftRequestError(message=e)

            download_requests_stream(request, self.file)

        # We verify the file if source_checksum is defined
        # and we cache the file for future reuse.
        if self.source_checksum:
            algorithm, digest = verify_checksum(self.source_checksum, self.file)
            file_cache.cache(filename=self.file, algorithm=algorithm, hash=hash)
        return self.file
