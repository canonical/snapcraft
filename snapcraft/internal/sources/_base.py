# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
import requests
import shutil

import snapcraft.internal.common
from snapcraft.internal.indicators import (
    download_requests_stream,
    download_urllib_source
)


class Base:

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None,
                 command=None):
        self.source = source
        self.source_dir = source_dir
        self.source_tag = source_tag
        self.source_commit = source_commit
        self.source_branch = source_branch
        self.source_depth = source_depth

        self.command = command


class FileBase(Base):

    def pull(self):
        if snapcraft.internal.common.isurl(self.source):
            self.download()
        else:
            shutil.copy2(self.source, self.source_dir)

        self.provision(self.source_dir)

    def download(self):
        self.file = os.path.join(
                self.source_dir, os.path.basename(self.source))

        if snapcraft.internal.common.get_url_scheme(self.source) == 'ftp':
            download_urllib_source(self.source, self.file)
        else:
            request = requests.get(
                self.source, stream=True, allow_redirects=True)
            request.raise_for_status()

            download_requests_stream(request, self.file)
