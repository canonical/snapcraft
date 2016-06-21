# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import logging
import os

import requests
import yaml
from progressbar import (ProgressBar, Percentage, Bar, AnimatedMarker)
from xdg import BaseDirectory

PARTS_URI = 'https://parts.snapcraft.io/v1/parts.yaml'

logging.getLogger("urllib3").setLevel(logging.CRITICAL)
logger = logging.getLogger(__name__)


class _Base:

    def __init__(self):
        self.parts_dir = os.path.join(BaseDirectory.xdg_data_home, 'snapcraft')
        os.makedirs(self.parts_dir, exist_ok=True)
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')


class _Update(_Base):

    def __init__(self):
        super().__init__()
        self._headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')
        self._parts_uri = os.environ.get('SNAPCRAFT_PARTS_URI', PARTS_URI)

    def execute(self):
        headers = self._load_headers()
        self._request = requests.get(self._parts_uri, stream=True,
                                     headers=headers)

        if self._request.status_code == 304:
            logger.info('The parts cache is already up to date.')
            return
        self._request.raise_for_status()

        self._download()
        self._save_headers()

    def _download(self):
        total_length = int(self._request.headers.get('Content-Length'))
        progress_bar = ProgressBar(
            widgets=['Updating parts list',
                     Bar(marker='=', left='[', right=']'),
                     ' ', Percentage()],
            maxval=total_length)

        total_read = 0
        progress_bar.start()
        with open(self.parts_yaml, 'wb') as parts_file:
            for buf in self._request.iter_content(1):
                parts_file.write(buf)
                total_read += len(buf)
                progress_bar.update(total_read)
        progress_bar.finish()

    def _load_headers(self):
        if not os.path.exists(self._headers_yaml):
            return None

        with open(self._headers_yaml) as headers_file:
            return yaml.load(headers_file)

    def _save_headers(self):
        headers = {'If-None-Match': self._request.headers.get('ETag')}

        with open(self._headers_yaml, 'w') as headers_file:
            headers_file.write(yaml.dump(headers))


def update():
    _Update().execute()

