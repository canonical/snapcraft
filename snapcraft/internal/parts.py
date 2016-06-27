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
import sys

import requests
import yaml
from progressbar import (
    AnimatedMarker,
    Bar,
    Percentage,
    ProgressBar,
    UnknownLength,
)
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
        total_length = int(self._request.headers.get('Content-Length', '0'))
        if total_length:
            progress_bar = ProgressBar(
                widgets=['Updating parts list',
                         Bar(marker='=', left='[', right=']'),
                         ' ', Percentage()],
                maxval=total_length)
        else:
            progress_bar = ProgressBar(
                widgets=['Updating parts list... ', AnimatedMarker()],
                maxval=UnknownLength)

        total_read = 0
        progress_bar.start()
        with open(self.parts_yaml, 'wb') as parts_file:
            for buf in self._request.iter_content(1024):
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


class _RemoteParts(_Base):

    def __init__(self):
        super().__init__()

        if os.path.exists(self.parts_yaml):
            with open(self.parts_yaml) as parts_file:
                self._parts = yaml.load(parts_file)
        else:
            self._parts = {}

    def get_part(self, part_name, full=False):
        remote_part = self._parts[part_name].copy()
        if not full:
            for key in ['description', 'maintainer']:
                remote_part.pop(key)
        return remote_part

    def compose(self, part_name, properties):
        """Return properties composed with the ones from part name in the wiki.
        :param str part_name: The name of the part to query from the wiki
        :param dict properties: The current set of properties
        :return: Part properties from the wiki composed with the properties
                 passed as a parameter.
        :rtype: dict
        :raises KeyError: if part_name is not found in the wiki.
        """
        remote_part = self.get_part(part_name)
        remote_part.update(properties)

        return remote_part


def update():
    _Update().execute()


def define(part_name):
    try:
        remote_part = _RemoteParts().get_part(part_name, full=True)
    except KeyError as e:
        raise RuntimeError(
            'Cannot find the part name {!r} in the cache. Please '
            'consider going to https://wiki.ubuntu.com/snapcraft/parts '
            'to add it.') from e
    print('Maintainer: {!r}'.format(remote_part.pop('maintainer')))
    print('Description: {!r}'.format(remote_part.pop('description')))
    print('')
    yaml.dump({part_name: remote_part},
              default_flow_style=False, stream=sys.stdout)


def get_remote_parts():
    return _RemoteParts()
