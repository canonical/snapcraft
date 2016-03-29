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

import hashlib
import json
import os
import logging

from .common import get_oauth_session


_STORE_DETAILS_URL = 'https://search.apps.ubuntu.com/api/v1/package/{}'
logger = logging.getLogger(__name__)


def download(snap, download_path, config, arch):
    """Download snap from the store to download_path"""
    session = get_oauth_session(config)
    if session is None:
        raise EnvironmentError(
            'No valid credentials found. Have you run "snapcraft login"?')

    # TODO add release header
    session.headers.update({'X-Ubuntu-Architecture': arch})

    logger.info('Getting details for {!r}'.format(snap))
    details_url = _STORE_DETAILS_URL.format(snap)
    details = session.get(details_url).content.decode('utf-8')
    details_json = json.loads(details)
    download_url = details_json.get('download_url')
    download_sha = details_json.get('download_sha512')

    if _is_downloaded(download_path, download_sha):
        logger.info('Already downloaded {!r}'.format(snap))
    else:
        logger.info('Downloading {!r}'.format(snap))
        download = session.get(download_url)
        with open(download_path, 'wb') as f:
            f.write(download.content)
        if _is_downloaded(download_path, download_sha):
            logger.info('Successfully downloaded {!r}'.format(snap))
        else:
            raise RuntimeError('Failed to download {!r}'.format(snap))


def _is_downloaded(download_path, download_sha):
    if not os.path.exists(download_path):
        return False

    file_sum = hashlib.sha512()
    with open(download_path, 'rb') as f:
        for file_chunk in iter(
                lambda: f.read(file_sum.block_size * 128), b''):
            file_sum.update(file_chunk)

    return download_sha == file_sum.hexdigest()
