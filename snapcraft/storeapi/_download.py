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

from snapcraft import config
from .common import get_oauth_session


_STORE_SEARCH_URL = 'https://search.apps.ubuntu.com/api/v1/search'

logger = logging.getLogger(__name__)


def download(snap_name, channel, download_path, arch):
    """Download snap from the store to download_path"""
    conf = config.Config()
    conf.load()
    session = get_oauth_session(conf)
    if session is None:
        raise EnvironmentError(
            'No valid credentials found. Have you run "snapcraft login"?')

    # TODO add release header
    session.headers.update({
        'accept': 'application/hal+json',
        'X-Ubuntu-Architecture': arch,
        'X-Ubuntu-Release': '16',
        'X-Ubuntu-Device-Channel': channel,
    })
    session.params = {
        'q': 'package_name:"{}"'.format(snap_name),
        'fields': 'download_url,anon_download_url,download_sha512',
    }

    logger.info('Getting details for {!r}'.format(snap_name))
    search = session.get(_STORE_SEARCH_URL).content.decode('utf-8')
    search_results = json.loads(search)
    logger.debug('search results {!r}'.format(search_results))

    pkg = search_results['_embedded']['clickindex:package']
    if len(pkg) != 1:
        raise EnvironmentError(
            'Unexpected store result {!r}'.format(search_results))
    download_url = pkg[0].get('anon_download_url', pkg[0]['download_url'])
    download_sha = pkg[0].get('download_sha512')

    if _is_downloaded(download_path, download_sha):
        logger.info('Already downloaded {!r}'.format(snap_name))
    else:
        logger.info('Downloading {!r}'.format(snap_name))
        download = session.get(download_url)
        with open(download_path, 'wb') as f:
            f.write(download.content)
        if _is_downloaded(download_path, download_sha):
            logger.info('Successfully downloaded {!r}'.format(snap_name))
        else:
            raise RuntimeError('Failed to download {!r}'.format(snap_name))


def _is_downloaded(download_path, download_sha):
    if not os.path.exists(download_path):
        return False

    file_sum = hashlib.sha512()
    with open(download_path, 'rb') as f:
        for file_chunk in iter(
                lambda: f.read(file_sum.block_size * 128), b''):
            file_sum.update(file_chunk)
    return download_sha == file_sum.hexdigest()
