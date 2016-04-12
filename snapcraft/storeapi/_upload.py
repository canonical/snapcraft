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
from __future__ import absolute_import, unicode_literals
import json
import logging
import functools
import time
import os

import requests
from concurrent.futures import ThreadPoolExecutor
from progressbar import (ProgressBar, Percentage, Bar, AnimatedMarker)
from requests_toolbelt import (MultipartEncoder, MultipartEncoderMonitor)

from .common import (
    get_oauth_session,
    retry,
)
from .compat import open, quote_plus, urljoin
from .constants import (
    UBUNTU_STORE_API_ROOT_URL,
    UBUNTU_STORE_UPLOAD_ROOT_URL,
    SCAN_STATUS_POLL_DELAY,
    SCAN_STATUS_POLL_RETRIES,
)

logger = logging.getLogger(__name__)


def _update_progress_bar(progress_bar, maximum_value, monitor):
    if monitor.bytes_read <= maximum_value:
        progress_bar.update(monitor.bytes_read)


def upload(binary_filename, snap_name, metadata_filename='', metadata=None,
           config=None):
    """Create a new upload based on a snap package."""

    data = upload_files(binary_filename, config=config)
    success = data.get('success', False)
    errors = data.get('errors', [])
    if not success:
        logger.info('Upload failed:\n\n%s\n', '\n'.join(errors))
        return dict(success=False)

    meta = read_metadata(metadata_filename)
    meta.update(metadata or {})
    result = upload_app(snap_name, data, metadata=meta, config=config)
    return result


def upload_files(binary_filename, config=None):
    """Upload a binary file to the Store.

    Submit a file to the Store upload service and return the
    corresponding upload_id.
    """
    updown_url = os.environ.get('UBUNTU_STORE_UPLOAD_ROOT_URL',
                                UBUNTU_STORE_UPLOAD_ROOT_URL)
    unscanned_upload_url = urljoin(updown_url, 'unscanned-upload/')

    result = {'success': False, 'errors': []}

    session = get_oauth_session(config)
    if session is None:
        result['errors'] = [
            'No valid credentials found. Have you run "snapcraft login"?']
        return result

    try:
        binary_file_size = os.path.getsize(binary_filename)
        binary_file = open(binary_filename, 'rb')
        encoder = MultipartEncoder(
            fields={
                'binary': ('filename', binary_file, 'application/octet-stream')
            }
        )

        # Create a progress bar that looks like: Uploading foo [==  ] 50%
        progress_bar = ProgressBar(
            widgets=['Uploading {} '.format(binary_filename),
                     Bar(marker='=', left='[', right=']'), ' ', Percentage()],
            maxval=os.path.getsize(binary_filename))
        progress_bar.start()
        # Print a newline so the progress bar has some breathing room.
        logger.info('')

        # Create a monitor for this upload, so that progress can be displayed
        monitor = MultipartEncoderMonitor(
            encoder, functools.partial(_update_progress_bar, progress_bar,
                                       binary_file_size))

        # Begin upload
        response = session.post(
            unscanned_upload_url,
            data=monitor, headers={'Content-Type': monitor.content_type})

        # Make sure progress bar shows 100% complete
        progress_bar.finish()

        if response.ok:
            response_data = response.json()
            result.update({
                'success': response_data.get('successful', True),
                'upload_id': response_data['upload_id'],
                'binary_filesize': os.path.getsize(binary_filename),
                'source_uploaded': False,
            })
        else:
            logger.error(
                'There was an error uploading the package.\n'
                'Reason: %s\n'
                'Text: %s',
                response.reason, response.text)
            result['errors'] = [response.text]
    except Exception as err:
        logger.exception(
            'An unexpected error was found while uploading files.')
        result['errors'] = [str(err)]
    finally:
        # Close the open file
        binary_file.close()

    return result


def read_metadata(metadata_filename):
    """Return a dictionary of metadata as read from a json file."""
    if metadata_filename:
        with open(metadata_filename, 'r') as metadata_file:
            # file is automatically closed by context manager
            metadata = json.load(metadata_file)
    else:
        metadata = {}

    return metadata


def upload_app(name, upload_data, metadata=None, config=None):
    """Request a new upload to be created for a given upload_id."""
    if metadata is None:
        metadata = {}

    upload_url = get_upload_url(name)

    result = {'success': False, 'errors': [],
              'application_url': '', 'revision': None}

    session = get_oauth_session(config)
    if session is None:
        result['errors'] = [
            'No valid credentials found. Have you run "snapcraft login"?']
        return result

    files = []
    try:
        data = get_post_data(upload_data, metadata)
        files = get_post_files(metadata)

        result = _upload_files(session, upload_url, data, files, result)
    except Exception as err:
        logger.exception(
            'There was an error uploading the application.')
        result['errors'] = [str(err)]
    finally:
        # make sure to close any open files used for request
        for fname, fd in files:
            fd.close()

    return result


def _upload_files(session, upload_url, data, files, result):
    response = session.post(upload_url, data=data, files=files)
    if response.ok:
        response_data = response.json()
        status_url = response_data['status_url']

        # This is just a waiting game, so we'll show an indeterminate
        # AnimatedMarker for it.
        progress_indicator = ProgressBar(
            widgets=['Checking package status... ', AnimatedMarker()],
            maxval=7)
        progress_indicator.start()

        # Execute the package scan in another thread so we can update the
        # progress indicator.
        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(get_scan_data, session, status_url)

            count = 0
            while not future.done():
                # Annoyingly, there doesn't seem to be a way to actually
                # make a progress indicator that will go on forever, so we
                # need to restart this one each time we reach the end of
                # its animation.
                if count >= 7:
                    progress_indicator.start()
                    count = 0

                # Actually update the progress indicator
                progress_indicator.update(count)
                count += 1
                time.sleep(0.15)

            # Grab the results from the package scan
            completed, data = future.result()

        progress_indicator.finish()

        if completed:
            message = data.get('message', '')
            if not message:
                result['success'] = True
                result['revision'] = data.get('revision')
            else:
                result['errors'] = [message]
        else:
            result['errors'] = [
                'Package scan took too long.',
            ]
            status_web_url = response_data.get('web_status_url')
            if status_web_url:
                result['errors'].append(
                    'Please check the status later at: {}.'.format(
                        status_web_url),
                )
        result['application_url'] = data.get('application_url', '')
    else:
        logger.error(
            'There was an error uploading the application.\n'
            'Reason: {}\n'
            'Text: {}'.format(response.reason, response.text))
        result['errors'] = [response.text]
    return result


def get_upload_url(name):
    """Return the url of the uploaded package."""
    store_api_url = os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                                   UBUNTU_STORE_API_ROOT_URL)
    upload_url = urljoin(store_api_url, 'click-package-upload/')
    upload_url += "%s/" % quote_plus(name)
    return upload_url


def get_post_data(upload_data, metadata):
    """Return the data to be posted in order to create the upload."""
    data = {
        'updown_id': upload_data['upload_id'],
        'binary_filesize': upload_data['binary_filesize'],
        'source_uploaded': upload_data['source_uploaded'],
    }
    data.update({
        key: value
        for (key, value) in metadata.items()
        if key not in (
            # make sure not to override upload_id, binary_filesize and
            # source_uploaded
            'upload_id', 'binary_filesize', 'source_uploaded',
            # skip files as they will be added to the files argument
            'icon_256', 'icon', 'screenshots',
        )
    })
    return data


def get_post_files(metadata):
    """Return data about files to upload during the package upload request."""
    files = []

    icon = metadata.get('icon', metadata.get('icon_256', ''))
    if icon:
        icon_file = open(icon, 'rb')
        files.append(('icon_256', icon_file))

    screenshots = metadata.get('screenshots', [])
    for screenshot in screenshots:
        screenshot_file = open(screenshot, 'rb')
        files.append(('screenshots', screenshot_file))

    return files


def is_scan_completed(response):
    """Return True if the response indicates the scan process completed."""
    if response is None:
        # To cope with spurious connection failures lacking a proper response:
        # either we'll retry and succeed or we fail for all retries and report
        # an error.
        return False
    if response.ok:
        return response.json().get('completed', False)
    return False


def get_scan_status(session, url):
    try:
        resp = session.get(url)
        return resp
    except (requests.ConnectionError, requests.HTTPError):
        # Something went wrong and we couldn't acquire the status. Upper
        # level (is_scan_completed) will deal with the None response
        # meaning we don't know the status. This avoid a spurious
        # connection error breaking an upload for a wrong reason.
        return None


def get_scan_data(session, status_url):
    """Return metadata about the state of the upload scan process."""
    # initial retry after 5 seconds
    # linear backoff after that
    # abort after 5 retries
    @retry(terminator=is_scan_completed,
           retries=SCAN_STATUS_POLL_RETRIES,
           delay=SCAN_STATUS_POLL_DELAY,
           backoff=1)
    def get_status():
        return get_scan_status(session, status_url)

    response, aborted = get_status()

    completed = False
    data = {}
    if not aborted:
        completed = is_scan_completed(response)
        data = response.json()
    return completed, data
