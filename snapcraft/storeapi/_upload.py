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
import os
import re

from .common import (
    get_oauth_session,
    is_scan_completed,
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


def upload(binary_filename, metadata_filename='', metadata=None, config=None):
    """Create a new upload based on a snap package."""

    # validate package filename
    pattern = (r'(.*/)?(?P<name>[\w\-_\.]+)_'
               '(?P<version>[\d\.]+)_(?P<arch>\w+)\.snap')
    match = re.match(pattern, binary_filename)
    if not match:
        logger.info('Invalid package filename.')
        return
    name = match.groupdict()['name']

    logger.info('Uploading files...')
    data = upload_files(binary_filename, config=config)
    success = data.get('success', False)
    errors = data.get('errors', [])
    if not success:
        logger.info('Upload failed:\n\n%s\n', '\n'.join(errors))
        return False

    logger.info('Uploading new version...')
    meta = read_metadata(metadata_filename)
    meta.update(metadata or {})
    result = upload_app(name, data, metadata=meta, config=config)
    success = result.get('success', False)
    errors = result.get('errors', [])
    app_url = result.get('application_url', '')
    revision = result.get('revision')

    if success:
        logger.info('Application uploaded successfully.')
        if revision:
            logger.info('Uploaded as revision %s.', revision)
    else:
        logger.info('Upload did not complete.')

    if errors:
        logger.info('Some errors were detected:\n\n%s\n\n',
                    '\n'.join(errors))

    if app_url:
        logger.info('Please check out the application at: %s.\n',
                    app_url)

    return success


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
        result['errors'] = ['No valid credentials found.']
        return result

    files = {'binary': open(binary_filename, 'rb')}

    try:
        response = session.post(
            unscanned_upload_url,
            files=files)
        if response.ok:
            response_data = response.json()
            result.update({
                'success': response_data.get('successful', True),
                'upload_id': response_data['upload_id'],
                'binary_filesize': os.path.getsize(binary_filename),
                'source_uploaded': 'source' in files,
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
        # make sure to close any open files used for request
        for fd in files.values():
            fd.close()

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
    upload_url = get_upload_url(name)

    result = {'success': False, 'errors': [],
              'application_url': '', 'revision': None}

    session = get_oauth_session(config)
    if session is None:
        result['errors'] = ['No valid credentials found.']
        return result

    if metadata is None:
        metadata = {}

    try:
        data = get_post_data(upload_data, metadata=metadata)
        files = get_post_files(metadata=metadata)

        response = session.post(upload_url, data=data, files=files)
        if response.ok:
            response_data = response.json()
            status_url = response_data['status_url']
            logger.info('Package submitted to %s', upload_url)
            logger.info('Checking package status...')
            completed, data = get_scan_data(session, status_url)
            if completed:
                logger.info('Package scan completed.')
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
                        'Please check the status later at: %s.' % (
                            status_web_url),
                    )
            result['application_url'] = data.get('application_url', '')
        else:
            logger.error(
                'There was an error uploading the application.\n'
                'Reason: %s\n'
                'Text: %s',
                response.reason, response.text)
            result['errors'] = [response.text]
    except Exception as err:
        logger.exception(
            'There was an error uploading the application.')
        result['errors'] = [str(err)]
    finally:
        # make sure to close any open files used for request
        for fname, fd in files:
            fd.close()

    return result


def get_upload_url(name):
    """Return the url of the uploaded package."""
    store_api_url = os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                                   UBUNTU_STORE_API_ROOT_URL)
    upload_url = urljoin(store_api_url, 'click-package-upload/')
    upload_url += "%s/" % quote_plus(name)
    return upload_url


def get_post_data(upload_data, metadata=None):
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


def get_post_files(metadata=None):
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


def get_scan_data(session, status_url):
    """Return metadata about the state of the upload scan process."""
    # initial retry after 5 seconds
    # linear backoff after that
    # abort after 5 retries
    @retry(terminator=is_scan_completed,
           retries=SCAN_STATUS_POLL_RETRIES,
           delay=SCAN_STATUS_POLL_DELAY,
           backoff=1, logger=logger)
    def get_status():
        return session.get(status_url)

    response, aborted = get_status()

    completed = False
    data = {}
    if not aborted:
        completed = is_scan_completed(response)
        data = response.json()
    return completed, data
