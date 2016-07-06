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
import functools
import os
from time import sleep

import requests
from concurrent.futures import ThreadPoolExecutor
from progressbar import (
    AnimatedMarker,
    Bar,
    Percentage,
    ProgressBar,
    UnknownLength,
)
from requests_toolbelt import (MultipartEncoder, MultipartEncoderMonitor)

from snapcraft.storeapi import (
    constants,
    errors
)


logger = logging.getLogger(__name__)


def _update_progress_bar(progress_bar, maximum_value, monitor):
    if monitor.bytes_read <= maximum_value:
        progress_bar.update(monitor.bytes_read)


def upload_files(binary_filename, updown_client):
    """Upload a binary file to the Store.

    Submit a file to the Store upload service and return the
    corresponding upload_id.
    """
    result = {'success': False, 'errors': []}

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
        response = updown_client.upload(monitor)

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


class StatusTracker:

    __messages = {
        'being_processed': 'Processing...',
        'ready_to_release': 'Ready to release!',
        'need_manual_review': 'Will need manual review...',
    }

    def __init__(self, status_details_url):
        self.__status_details_url = status_details_url
        self._set_dummy_status()

    def track(self):
        widgets = [self._get_message(), AnimatedMarker()]

        progress_indicator = ProgressBar(widgets=widgets, maxval=UnknownLength)
        progress_indicator.start()
        indicator_count = 0
        while not self.__current_status['processed']:
            self._update_status()
            widgets[0] = self._get_message()
            indicator_count += 1
            sleep(constants.SCAN_STATUS_POLL_DELAY)
            progress_indicator.update(indicator_count)
        progress_indicator.finish()

        return self.__current_status

    def _get_message(self):
        return self.__messages.get(self.__current_status['code'],
                                   self.__current_status['code'])

    def _update_status(self):
        response = requests.get(self.__status_details_url)
        if response.ok:
            self.__current_status = response.json()
        else:
            self._set_dummy_status()

    def _set_dummy_status(self):
        self.__current_status = {'processed': False, 'code': 'being_processed'}
