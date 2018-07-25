# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import sys

from urllib.request import urlretrieve
from progressbar import AnimatedMarker, Bar, Percentage, ProgressBar, UnknownLength


def _init_progress_bar(total_length, destination, message=None):
    if not message:
        message = "Downloading {!r}".format(os.path.basename(destination))

    valid_length = total_length and total_length > 0

    if valid_length and is_dumb_terminal():
        widgets = [message, " ", Percentage()]
        maxval = total_length
    elif valid_length and not is_dumb_terminal():
        widgets = [message, Bar(marker="=", left="[", right="]"), " ", Percentage()]
        maxval = total_length
    elif not valid_length and is_dumb_terminal():
        widgets = [message]
        maxval = UnknownLength
    else:
        widgets = [message, AnimatedMarker()]
        maxval = UnknownLength

    return ProgressBar(widgets=widgets, maxval=maxval)


def download_requests_stream(request_stream, destination, message=None, total_read=0):
    """This is a facility to download a request with nice progress bars."""

    # Doing len(request_stream.content) may defeat the purpose of a
    # progress bar
    total_length = 0
    if not request_stream.headers.get("Content-Encoding", ""):
        total_length = int(request_stream.headers.get("Content-Length", "0"))
        # Content-Length in the case of resuming will be
        # Content-Length - total_read so we add back up to have the feel of
        # resuming
        if os.path.exists(destination):
            total_length += total_read

    progress_bar = _init_progress_bar(total_length, destination, message)
    progress_bar.start()

    if os.path.exists(destination):
        mode = "ab"
    else:
        mode = "wb"
    with open(destination, mode) as destination_file:
        for buf in request_stream.iter_content(1024):
            destination_file.write(buf)
            total_read += len(buf)
            progress_bar.update(total_read)
    progress_bar.finish()


class UrllibDownloader(object):
    """This is a facility to download an uri with nice progress bars."""

    def __init__(self, uri, destination, message=None):
        self.uri = uri
        self.destination = destination
        self.message = message
        self.progress_bar = None

    def download(self):
        urlretrieve(self.uri, self.destination, self._progress_callback)

        if self.progress_bar:
            self.progress_bar.finish()

    def _progress_callback(self, block_num, block_size, total_length):
        if not self.progress_bar:
            self.progress_bar = _init_progress_bar(
                total_length, self.destination, self.message
            )
            self.progress_bar.start()

        total_read = block_num * block_size
        self.progress_bar.update(
            min(total_read, total_length) if total_length > 0 else total_read
        )


def download_urllib_source(uri, destination, message=None):
    UrllibDownloader(uri, destination, message).download()


def is_dumb_terminal():
    """Return True if on a dumb terminal."""
    is_stdout_tty = os.isatty(1)
    is_term_dumb = os.environ.get("TERM", "") == "dumb"
    is_windows = sys.platform == "win32"
    return not is_stdout_tty or is_term_dumb or is_windows
