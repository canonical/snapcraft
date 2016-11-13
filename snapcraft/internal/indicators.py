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

import os
import sys

from urllib.request import FancyURLopener
from progressbar import (
    AnimatedMarker,
    Bar,
    Percentage,
    ProgressBar,
    UnknownLength,
)


def _init_progress_bar(total_length, destination, message=None):
    if not message:
        message = 'Downloading {!r}'.format(os.path.basename(destination))

    valid_length = total_length and total_length > 0

    if valid_length and is_dumb_terminal():
        widgets = [message, ' ', Percentage()]
        maxval = total_length
    elif valid_length and not is_dumb_terminal():
        widgets = [message,
                   Bar(marker='=', left='[', right=']'),
                   ' ', Percentage()]
        maxval = total_length
    elif not valid_length and is_dumb_terminal():
        widgets = [message]
        maxval = UnknownLength
    else:
        widgets = [message, AnimatedMarker()]
        maxval = UnknownLength

    return ProgressBar(widgets=widgets, maxval=maxval)


def download_requests_stream(request_stream, destination, message=None):
    """This is a facility to download a request with nice progress bars."""

    # Doing len(request_stream.content) may defeat the purpose of a
    # progress bar
    total_length = 0
    if not request_stream.headers.get('Content-Encoding', ''):
        total_length = int(request_stream.headers.get('Content-Length', '0'))

    total_read = 0
    progress_bar = _init_progress_bar(total_length, destination, message)
    progress_bar.start()
    with open(destination, 'wb') as destination_file:
        for buf in request_stream.iter_content(1024):
            destination_file.write(buf)
            total_read += len(buf)
            progress_bar.update(total_read)
    progress_bar.finish()


def download_urllib_source(uri, destination, message=None):
    """This is a facility to download an uri with nice progress bars."""

    def progress_cb(progress_bar, block_num, block_size, total_length):
        if not progress_bar:
            progress_bar = _init_progress_bar(
                total_length, destination, message)
            progress_bar.start()

        total_read = block_num * block_size
        progress_bar.update(
            min(total_read, total_length) if total_length > 0 else total_read)

    progress_bar = None
    FancyURLopener().retrieve(
        uri, destination, lambda n, s, l: progress_cb(progress_bar, n, s, l))

    if progress_bar:
        progress_bar.finish()


def is_dumb_terminal():
    """Return True if on a dumb terminal."""
    is_stdout_tty = os.isatty(sys.stdout.fileno())
    is_term_dumb = os.environ.get('TERM', '') == 'dumb'
    return not is_stdout_tty or is_term_dumb
