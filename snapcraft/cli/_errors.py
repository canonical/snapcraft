# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

import sys
import traceback

from . import echo
from snapcraft.internal import errors

import click
# raven is not available on 16.04
try:
    from raven import Client as RavenClient
    from raven.transport import RequestsHTTPTransport
except ImportError:
    RavenClient = None

_SEND_TO_SENTRY = (
    'Sorry, Snapcraft had an internal error.\n'
    'To help us improve, would you like to send error data?'
)


def exception_handler(exception_type, exception, exception_traceback, *,
                      debug=False):
    """Catch all Snapcraft exceptions unless debugging.

    This function is the global excepthook, properly handling uncaught
    exceptions. "Proper" being defined as:

    When debug=False:
        - If exception is a SnapcraftError, just display a nice error and exit
          according to the exit code in the exception.
        - If exception is NOT a SnapcraftError, show traceback and exit 1

    When debug=True:
        - If exception is a SnapcraftError, show traceback and exit according
          to the exit code in the exception.
        - If exception is NOT a SnapcraftError, show traceback and exit 1
    """

    exit_code = 1
    is_snapcraft_error = issubclass(exception_type, errors.SnapcraftError)

    if debug or not is_snapcraft_error:
        traceback.print_exception(
            exception_type, exception, exception_traceback)

    if not is_snapcraft_error and RavenClient is not None:
        _submit_trace(exception)

    should_print_error = not debug and (
        exception_type != errors.ContainerSnapcraftCmdError)

    if is_snapcraft_error:
        exit_code = exception.get_exit_code()
        if should_print_error:
            echo.error(str(exception))

    sys.exit(exit_code)


def _submit_trace(exception):
    client = RavenClient(
        'https://b0fef3e0ced2443c92143ae0d038b0a4:'
        'b7c67d7fa4ee46caae12b29a80594c54@sentry.io/277754',
        transport=RequestsHTTPTransport)
    if click.confirm(_SEND_TO_SENTRY):
        try:
            raise exception
        except Exception:
            client.captureException()
