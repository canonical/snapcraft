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

import os
import sys
import traceback
from textwrap import dedent

from . import echo
from snapcraft.internal import errors

import click
# raven is not available on 16.04
try:
    from raven import Client as RavenClient
    from raven.transport import RequestsHTTPTransport
except ImportError:
    RavenClient = None

# TODO:
# - annotate the part and lifecycle step in the message
# - add link to privacy policy
# - add Always option
_MSG_SEND_TO_SENTRY_TRACEBACK = dedent("""\
    Sorry, Snapcraft ran into an error when trying to running through its
    lifecycle that generated the following traceback:""")
_MSG_SEND_TO_SENTRY_TRACEBACK_CONFIRM = dedent("""\
    You can anonymously report this issue to the snapcraft developers.
    No other data than this traceback and the version of snapcraft in use will
    be sent.
    Would you like send this error data?""")
_MSG_SEND_TO_SENTRY_ENV = dedent("""\
    Sending error data: SNAPCRAFT_SEND_ERROR_DATA is set to 'y'.""")
_MSG_SEND_TO_SENTRY_THANKS = 'Thank you for sending the report.'


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

    if debug:
        traceback.print_exception(
            exception_type, exception, exception_traceback)

    if RavenClient is not None and not is_snapcraft_error:
        is_env_send_data = os.environ.get(
            'SNAPCRAFT_SEND_ERROR_DATA', 'n') == 'y'
        msg = _MSG_SEND_TO_SENTRY_TRACEBACK_CONFIRM
        click.echo(_MSG_SEND_TO_SENTRY_TRACEBACK)
        traceback.print_exception(
            exception_type, exception, exception_traceback)
        if is_env_send_data:
            click.echo(_MSG_SEND_TO_SENTRY_ENV)
            _submit_trace(exception)
            click.echo(_MSG_SEND_TO_SENTRY_THANKS)
        elif click.confirm(msg):
            _submit_trace(exception)
            click.echo(_MSG_SEND_TO_SENTRY_THANKS)
    else:
        should_print_error = not debug and (
            exception_type != errors.ContainerSnapcraftCmdError)

        exit_code = exception.get_exit_code()
        if should_print_error:
            echo.error(str(exception))

    sys.exit(exit_code)


def _submit_trace(exception):
    client = RavenClient(
        'https://b0fef3e0ced2443c92143ae0d038b0a4:'
        'b7c67d7fa4ee46caae12b29a80594c54@sentry.io/277754',
        transport=RequestsHTTPTransport)
    try:
        raise exception
    except Exception:
        client.captureException()
