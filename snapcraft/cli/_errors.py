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
    exceptions and determine if they need to be reported.

    These are the rules of engagement:

        - a non snapcraft handled error occurs and raven is setup,
          so we go over confirmation logic showing the relevant traceback
        - a non snapcraft handled error occurs and raven is not setup,
          so we just show the traceback
        - a snapcraft handled error occurs, debug=True so a traceback
          is shown
        - a snapcraft handled error occurs, debug=False so only the
          exception message is shown
    """
    exit_code = 1
    is_snapcraft_error = issubclass(exception_type, errors.SnapcraftError)
    is_raven_setup = RavenClient is not None

    if is_raven_setup and not is_snapcraft_error:
        is_env_send_data = os.getenv(
            'SNAPCRAFT_SEND_ERROR_DATA', 'n') == 'y'
        msg = _MSG_SEND_TO_SENTRY_TRACEBACK_CONFIRM
        click.echo(_MSG_SEND_TO_SENTRY_TRACEBACK)
        traceback.print_exception(
            exception_type, exception, exception_traceback)
        if is_env_send_data or click.confirm(msg):
            if is_env_send_data:
                click.echo(_MSG_SEND_TO_SENTRY_ENV)
            _submit_trace(exception)
            click.echo(_MSG_SEND_TO_SENTRY_THANKS)
    elif not is_snapcraft_error:
        traceback.print_exception(
            exception_type, exception, exception_traceback)
    elif is_snapcraft_error and debug:
        exit_code = exception.get_exit_code()
        traceback.print_exception(
            exception_type, exception, exception_traceback)
    elif is_snapcraft_error and not debug:
        exit_code = exception.get_exit_code()
        # if the error comes from running snapcraft in the container, it
        # has already been displayed so we should avoid that situation
        # of a double error print
        if exception_type != errors.ContainerSnapcraftCmdError:
            echo.error(str(exception))
    else:
        click.echo('Unhandled error case')
        exit_code = -1

    sys.exit(exit_code)


def _submit_trace(exception):
    client = RavenClient(
        'https://b0fef3e0ced2443c92143ae0d038b0a4:'
        'b7c67d7fa4ee46caae12b29a80594c54@sentry.io/277754',
        transport=RequestsHTTPTransport,
        # Should Raven automatically log frame stacks (including locals)
        # for all calls as it would for exceptions.
        auto_log_stacks=False,
        # Removes all stacktrace context variables. This will cripple the
        # functionality of Sentry, as you’ll only get raw tracebacks,
        # but it will ensure no local scoped information is available to the
        # server.
        processors=('raven.processors.RemoveStackLocalsProcessor',))
    try:
        raise exception
    except Exception:
        client.captureException()
