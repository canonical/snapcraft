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

import distutils.util
import os
import sys
import shutil
import logging
import tempfile
import traceback
from textwrap import dedent
from typing import Dict  # noqa: F401

import click

from . import echo
import snapcraft
from snapcraft.config import CLIConfig as _CLIConfig
from snapcraft.internal import errors
from snapcraft.internal.build_providers.errors import ProviderExecError

# raven is not available on 16.04
try:
    from raven import Client as RavenClient
    from raven.transport import RequestsHTTPTransport
except ImportError:
    RavenClient = None

# TODO:
# - annotate the part and lifecycle step in the message
# - add link to privacy policy
_MSG_TRACEBACK_PRINT = "Sorry, an error occurred in Snapcraft:"
_MSG_TRACEBACK_FILE = "Sorry, an error occurred in Snapcraft."
_MSG_TRACEBACK_LOCATION = "You can find the traceback in file {!r}."
_MSG_SEND_TO_SENTRY_TRACEBACK_PROMPT = dedent(
    """\
    We would appreciate it if you anonymously reported this issue.
    No other data than the traceback and the version of snapcraft in use will be sent.
    Would you like to send this error data? (Yes/No/Always/View)"""
)
_MSG_SEND_TO_SENTRY_THANKS = "Thank you, sent."
_MSG_SILENT_REPORT = (
    "Sending an error report because SNAPCRAFT_ENABLE_SILENT_REPORT is set."
)
_MSG_ALWAYS_REPORT = dedent(
    """\
    Sending an error report because ALWAYS was selected in a past prompt.
    This behavior can be changed by changing the always_send entry in {}."""
)
_MSG_MANUALLY_REPORT = dedent(
    """\
    We would appreciate it if you created a bug report at
    https://launchpad.net/snapcraft/+filebug with the above text included."""
)

_YES_VALUES = ["yes", "y"]
_NO_VALUES = ["no", "n"]
_ALWAYS_VALUES = ["always", "a"]
_VIEW_VALUES = ["view", "v"]

TRACEBACK_MANAGED = os.path.join(tempfile.gettempdir(), "snapcraft_provider_traceback")
TRACEBACK_HOST = TRACEBACK_MANAGED + ".{}".format(os.getpid())


logger = logging.getLogger(__name__)


def exception_handler(  # noqa: C901
    exception_type, exception, exception_traceback, *, debug=False
) -> None:
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
    exc_info = (exception_type, exception, exception_traceback)
    exit_code = 1
    # We're building directly on host (i.e. no inner instances have sent
    # trace information to sentry), or crash is our own.
    is_snapcraft_host = not os.path.isfile(TRACEBACK_HOST)
    is_snapcraft_managed_host = (
        os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host"
    )
    # When inner instance crashes let it send its trace information. We filter
    # out ProviderExecError to prevent sending another trace dump outside.
    is_snapcraft_error = issubclass(exception_type, errors.SnapcraftError) and (
        is_snapcraft_host or exception_type is not ProviderExecError
    )
    is_snapcraft_reportable_error = issubclass(
        exception_type, errors.SnapcraftReportableError
    )
    is_raven_setup = snapcraft.internal.common.is_snap()
    is_connected_to_tty = (
        # used by inner instance, variable set by outer instance
        (distutils.util.strtobool(os.getenv("SNAPCRAFT_HAS_TTY", "n")) == 1)
        if is_snapcraft_managed_host
        else sys.stdout.isatty()
    )
    ask_to_report = False

    if not is_snapcraft_error:
        ask_to_report = True
    elif is_snapcraft_error and debug:
        exit_code = exception.get_exit_code()
        traceback.print_exception(*exc_info)
    elif is_snapcraft_error and not debug:
        exit_code = exception.get_exit_code()
        echo.error(str(exception))
        if is_snapcraft_reportable_error:
            ask_to_report = True
    else:
        click.echo("Unhandled error case")
        exit_code = -1

    if ask_to_report:
        if is_snapcraft_managed_host or is_snapcraft_host:
            if not is_raven_setup:
                click.echo(_MSG_TRACEBACK_PRINT)
                traceback.print_exception(*exc_info, file=sys.stdout)
                click.echo(_MSG_MANUALLY_REPORT)
            else:
                if is_snapcraft_managed_host:
                    # On inner host, prepare our traceback data to be retrieved.
                    traceback.print_exception(
                        exc_info[0],
                        exc_info[1],
                        exc_info[2],
                        file=open(TRACEBACK_MANAGED, "w"),
                    )

                # Print traceback if not on terminal
                if is_connected_to_tty:
                    click.echo(_MSG_TRACEBACK_FILE)
                else:
                    click.echo(_MSG_TRACEBACK_PRINT)
                    traceback.print_exception(*exc_info, file=sys.stdout)

                # Also ask the user to send traceback data to sentry.
                if _is_send_to_sentry(exc_info):
                    try:
                        _submit_trace(exc_info)
                    except Exception as exc:
                        # we really cannot do much more from this exit handler.
                        echo.error(
                            "Encountered an issue while trying to submit the report: {}".format(
                                str(exc)
                            )
                        )
                    else:
                        click.echo(_MSG_SEND_TO_SENTRY_THANKS)

                if not is_snapcraft_managed_host:
                    trace_filepath = _handle_trace_output(exc_info)
                    click.echo(_MSG_TRACEBACK_LOCATION.format(trace_filepath))
        else:
            # On outer host, check if we have traceback from managed host
            # and retrieve it.
            if os.path.isfile(TRACEBACK_HOST):
                trace_filepath = os.path.join(tempfile.mkdtemp(), "trace.txt")
                shutil.move(TRACEBACK_HOST, trace_filepath)
            else:
                trace_filepath = _handle_trace_output(exc_info)
            click.echo(_MSG_TRACEBACK_LOCATION.format(trace_filepath))

    sys.exit(exit_code)


def _handle_trace_output(exc_info) -> str:
    trace_filepath = os.path.join(tempfile.mkdtemp(), "trace.txt")
    with open(trace_filepath, "w") as trace_file:
        # mypy does not like *exc_info with a kwarg that follows
        # snapcraft/cli/_errors.py:132: error: "print_exception" gets multiple values for keyword argument "file"
        traceback.print_exception(
            exc_info[0], exc_info[1], exc_info[2], file=trace_file
        )
    return trace_filepath


def _is_send_to_sentry(exc_info) -> bool:  # noqa: C901
    # Check to see if error reporting has been disabled
    if (
        distutils.util.strtobool(os.getenv("SNAPCRAFT_ENABLE_ERROR_REPORTING", "y"))
        == 0
    ):
        return False

    # Check the environment to see if we should allow for silent reporting
    if distutils.util.strtobool(os.getenv("SNAPCRAFT_ENABLE_SILENT_REPORT", "n")) == 1:
        click.echo(_MSG_SILENT_REPORT)
        return True

    # If ALWAYS has already been selected from before do not even bother to
    # prompt again.
    config_errors = None
    try:
        with _CLIConfig(read_only=True) as cli_config:
            if cli_config.get_sentry_send_always():
                click.echo(_MSG_ALWAYS_REPORT.format(cli_config.config_path))
                return True
    except errors.SnapcraftInvalidCLIConfigError as config_error:
        echo.warning(
            "There was an issue while trying to retrieve "
            "configuration data: {!s}".format(config_error)
        )
        config_errors = config_error

    # Either ALWAYS has not been selected in a previous run or the
    # configuration for where that value is stored cannot be read, so
    # resort to prompting.
    while True:
        try:
            response = _prompt_sentry()
        except click.exceptions.Abort:
            # This was most likely triggered by a KeyboardInterrupt so
            # adding a new line makes things look nice.
            print()
            return False

        if response in _VIEW_VALUES:
            traceback.print_exception(
                exc_info[0], exc_info[1], exc_info[2], file=sys.stdout
            )
        elif response in _YES_VALUES:
            return True
        elif response in _NO_VALUES:
            return False
        elif response in _ALWAYS_VALUES:
            if config_errors is not None:
                echo.warning(
                    "Not saving choice to always send data to Sentry as "
                    "the configuration file is corrupted.\n"
                    "Please edit and fix or alternatively remove the "
                    "configuration file {!r} from disk.".format(
                        config_errors.config_file
                    )
                )  # type: ignore
            else:
                click.echo("Saving choice to always send data to Sentry")
                with _CLIConfig() as cli_config:
                    cli_config.set_sentry_send_always(True)
            return True
        else:
            echo.warning("Could not determine choice, assuming no.")
            return False


def _prompt_sentry():
    msg = _MSG_SEND_TO_SENTRY_TRACEBACK_PROMPT
    all_valid = _YES_VALUES + _NO_VALUES + _ALWAYS_VALUES + _VIEW_VALUES

    def validate(value):
        if value.lower() in all_valid:
            return value
        raise click.BadParameter("Please choose a valid answer.")

    return click.prompt(msg, default="no", value_proc=validate).lower()


def _submit_trace(exc_info):
    kwargs = dict()  # Dict[str,str]
    if "+git" not in snapcraft.__version__:
        kwargs["release"] = snapcraft.__version__

    client = RavenClient(
        "https://b0fef3e0ced2443c92143ae0d038b0a4:"
        "b7c67d7fa4ee46caae12b29a80594c54@sentry.io/277754",
        transport=RequestsHTTPTransport,
        # Should Raven automatically log frame stacks (including locals)
        # for all calls as it would for exceptions.
        auto_log_stacks=False,
        # Set a name to not send the real hostname.
        name="snapcraft",
        # Removes all stacktrace context variables. This will cripple the
        # functionality of Sentry, as you’ll only get raw tracebacks,
        # but it will ensure no local scoped information is available to the
        # server.
        processors=(
            "raven.processors.RemoveStackLocalsProcessor",
            "raven.processors.SanitizePasswordsProcessor",
        ),
        **kwargs
    )
    client.captureException(exc_info=exc_info)
