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
import tempfile
import traceback
from textwrap import dedent
from typing import Dict  # noqa: F401

import click

from . import echo
import snapcraft
from snapcraft.config import CLIConfig as _CLIConfig
from snapcraft.internal import errors
from snapcraft.internal.lxd import errors as lxd_errors

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
_MSG_SEND_TO_SENTRY_TRACEBACK_PROMPT = dedent(
    """\
    We would appreciate it if you anonymously reported this issue.
    No other data than the traceback ({!r}) and the version of snapcraft in use will be sent.
    Would you like to send this error data? (Yes/No/Always)"""
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


def exception_handler(
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
    # TODO pickle the traceback if on a manged host so the actual host can deal with it.
    exc_info = (exception_type, exception, exception_traceback)
    exit_code = 1
    is_snapcraft_error = issubclass(exception_type, errors.SnapcraftError)
    is_snapcraft_reportable_error = issubclass(
        exception_type, errors.SnapcraftReportableError
    )
    is_raven_setup = RavenClient is not None
    is_snapcraft_managed_host = (
        distutils.util.strtobool(os.getenv("SNAPCRAFT_MANAGED_HOST", "n")) == 1
    )
    ask_to_report = False

    if not is_snapcraft_error:
        ask_to_report = True
    elif is_snapcraft_error and debug:
        exit_code = exception.get_exit_code()
        traceback.print_exception(*exc_info)
    elif is_snapcraft_error and not debug:
        exit_code = exception.get_exit_code()
        # if the error comes from running snapcraft in the container, it
        # has already been displayed so we should avoid that situation
        # of a double error print
        if exception_type != lxd_errors.ContainerSnapcraftCmdError:
            echo.error(str(exception))
        if is_snapcraft_reportable_error:
            ask_to_report = True
    else:
        click.echo("Unhandled error case")
        exit_code = -1

    if ask_to_report:
        if not is_raven_setup or is_snapcraft_managed_host:
            click.echo(_MSG_TRACEBACK_PRINT)
            traceback.print_exception(*exc_info)
            click.echo(_MSG_MANUALLY_REPORT)
        else:
            trace_filepath = _handle_trace_output(exc_info)
            if _is_send_to_sentry(trace_filepath):
                _submit_trace(exc_info)
                click.echo(_MSG_SEND_TO_SENTRY_THANKS)

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


def _is_send_to_sentry(trace_filepath: str) -> bool:  # noqa: C901
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
    try:
        response = _prompt_sentry(trace_filepath)
    except click.exceptions.Abort:
        # This was most likely triggered by a KeyboardInterrupt so
        # adding a new line makes things look nice.
        print()
        return False
    if response in _YES_VALUES:
        return True
    elif response in _NO_VALUES:
        return False
    elif response in _ALWAYS_VALUES:
        if config_errors is not None:
            echo.warning(
                "Not saving choice to always send data to Sentry as "
                "the configuration file is corrupted.\n"
                "Please edit and fix or alternatively remove the "
                "configuration file {!r} from disk.".format(config_errors.config_file)
            )  # type: ignore
        else:
            click.echo("Saving choice to always send data to Sentry")
            with _CLIConfig() as cli_config:
                cli_config.set_sentry_send_always(True)
        return True
    else:
        echo.warning("Could not determine choice, assuming no.")
        return False


def _prompt_sentry(trace_filepath: str):
    msg = _MSG_SEND_TO_SENTRY_TRACEBACK_PROMPT.format(trace_filepath)
    all_valid = _YES_VALUES + _NO_VALUES + _ALWAYS_VALUES

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
