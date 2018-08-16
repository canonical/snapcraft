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

from . import echo
from snapcraft.config import CLIConfig as _CLIConfig
from snapcraft.internal import errors
from snapcraft.internal.lxd import errors as lxd_errors

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
_MSG_TRACEBACK_PRINT = dedent(
    """\
    Sorry, Snapcraft ran into an error when trying to running through its
    lifecycle that generated the following traceback:"""
)
_MSG_TRACEBACK_FILE = dedent(
    """\
    Sorry, Snapcraft ran into an error when trying to running through its
    lifecycle that generated a trace that has been put in {!r}."""
)
_MSG_REPORTABLE_ERROR = "This is a problem in snapcraft; a trace has been put in {!r}."
_MSG_SEND_TO_SENTRY_TRACEBACK_PROMPT = dedent(
    """\
    You can anonymously report this issue to the snapcraft developers.
    No other data than this traceback and the version of snapcraft in
    use will be sent.
    Would you like send this error data? (Yes/No/Always)"""
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
_MSG_RAVEN_MISSING = dedent(
    """\
    "Submitting this error to the Snapcraft developers is not possible through the CLI
    without Raven installed.
    If you wish to report this issue, please copy the contents of the previous traceback
    and submit manually at https://launchpad.net/snapcraft/+filebug."""
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

    if not is_snapcraft_error:
        _handle_trace_output(exc_info, is_snapcraft_managed_host, _MSG_TRACEBACK_FILE)
        if not is_raven_setup:
            echo.warning(_MSG_RAVEN_MISSING)
        elif _is_send_to_sentry():
            _submit_trace(exc_info)
            click.echo(_MSG_SEND_TO_SENTRY_THANKS)
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
        if is_snapcraft_reportable_error and is_raven_setup:
            _handle_trace_output(
                exc_info, is_snapcraft_managed_host, _MSG_REPORTABLE_ERROR
            )
            if _is_send_to_sentry():
                _submit_trace(exc_info)
                click.echo(_MSG_SEND_TO_SENTRY_THANKS)
    else:
        click.echo("Unhandled error case")
        exit_code = -1

    sys.exit(exit_code)


def _handle_trace_output(
    exc_info, is_snapcraft_managed_host: bool, trace_file_msg_tmpl: str
) -> None:
    if is_snapcraft_managed_host:
        click.echo(_MSG_TRACEBACK_PRINT)
        traceback.print_exception(*exc_info)
    else:
        trace_filepath = os.path.join(tempfile.mkdtemp(), "trace.txt")
        with open(trace_filepath, "w") as trace_file:
            # mypy does not like *exc_info with a kwarg that follows
            # snapcraft/cli/_errors.py:132: error: "print_exception" gets multiple values for keyword argument "file"
            traceback.print_exception(
                exc_info[0], exc_info[1], exc_info[2], file=trace_file
            )
        click.echo(trace_file_msg_tmpl.format(trace_filepath))


def _is_send_to_sentry() -> bool:  # noqa: C901
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
        response = _prompt_sentry()
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


def _prompt_sentry():
    msg = _MSG_SEND_TO_SENTRY_TRACEBACK_PROMPT
    all_valid = _YES_VALUES + _NO_VALUES + _ALWAYS_VALUES

    def validate(value):
        if value.lower() in all_valid:
            return value
        raise click.BadParameter("Please choose a valid answer.")

    return click.prompt(msg, default="no", value_proc=validate).lower()


def _submit_trace(exc_info):
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
    )
    client.captureException(exc_info=exc_info)
