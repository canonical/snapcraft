# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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
import logging
import os
import shutil
import sys
import tempfile
import traceback
from textwrap import dedent
from typing import Dict

import click
from raven import Client as RavenClient
from raven.transport import RequestsHTTPTransport

import snapcraft
from snapcraft.config import CLIConfig as _CLIConfig
from snapcraft.internal import errors

from . import echo

# TODO:
# - annotate the part and lifecycle step in the message
# - add link to privacy policy
_MSG_TRACEBACK_PRINT = "Sorry, an error occurred in Snapcraft:"
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


def _is_connected_to_tty() -> bool:
    # Used by inner instance, SNAPCRAFT_HAS_TTY is set by outer instance.
    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        return distutils.util.strtobool(os.getenv("SNAPCRAFT_HAS_TTY", "n")) == 1

    return sys.stdout.isatty()


def _is_reportable_error(exc_info) -> bool:
    # SnapcraftException has explicit `repotable` attribute.
    if isinstance(exc_info[1], errors.SnapcraftException):
        return exc_info[1].get_reportable()

    # Report non-snapcraft errors.
    if not issubclass(exc_info[0], errors.SnapcraftError) and not isinstance(
        exc_info[1], KeyboardInterrupt
    ):
        return True

    # Report SnapcraftReportableError errors.
    if issubclass(exc_info[0], errors.SnapcraftReportableError):
        return True

    return False


def _is_printable_traceback(exc_info, debug) -> bool:
    # Always print with debug.
    if debug:
        return True

    # If it is not a reportable error, then it is not a printable traceback.
    if not _is_reportable_error(exc_info):
        return False

    # Print if not connected to a tty.
    if not _is_connected_to_tty():
        return True

    # Print if not using snap.
    if not snapcraft.internal.common.is_snap():
        return True

    return False


def _handle_sentry_submission(exc_info) -> None:
    # Only attempt to submit reportable errors.
    if not _is_reportable_error(exc_info):
        return

    # Only attempt if running as snap (with Raven requirement).
    if not snapcraft.internal.common.is_snap():
        # Suggest manual reporting instead.
        click.echo(_MSG_MANUALLY_REPORT)
        return

    if _is_send_to_sentry(exc_info):
        try:
            _submit_trace(exc_info)
        except Exception as exc:
            # Failed to send - let user know and suggest manual reporting.
            echo.error(
                "Encountered an issue while trying to submit the report: {}".format(
                    str(exc)
                )
            )
            click.echo(_MSG_MANUALLY_REPORT)
        else:
            # Sent successfully.
            click.echo(_MSG_SEND_TO_SENTRY_THANKS)


def _print_snapcraft_exception_message(exception: errors.SnapcraftException):
    parts = [exception.get_brief()]

    resolution = exception.get_resolution()
    if resolution:
        parts.extend(["", "Recommended resolution:", resolution])

    details = exception.get_details()
    if details:
        parts.extend(["", "Detailed information:", details])

    docs_url = exception.get_docs_url()
    if docs_url:
        parts.extend(["", "For more information, check out:", docs_url])

    echo.error("\n".join(parts))


def _print_exception_message(exception: Exception) -> None:
    if isinstance(exception, errors.SnapcraftException):
        _print_snapcraft_exception_message(exception)
    else:
        echo.error(str(exception))


def _process_exception(exc_info, debug, trace_filepath):
    _print_exception_message(exc_info[1])

    if _is_printable_traceback(exc_info, debug):
        _print_trace_output(exc_info)

    _handle_sentry_submission(exc_info)

    if trace_filepath:
        with open(trace_filepath, "w") as tf:
            _print_trace_output(exc_info, tf)


def _process_inner_exception(exc_info, debug):
    # On inner host, always save trace output and process exception.
    _process_exception(exc_info, debug, TRACEBACK_MANAGED)


def _process_outer_exception(exc_info, debug):
    # Temporary file path to store trace in, if needed.
    trace_filepath = os.path.join(tempfile.mkdtemp(), "trace.txt")

    # On outer host, check if we have traceback generated from inner.
    if os.path.isfile(TRACEBACK_HOST):
        # If traceback file exists, it has already been processed by
        # the inside snapcraft (i.e. printing and sentry submission)
        # we just need to retrieve it for the user if it's reportable.
        if not _is_reportable_error(exc_info):
            return

        shutil.move(TRACEBACK_HOST, trace_filepath)
    else:
        # No traceback found, this must be captured and processed.
        # If reportable or debug is enabled, capture trace file.
        if _is_reportable_error(exc_info) or debug:
            # Only show this message for reportable errors.
            click.echo(_MSG_TRACEBACK_PRINT)

            _process_exception(exc_info, debug, trace_filepath)
        else:
            _process_exception(exc_info, debug, None)

            # No trace file, nothing left to do.
            return

    # This is a reportable error, let the user know where to find the trace.
    click.echo(_MSG_TRACEBACK_LOCATION.format(trace_filepath))


def _get_exception_exit_code(exception: Exception):
    if isinstance(exception, errors.SnapcraftException):
        return exception.get_exit_code()

    if isinstance(exception, errors.SnapcraftError):
        return exception.get_exit_code()

    # If non-snapcraft error, exit code defaults to 1.
    return 1


def exception_handler(  # noqa: C901
    exception_type, exception, exception_traceback, *, debug=False
) -> None:
    """Catch all Snapcraft exceptions unless debugging.

    This function is the global excepthook, properly handling uncaught
    exceptions and determine if they need to be reported.

    These are the rules of engagement:

        - Print exception message (always)

        - Capture trace in temporary file and print path, conditionally if:
          (1) Exception is SnapcraftReportableError or
          (2) Exception is non-Snapcraft error
          (3) debug is true

        - Automatically send bug report if:
          (1) SNAPCRAFT_ENABLE_SILENT_REPORT=y
          (2) "Always" send has been previously set.

        - Prompt to send bug report, conditionally if connected to TTY and:
          (1) Exception is SnapcraftReportableError or
          (2) Exception is non-Snapcraft error

        - Print trace, conditionally if:
          (1) debug is true, or
          (2) Reportable (see above) and either:
              (a) no TTY
              (b) not running as snap

        - Use exit code from snapcraft error (if available), otherwise 1.
    """
    exit_code = _get_exception_exit_code(exception)
    exc_info = (exception_type, exception, exception_traceback)

    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        _process_inner_exception(exc_info, debug)
    else:
        _process_outer_exception(exc_info, debug)

    sys.exit(exit_code)


def _print_trace_output(exc_info, file=sys.stdout) -> None:
    # mypy does not like *exc_info with a kwarg that follows
    # snapcraft/cli/_errors.py:132: error: "print_exception" gets multiple values for keyword argument "file"
    traceback.print_exception(exc_info[0], exc_info[1], exc_info[2], file=file)


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
    # resort to prompting (if connected to TTY).
    if not _is_connected_to_tty():
        return False

    while True:
        try:
            response = _prompt_sentry()
        except click.exceptions.Abort:
            # This was most likely triggered by a KeyboardInterrupt so
            # adding a new line makes things look nice.
            print()
            return False

        if response in _VIEW_VALUES:
            _print_trace_output(exc_info)
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

    return echo.prompt(msg, default="no", value_proc=validate).lower()


def _submit_trace(exc_info):
    kwargs: Dict[str, str] = dict()
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
