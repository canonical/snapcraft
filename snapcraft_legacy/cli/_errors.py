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

import logging
import os
import shutil
import sys
import tempfile
import traceback

import click
from craft_application.util import strtobool
import craft_store

import snapcraft_legacy
from snapcraft_legacy.internal import errors

from . import echo

# TODO:
# - annotate the part and lifecycle step in the message
# - add link to privacy policy
_MSG_TRACEBACK_PRINT = "Sorry, an error occurred in Snapcraft:"
_MSG_TRACEBACK_LOCATION = "You can find the traceback in file {!r}."

TRACEBACK_MANAGED = os.path.join(tempfile.gettempdir(), "snapcraft_provider_traceback")
TRACEBACK_HOST = TRACEBACK_MANAGED + ".{}".format(os.getpid())


logger = logging.getLogger(__name__)


def _is_connected_to_tty() -> bool:
    # Used by inner instance, SNAPCRAFT_HAS_TTY is set by outer instance.
    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        return strtobool(os.getenv("SNAPCRAFT_HAS_TTY", "n")) == 1

    return sys.stdout.isatty()


def _is_reportable_error(exc_info) -> bool:
    # SnapcraftException has explicit `repotable` attribute.
    if isinstance(exc_info[1], errors.SnapcraftException):
        return exc_info[1].get_reportable()

    # Report non-snapcraft errors.
    if (
        not issubclass(exc_info[0], errors.SnapcraftError)
        and not issubclass(exc_info[0], craft_store.errors.CraftStoreError)
        and not isinstance(exc_info[1], KeyboardInterrupt)
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
    if not snapcraft_legacy.internal.common.is_snap():
        return True

    return False


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
        # the inside snapcraft (i.e. printing and error handling)
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

        - Use exit code from snapcraft_legacy error (if available), otherwise 1.
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
