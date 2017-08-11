# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import contextlib
import functools
import sys

from ._options import get_project_options
from . import echo

import snapcraft.internal.errors


@contextlib.contextmanager
def exception_catcher(debug):
    try:
        yield
    except snapcraft.internal.errors.SnapcraftError as e:
        if debug:
            raise
        else:
            echo.error(str(e))
            sys.exit(e.exit_code)
    except Exception:
        raise


def exception_handler(func):
    """Catch all Snapcraft exceptions unless debugging.

    This is a decorator meant to surround Snapcraft's exit points and handle
    exceptions appropriately. That is defined as:

    When debug=False:
        If exception is a SnapcraftError, just display a nice error and exit 1.
        If exception is NOT a SnapcraftError, raise so traceback is shown.

    When debug=True:
        Raise any exception so traceback is shown.
    """

    @functools.wraps(func)
    def _exception_handler(*args, **kwargs):
        options = get_project_options(**kwargs)
        with exception_catcher(options.debug):
            func(*args, **kwargs)

    return _exception_handler
