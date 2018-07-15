# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

from unittest import skipUnless

from snapcraft.internal import errors, os_release


# TODO: remove this duplicated functionality once __init__ for
#       the testing packages are cleaned up. (LP: #1729593)
def skip_unless_codename(codename, message):
    def _wrap(func):
        release = os_release.OsRelease()
        actual_codename = None
        with contextlib.suppress(errors.OsReleaseCodenameError):
            actual_codename = release.version_codename()

        @functools.wraps(func)
        @skipUnless(actual_codename == codename, message)
        def _skip_test(*args, **kwargs):
            func(*args, **kwargs)

        return _skip_test

    return _wrap
