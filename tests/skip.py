# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import functools
from typing import Any, Callable

from unittest import skipUnless

from tests import os_release


def skip_unless_codename(codename, message: str) -> Callable[..., Callable[..., None]]:
    if type(codename) is str:
        codename = [codename]

    def _wrap(func: Callable[..., None]) -> Callable[..., None]:
        actual_codename = os_release.get_version_codename()

        @functools.wraps(func)
        @skipUnless(actual_codename in codename, message)
        def _skip_test(*args: Any, **kwargs: Any) -> None:
            func(*args, **kwargs)

        return _skip_test

    return _wrap
