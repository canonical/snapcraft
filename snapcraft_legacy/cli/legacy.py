# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2022 Canonical Ltd.
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

"""Legacy execution entry points."""

import sys

from ._errors import exception_handler
from ._runner import run  # noqa: F401


def legacy_run():
    try:
        run()
    # Workaround to catch errors.
    except Exception as err:
        exception_handler(type(err), err, err.__traceback__)

    # ensure this call never returns
    sys.exit()
