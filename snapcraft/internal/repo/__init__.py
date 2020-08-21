# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2020 Canonical Ltd
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

"""Operations with platform specific package repositories."""

from snapcraft.internal.os_release import OsRelease
import snapcraft.internal.errors

try:
    distro = OsRelease().id()
except snapcraft.internal.errors.OsReleaseIdError:
    distro = "unknown"

if distro == "ubuntu":
    from .apt import AptRepo
else:
    AptRepo = None  # type: ignore

from . import errors  # noqa
from . import snaps  # noqa
from .fixups import fix_pkg_config  # noqa
