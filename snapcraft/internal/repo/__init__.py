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
import shutil

from snapcraft.internal.errors import MissingCommandError
from . import errors               # noqa
from ._base import BaseRepo        # noqa
from ._base import fix_pkg_config  # noqa
from ._deb import Ubuntu
from ._platform import _is_deb_based


def install_build_packages():
    """Stub method for the platform specific implementation."""
    raise NotImplementedError()


def is_package_installed():
    """Stub method for the platform specific implementation."""
    raise NotImplementedError()


if _is_deb_based():
    Repo = Ubuntu
    install_build_packages = _deb.install_build_packages              # noqa
    is_package_installed = _deb.is_package_installed                  # noqa
else:
    raise RuntimeError(
        'snapcraft is not supported on this operating system')


def check_for_command(command):
    if not shutil.which(command):
        raise MissingCommandError([command])
