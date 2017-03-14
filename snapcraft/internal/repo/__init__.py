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

import platform
import shutil

from snapcraft.internal.errors import MissingCommandError

_DEB_BASED_PLATFORM = [
    'Ubuntu',
    'Debian',
]


def _is_deb_based():
    return platform.linux_distribution()[0] in _DEB_BASED_PLATFORM


# stubs for imports
class Repo:
    """Stub for a BaseRepo implementation.

    The class need to be overridden during the platform check.
    """
    def __init__(self):
        raise NotImplementedError()


def get_packages_for_source_type():
    """Stub method for the platform specific implementation."""
    raise NotImplementedError()


def install_build_packages():
    """Stub method for the platform specific implementation."""
    raise NotImplementedError()


def is_package_installed():
    """Stub method for the platform specific implementation."""
    raise NotImplementedError()


if _is_deb_based():
    from . import _deb
    Repo = _deb.Ubuntu                                                # noqa
    get_packages_for_source_type = _deb.get_packages_for_source_type  # noqa
    install_build_packages = _deb.install_build_packages              # noqa
    is_package_installed = _deb.is_package_installed                  # noqa
else:
    raise RuntimeError(
        'snapcraft is not supported on this operating system')

from . import errors               # noqa
from ._base import BaseRepo        # noqa
from ._base import fix_pkg_config  # noqa
# This is for legacy reasons
from ._deb import Ubuntu           # noqa


def check_for_command(command):
    if not shutil.which(command):
        raise MissingCommandError([command])
