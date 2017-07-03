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

import logging
import sys as _sys
import distro as _distro

logger = logging.getLogger(__name__)
_DEB_BASED_PLATFORM = [
    'ubuntu',
    'elementary',
    'debian',
    'neon',
]


def _is_deb_based(distro=None):
    if not distro:
        distro = _distro.id()
    return distro in _DEB_BASED_PLATFORM


def _get_repo_for_platform():
    distro = _distro.id()
    if _is_deb_based(distro):
        from ._deb import Ubuntu
        return Ubuntu
    else:
        if _sys.platform == 'linux':
            system = distro
        else:
            system = _sys.platform
        from ._base import DummyRepo
        logger.warning('Currently only `cleanbuild` will work on this system '
                       '({!r})'.format(system))
        return DummyRepo
