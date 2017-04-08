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

from platform import linux_distribution as _linux_distribution

from ._deb import Ubuntu

_DEB_BASED_PLATFORM = [
    'Ubuntu',
    'Debian',
    'elementary',
    # Not sure what was going on when this was added.
    '"elementary"',
    'debian',
    'neon',
]


def _is_deb_based(distro):
    return distro in _DEB_BASED_PLATFORM


def _get_repo_for_platform():
    distro = _linux_distribution()[0]
    if _is_deb_based(distro):
        return Ubuntu
    else:
        raise RuntimeError(
            'snapcraft is not supported on this operating system '
            '({!r})'.format(distro))
