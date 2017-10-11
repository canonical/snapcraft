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

from snapcraft.internal.common import get_os_release_info
from snapcraft.internal import errors


class RepoError(errors.SnapcraftError):
    pass


class NoNativeBackendError(RepoError):

    fmt = ("Native builds aren't supported on {distro}. "
           "You can however use 'snapcraft cleanbuild' with a container.")

    def __init__(self):
        super().__init__(distro=get_os_release_info()['NAME'])


class PackageNotFoundError(RepoError):

    fmt = 'The package {package!r} was not found.'

    def __init__(self, package):
        super().__init__(package=package)

    # Aliases for the benefit of existing plugin code

    @property
    def message(self):
        return str(self)

    @property
    def package_name(self):
        return self.package


class BuildPackageNotFoundError(PackageNotFoundError):

    fmt = "Could not find a required package in 'build-packages': {package}"


class UnpackError(RepoError):

    fmt = 'Error while provisioning {package!r}'

    def __init__(self, package):
        super().__init__(package=package)


class SnapInstallError(RepoError):

    fmt = ('Error while installing snap {snap_name!r} from channel '
           '{snap_channel!r}')

    def __init__(self, *, snap_name, snap_channel):
        super().__init__(snap_name=snap_name, snap_channel=snap_channel)


class SnapRefreshError(RepoError):

    fmt = ('Error while refreshing snap {snap_name!r} to channel '
           '{snap_channel!r}')

    def __init__(self, *, snap_name, snap_channel):
        super().__init__(snap_name=snap_name, snap_channel=snap_channel)
