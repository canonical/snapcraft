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

from typing import Sequence

from snapcraft.internal.os_release import OsRelease
from ._platform import _is_deb_based
from snapcraft.internal import errors

from typing import List


class RepoError(errors.SnapcraftError):
    pass


class NoNativeBackendError(RepoError):

    fmt = (
        "Native builds aren't supported on {distro}. "
        "You can however use 'snapcraft cleanbuild' with a container."
    )

    def __init__(self):
        try:
            distro = OsRelease().name()
        except errors.OsReleaseNameError:
            distro = "this system"
        super().__init__(distro=distro)


class CacheUpdateFailedError(RepoError):

    fmt = (
        "Failed to update the package cache: "
        "Some files could not be downloaded:{errors}"
        "Check that the sources on your host are configured correctly."
    )

    def __init__(self, errors: str) -> None:
        if errors:
            errors = "\n\n{}\n\n".format(errors.replace(", ", "\n"))
        else:
            errors = " "
        super().__init__(errors=errors)


class BuildPackageNotFoundError(RepoError):

    fmt = "Could not find a required package in 'build-packages': {package}"

    def __init__(self, package):
        super().__init__(package=package)


class BuildPackagesNotInstalledError(RepoError):

    fmt = "Could not install all requested build packages: {packages}"

    def __init__(self, *, packages: List[str]) -> None:
        super().__init__(packages=" ".join(packages))


class PackageBrokenError(RepoError):

    fmt = "The package {package} has unmet dependencies: {deps}"

    def __init__(self, package: str, deps: List[str]) -> None:
        super().__init__(package=package, deps=" ".join(deps))


class PackageNotFoundError(RepoError):
    @property
    def message(self):
        message = "The package {!r} was not found.".format(self.package_name)
        # If the package was multiarch, try to help.
        distro = OsRelease().id()
        if _is_deb_based(distro) and ":" in self.package_name:
            (name, arch) = self.package_name.split(":", 2)
            if arch:
                message += (
                    "\nYou may need to add support for this architecture with "
                    "'dpkg --add-architecture {}'.".format(arch)
                )
        return message

    def __init__(self, package_name):
        self.package_name = package_name

    def __str__(self):
        return self.message


class UnpackError(RepoError):

    fmt = "Error while provisioning {package!r}"

    def __init__(self, package):
        super().__init__(package=package)


class SnapUnavailableError(RepoError):

    fmt = (
        "Failed to install or refresh a snap: {snap_name!r} does not exist "
        "or is not available on the desired channel {snap_channel!r}. "
        "Use `snap info {snap_name}` to get a list of channels the "
        "snap is available on."
    )

    def __init__(self, *, snap_name: str, snap_channel: str) -> None:
        super().__init__(snap_name=snap_name, snap_channel=snap_channel)


class SnapFindError(RepoError):

    fmt = (
        "Could not find the snap {snap_name!r} installed on this host.\n"
        "Install the snap and try again."
    )

    def __init__(self, *, snap_name):
        super().__init__(snap_name=snap_name)


class SnapInstallError(RepoError):

    fmt = "Error while installing snap {snap_name!r} from channel {snap_channel!r}"

    def __init__(self, *, snap_name, snap_channel):
        super().__init__(snap_name=snap_name, snap_channel=snap_channel)


class SnapGetAssertionError(RepoError):

    fmt = (
        "Error while retrieving assertion with parameters "
        "{assertion_params!r}\n"
        "Verify the assertion exists and try again."
    )

    def __init__(self, *, assertion_params: Sequence[str]) -> None:
        super().__init__(assertion_params=assertion_params)


class SnapRefreshError(RepoError):

    fmt = "Error while refreshing snap {snap_name!r} to channel {snap_channel!r}"

    def __init__(self, *, snap_name, snap_channel):
        super().__init__(snap_name=snap_name, snap_channel=snap_channel)


class SnapdConnectionError(RepoError):

    fmt = (
        "Failed to get information for snap {snap_name!r}: "
        "could not connect to {url!r}."
    )

    def __init__(self, snap_name: str, url: str) -> None:
        super().__init__(snap_name=snap_name, url=url)
