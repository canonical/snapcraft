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

from platform import linux_distribution as _linux_distribution
from ._platform import _is_deb_based


class BuildPackageNotFoundError(Exception):

    def __init__(self, package_name):
        self.package_name = package_name

    def __str__(self):
        return ('Could not find a required package in '
                '\'build-packages\': {}'.format(str(self.package_name)))


class PackageNotFoundError(Exception):

    @property
    def message(self):
        message = 'The package {!r} was not found.'.format(
            self.package_name)
        # If the package was multiarch, try to help.
        distro = _linux_distribution()[0]
        if _is_deb_based(distro) and ':' in self.package_name:
            (name, arch) = self.package_name.split(':', 2)
            if arch:
                message += (
                    '\nYou may need to add support for this architecture with '
                    "'dpkg --add-architecture {}'.".format(arch))
        return message

    def __init__(self, package_name):
        self.package_name = package_name


class UnpackError(Exception):

    @property
    def message(self):
        return 'Error while provisioning "{}"'.format(self.package_name)

    def __init__(self, package_name):
        self.package_name = package_name
