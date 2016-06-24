# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


# TODO move to snapcraft.errors --elopio - 2016-06-20
class SnapcraftError(Exception):
    """Base class for all storeapi exceptions.

    :cvar fmt: A format string that daughter classes override

    """
    fmt = 'Daughter classes should redefine this'

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __str__(self):
        return self.fmt.format([], **self.__dict__)


class InvalidCredentialsError(SnapcraftError):

    fmt = 'Invalid credentials: {}.'

    def __init__(self, message):
        super().__init__(message=message)


class StoreError(SnapcraftError):
    """Base class for all storeapi exceptions.

    :cvar fmt: A format string that daughter classes override

    """


class SnapNotFoundError(StoreError):

    fmt = 'The "{name}" for {arch} was not found in {channel}.'

    def __init__(self, name, channel, arch):
        super().__init__(name=name, channel=channel, arch=arch)


class SHAMismatchError(StoreError):

    fmt = 'SHA512 checksum for {path} is not {expected_sha}.'

    def __init__(self, path, expected_sha):
        super().__init__(path=path, expected_sha=expected_sha)


class StoreAuthenticationError(StoreError):

    fmt = 'Authentication error: {}.'

    def __init__(self, message):
        super().__init__(message=message)
