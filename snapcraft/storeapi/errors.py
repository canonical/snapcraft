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


import re


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


class StoreRegistrationError(StoreError):

    __FMT_ALREADY_REGISTERED = (
        'The name {snap_name!r} is already taken.\n\n'
        'We can if needed rename snaps to ensure they match the expectations '
        'of most users. If you are the publisher most users expect for '
        '{snap_name!r} then claim the name at {register_claim_url!r}')

    fmt = 'Registration failed.'

    def __init__(self, snap_name, response=None):
        try:
            response_json = response.json()
        except AttributeError:
            response_json = {}

        if response_json.get('status') == 409:
            response_json['register_claim_url'] = self.__get_claim_url(
                response_json.get('register_name_url', ''))
            if response_json['code'] == 'already_registered':
                self.fmt = self.__FMT_ALREADY_REGISTERED

        super().__init__(snap_name=snap_name, **response_json)

    def __get_claim_url(self, url):
        # TODO use the store provided claim url once it is there
        # LP: #1598905
        return re.sub('register-name', 'register-name-dispute', url, count=0)
