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

from simplejson.scanner import JSONDecodeError

from snapcraft.internal.errors import SnapcraftError


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


class StoreAccountInformationError(StoreError):

    fmt = 'Error fetching account information from store: {error}'

    def __init__(self, response):
        error = '{} {}'.format(response.status_code, response.reason)
        try:
            response_json = response.json()
            if 'error_list' in response_json:
                error = ' '.join(
                    error['message'] for error in response_json['error_list'])
        except JSONDecodeError:
            pass
        super().__init__(error=error)


class StoreKeyRegistrationError(StoreError):

    fmt = 'Key registration failed: {error}'

    def __init__(self, response):
        error = '{} {}'.format(response.status_code, response.reason)
        try:
            response_json = response.json()
            if 'error_list' in response_json:
                error = ' '.join(
                    error['message'] for error in response_json['error_list'])
        except JSONDecodeError:
            pass
        super().__init__(error=error)


class StoreRegistrationError(StoreError):

    __FMT_ALREADY_REGISTERED = (
        'The name {snap_name!r} is already taken.\n\n'
        'We can if needed rename snaps to ensure they match the expectations '
        'of most users. If you are the publisher most users expect for '
        '{snap_name!r} then claim the name at {register_name_url!r}')

    __FMT_RESERVED = (
        'The name {snap_name!r} is reserved.\n\n'
        'If you are the publisher most users expect for '
        '{snap_name!r} then please claim the name at {register_name_url!r}')

    __FMT_RETRY_WAIT = (
        'You must wait {retry_after} seconds before trying to register '
        'your next snap.')

    fmt = 'Registration failed.'

    __error_messages = {
        'already_registered': __FMT_ALREADY_REGISTERED,
        'reserved_name': __FMT_RESERVED,
        'register_window': __FMT_RETRY_WAIT,
    }

    def __init__(self, snap_name, response):
        try:
            response_json = response.json()
        except JSONDecodeError:
            response_json = {}

        error_code = response_json.get('code')
        if error_code:
            # we default to self.fmt in case error_code is not mapped yet.
            self.fmt = self.__error_messages.get(error_code, self.fmt)

        super().__init__(snap_name=snap_name, **response_json)


class StoreUploadError(StoreError):

    fmt = (
        'There was an error uploading the package.\n'
        'Reason: {reason!r}\n'
        'Text: {text!r}')

    def __init__(self, response):
        super().__init__(reason=response.reason, text=response.text)


class StorePushError(StoreError):

    __FMT_NOT_REGISTERED = (
        'Sorry, try `snapcraft register {snap_name}` before pushing again.')

    fmt = 'Received {status_code!r}: {text!r}'

    def __init__(self, snap_name, response):
        try:
            response_json = response.json()
        except (AttributeError, JSONDecodeError):
            response_json = {}

        if response.status_code == 404:
            self.fmt = self.__FMT_NOT_REGISTERED
        elif response.status_code == 401 or response.status_code == 403:
            try:
                response_json['text'] = response.text
            except AttributeError:
                response_json['text'] = 'error while pushing'

        super().__init__(snap_name=snap_name, status_code=response.status_code,
                         **response_json)


class StoreReviewError(StoreError):

    __FMT_NEED_MANUAL_REVIEW = (
        'Publishing checks failed.\n'
        'To release this to stable channel please request a review on '
        'the snapcraft list.\n'
        'Use devmode in the edge or beta channels to disable confinement.')

    __FMT_PROCESSING_ERROR = (
        'There has been a problem while analyzing the snap, check the snap '
        'and try to push again.')

    __messages = {
        'need_manual_review': __FMT_NEED_MANUAL_REVIEW,
        'processing_error': __FMT_PROCESSING_ERROR,
    }

    def __init__(self, result):
        self.fmt = self.__messages[result['code']]
        super().__init__()


class StoreReleaseError(StoreError):

    __FMT_NOT_REGISTERED = (
        'Sorry, try `snapcraft register {snap_name}` before trying to '
        'release or choose an existing revision.')

    fmt = 'Received {status_code!r}: {text!r}'

    def __init__(self, snap_name, response):
        try:
            response_json = response.json()
        except (AttributeError, JSONDecodeError):
            response_json = {}

        if response.status_code == 404:
            self.fmt = self.__FMT_NOT_REGISTERED
        elif response.status_code == 401 or response.status_code == 403:
            try:
                response_json['text'] = response.text
            except AttributeError:
                response_json['text'] = 'error while releasing'
        elif 'errors' in response_json:
            self.fmt = '{errors}'

        super().__init__(snap_name=snap_name, status_code=response.status_code,
                         **response_json)
