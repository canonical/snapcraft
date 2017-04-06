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

    fmt = 'Invalid credentials: {message}.'

    def __init__(self, message):
        super().__init__(message=message)


class StoreError(SnapcraftError):
    """Base class for all storeapi exceptions.

    :cvar fmt: A format string that daughter classes override

    """


class StoreRetryError(StoreError):

    fmt = 'There seems to be a network error: {error}'

    def __init__(self, exception):
        super().__init__(error=str(exception))


class SnapNotFoundError(StoreError):

    __FMT_ARCH_CHANNEL = (
        'Snap {name!r} for {arch!r} cannot be found in the {channel!r} '
        'channel.')
    __FMT_CHANNEL = 'Snap {name!r} was not found in the {channel!r} channel.'
    __FMT_SERIES_ARCH = (
        'Snap {name!r} for {arch!r} was not found in {series!r} series.')
    __FMT_SERIES = 'Snap {name!r} was not found in {series!r} series.'

    fmt = 'Snap {name!r} was not found.'

    def __init__(self, name, channel=None, arch=None, series=None):
        if channel and arch:
            self.fmt = self.__FMT_ARCH_CHANNEL
        elif channel:
            self.fmt = self.__FMT_CHANNEL
        elif series and arch:
            self.fmt = self.__FMT_SERIES_ARCH
        elif series:
            self.fmt = self.__FMT_SERIES

        super().__init__(name=name, channel=channel, arch=arch, series=series)


class SHAMismatchError(StoreError):

    fmt = 'SHA512 checksum for {path} is not {expected_sha}.'

    def __init__(self, path, expected_sha):
        super().__init__(path=path, expected_sha=expected_sha)


class StoreAuthenticationError(StoreError):

    fmt = 'Authentication error: {}.'

    def __init__(self, message):
        super().__init__(message=message)


class StoreTwoFactorAuthenticationRequired(StoreAuthenticationError):

    def __init__(self):
        super().__init__("Two-factor authentication required.")


class StoreMacaroonNeedsRefreshError(StoreError):

    fmt = 'Authentication macaroon needs to be refreshed.'


class DeveloperAgreementSignError(StoreError):

    fmt = (
        'There was an error while signing developer agreement.\n'
        'Reason: {reason!r}\n'
        'Text: {text!r}')

    def __init__(self, response):
        super().__init__(reason=response.reason, text=response.text)


class NeedTermsSignedError(StoreError):

    fmt = (
        'Developer Terms of Service agreement must be signed '
        'before continuing: {message}')

    def __init__(self, message):
        super().__init__(message=message)


class StoreAccountInformationError(StoreError):

    fmt = 'Error fetching account information from store: {error}'

    def __init__(self, response):
        error = '{} {}'.format(response.status_code, response.reason)
        extra = []
        try:
            response_json = response.json()
            if 'error_list' in response_json:
                error = ' '.join(
                    error['message'] for error in response_json['error_list'])
                extra = [
                    error['extra'] for error in response_json[
                        'error_list'] if 'extra' in error]
        except JSONDecodeError:
            pass
        super().__init__(error=error, extra=extra)


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

    __FMT_ALREADY_OWNED = 'You already own the name {snap_name!r}.'

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
        'already_owned': __FMT_ALREADY_OWNED,
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
        'You are not the publisher or allowed to push revisions for this '
        'snap. To become the publisher, run `snapcraft register {snap_name}` '
        'and try to push again.')

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
        'The store was unable to accept this snap.')

    __FMT_PROCESSING_UPLOAD_DELTA_ERROR = (
        'There has been a problem while processing a snap delta.')

    __messages = {
        'need_manual_review': __FMT_NEED_MANUAL_REVIEW,
        'processing_error': __FMT_PROCESSING_ERROR,
        'processing_upload_delta_error': __FMT_PROCESSING_UPLOAD_DELTA_ERROR,
    }

    def __init__(self, result):
        self.fmt = self.__messages[result['code']]
        errors = result.get('errors')
        if errors:
            for error in errors:
                message = error.get('message')
                if message:
                    self.fmt = '{}\n  - {message}'.format(
                        self.fmt, message=message)
        self.code = result['code']
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


class StoreValidationError(StoreError):

    fmt = 'Received error {status_code!r}: {text!r}'

    def __init__(self, snap_id, response, message=None):
        try:
            response_json = response.json()
            response_json['text'] = response.json()['error_list'][0]['message']
        except (AttributeError, JSONDecodeError):
            response_json = {'text': message or response}

        super().__init__(status_code=response.status_code,
                         **response_json)


class StoreSnapBuildError(StoreError):

    fmt = 'Could not assert build: {error}'

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


class StoreSnapRevisionsError(StoreError):

    fmt = (
        'Error fetching revisions of snap id {snap_id!r} for {arch!r} '
        'in {series!r} series: {error}.')

    def __init__(self, response, snap_id, series, arch):
        error = '{} {}'.format(response.status_code, response.reason)
        try:
            response_json = response.json()
            if 'error_list' in response_json:
                error = ' '.join(
                    error['message'] for error in response_json['error_list'])
        except JSONDecodeError:
            pass

        super().__init__(
            snap_id=snap_id, arch=arch or 'any arch',
            series=series or 'any', error=error)


class StoreDeltaApplicationError(Exception):
    pass


class StoreSnapStatusError(StoreSnapRevisionsError):

    fmt = (
        'Error fetching status of snap id {snap_id!r} for {arch!r} '
        'in {series!r} series: {error}.')


class StoreChannelClosingError(StoreError):

    fmt = 'Could not close channel: {error}'

    def __init__(self, response):
        try:
            e = response.json()['error_list'][0]
            error = '{}'.format(e['message'])
        except (JSONDecodeError, KeyError, IndexError):
            error = '{} {}'.format(
                response.status_code, response.reason)

        super().__init__(error=error)
