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

import contextlib

import snapcraft.internal.errors
from snapcraft import formatting_utils

# dict of jsonschema validator -> cause pairs. Wish jsonschema just gave us
# better messages.
_VALIDATION_ERROR_CAUSES = {
    'maxLength': 'maximum length is {validator_value}',
    'minLength': 'minimum length is {validator_value}',
}


class ProjectLoaderError(snapcraft.internal.errors.SnapcraftError):

    fmt = ''


class InvalidEpochError(ProjectLoaderError):

    fmt = 'epochs are positive integers followed by an optional asterisk'


class DuplicateAliasError(ProjectLoaderError):

    fmt = 'Multiple parts have the same alias defined: {aliases!r}'

    def __str__(self):
        if isinstance(self.aliases, (list, set)):
            self.aliases = ','.join(self.aliases)

        return super().__str__()


class MissingSnapcraftYamlError(ProjectLoaderError):

    fmt = ('Could not find {snapcraft_yaml}. Are you sure you are in the '
           'right directory?\n'
           'To start a new project, use `snapcraft init`')


class YamlValidationError(ProjectLoaderError):

    fmt = 'Issues while validating {snapcraft_yaml}: {message}'

    @classmethod
    def from_validation_error(cls, error):
        """Take a jsonschema.ValidationError and create a SnapcraftSchemaError.

        The validation errors coming from jsonschema are a nightmare. This
        class tries to make them a bit more understandable.
        """

        messages = []

        # error.validator_value may contain a custom validation error message.
        # If so, use it instead of the garbage message jsonschema gives us.
        with contextlib.suppress(TypeError, KeyError):
            messages.append(
                error.validator_value['validation-failure'].format(error))

        if not messages:
            messages.append(error.message)

        path = []
        while error.absolute_path:
            element = error.absolute_path.popleft()
            # assume numbers are indices and use 'xxx[123]' notation.
            if isinstance(element, int):
                path[-1] = '{}[{}]'.format(path[-1], element)
            else:
                path.append(str(element))
        if path:
            messages.insert(0, "The '{}' property does not match the "
                               "required schema:".format('/'.join(path)))
        cause = error.cause or _determine_cause(error)
        if cause:
            messages.append('({})'.format(cause))

        return cls(' '.join(messages))

    def __init__(self, message, snapcraft_yaml='snapcraft.yaml'):
        super().__init__(message=message, snapcraft_yaml=snapcraft_yaml)


class SnapcraftLogicError(ProjectLoaderError):

    fmt = 'Issue detected while analyzing snapcraft.yaml: {message}'

    def __init__(self, message):
        super().__init__(message=message)


def _determine_cause(error):
    """Attempt to determine a cause from validation error.

    :return: A string representing the cause of the error (it may be empty if
             no cause can be determined).
    :rtype: str
    """

    message = _VALIDATION_ERROR_CAUSES.get(error.validator, '').format(
        validator_value=error.validator_value)

    if not message and error.validator == 'anyOf':
        message = _interpret_anyOf(error)

    return message


def _interpret_anyOf(error):
    """Interpret a validation error caused by the anyOf validator.

    Returns:
        A string containing a (hopefully) helpful validation error message. It
        may be empty.
    """

    usages = []
    try:
        for validator in error.validator_value:
            usages.append(validator['usage'])
    except (TypeError, KeyError):
        return ''

    return 'must be one of {}'.format(formatting_utils.humanize_list(
        usages, 'or'))
