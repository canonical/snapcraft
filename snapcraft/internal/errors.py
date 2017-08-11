# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from snapcraft import formatting_utils

# dict of jsonschema validator -> cause pairs. Wish jsonschema just gave us
# better messages.
_VALIDATION_ERROR_CAUSES = {
    'maxLength': 'maximum length is {validator_value}',
    'minLength': 'minimum length is {validator_value}',
}


class SnapcraftError(Exception):
    """Base class for all snapcraft exceptions.

    :cvar fmt: A format string that daughter classes override

    """
    fmt = 'Daughter classes should redefine this'

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __str__(self):
        return self.fmt.format([], **self.__dict__)


class MissingState(Exception):
    pass


class SnapcraftEnvironmentError(Exception):
    pass


class PrimeFileConflictError(SnapcraftError):

    fmt = (
        'The following files have been excluded by the `stage` keyword, '
        'but included by the `prime` keyword: {fileset!r}'
    )


class DuplicateAliasError(SnapcraftError):

    fmt = 'Multiple parts have the same alias defined: {aliases!r}'

    def __str__(self):
        if isinstance(self.aliases, (list, set)):
            self.aliases = ','.join(self.aliases)

        return super().__str__()


class SnapcraftPartMissingError(SnapcraftError):

    fmt = (
        'Cannot find the definition for part {part_name!r}.\n'
        'It may be a remote part, run `snapcraft update` '
        'to refresh the remote parts cache.'
    )


class SnapcraftLogicError(SnapcraftError):

    fmt = 'Issue detected while analyzing snapcraft.yaml: {message}'

    def __init__(self, message):
        super().__init__(message=message)


class PluginError(SnapcraftError):

    fmt = 'Issue while loading part: {message}'

    def __init__(self, message):
        super().__init__(message=message)


class PluginNotDefinedError(SnapcraftError):

    fmt = ("Issues while validating snapcraft.yaml: the 'plugin' keyword is "
           "missing for the {part_name} part.")


class SnapcraftYamlFileError(SnapcraftError):

    fmt = ('Could not find {snapcraft_yaml}. Are you sure you are in the '
           'right directory?\n'
           'To start a new project, use `snapcraft init`')


class SnapcraftPartConflictError(SnapcraftError):

    fmt = (
        'Parts {other_part_name!r} and {part_name!r} have the following file '
        'paths in common which have different contents:\n'
        '{file_paths}\n\n'
        'Snapcraft offers some capabilities to solve this by use of the '
        'following keywords:\n'
        '    - `filesets`\n'
        '    - `stage`\n'
        '    - `snap`\n'
        '    - `organize`\n\n'
        'Learn more about these part keywords by running '
        '`snapcraft help plugins`'
    )

    def __init__(self, *, part_name, other_part_name, conflict_files):
        spaced_conflict_files = ('    {}'.format(i) for i in conflict_files)
        super().__init__(part_name=part_name,
                         other_part_name=other_part_name,
                         file_paths='\n'.join(sorted(spaced_conflict_files)))


class MissingCommandError(SnapcraftError):

    fmt = (
        'One or more required commands are missing, please install:'
        ' {required_commands!r}'
    )

    def __init__(self, required_commands):
        super().__init__(required_commands=required_commands)


class InvalidWikiEntryError(SnapcraftError):

    fmt = 'Invalid wiki entry: {error!r}'

    def __init__(self, error=None):
        super().__init__(error=error)


class MissingGadgetError(SnapcraftError):

    fmt = (
        'When creating gadget snaps you are required to provide a gadget.yaml file\n'  # noqa
        'in the root of your snapcraft project\n\n'
        'Read more about gadget snaps and the gadget.yaml on:\n'
        'https://github.com/snapcore/snapd/wiki/Gadget-snap')


class RequiredCommandFailure(SnapcraftError):

    fmt = '{command!r} failed.'


class RequiredCommandNotFound(SnapcraftError):

    fmt = '{cmd_list[0]!r} not found.'


class RequiredPathDoesNotExist(SnapcraftError):

    fmt = 'Required path does not exist: {path!r}'


class SnapcraftPathEntryError(SnapcraftError):

    fmt = 'The path {value!r} set for {key!r} in {app!r} does not exist.'


class SnapcraftSchemaError(SnapcraftError):

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
