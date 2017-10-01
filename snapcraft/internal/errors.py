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

from snapcraft import formatting_utils


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

    def get_exit_code(self):
        """Exit code to use if this exception causes Snapcraft to exit."""
        return 2


class MissingStateCleanError(SnapcraftError):
    fmt = (
        "Failed to clean step {step!r}: Missing necessary state. This won't "
        "work until a complete clean has occurred."
    )

    def __init__(self, step):
        super().__init__(step=step)


class StepOutdatedError(SnapcraftError):

    fmt = (
        'The {step!r} step of {part!r} is out of date:\n'
        '{report}'
        'In order to continue, please clean that part\'s {step!r} step '
        'by running:\n'
        'snapcraft clean {parts_names} -s {step}\n'
    )

    def __init__(self, *, step, part,
                 dirty_properties=None, dirty_project_options=None,
                 dependents=None):
        messages = []
        if dirty_properties:
            humanized_properties = formatting_utils.humanize_list(
                dirty_properties, 'and')
            pluralized_connection = formatting_utils.pluralize(
                dirty_properties, 'property appears',
                'properties appear')
            messages.append(
                'The {} part {} to have changed.\n'.format(
                    humanized_properties, pluralized_connection))
        if dirty_project_options:
            humanized_options = formatting_utils.humanize_list(
                dirty_project_options, 'and')
            pluralized_connection = formatting_utils.pluralize(
                dirty_project_options, 'option appears',
                'options appear')
            messages.append(
                'The {} project {} to have changed.\n'.format(
                    humanized_options, pluralized_connection))
        if dependents:
            humanized_dependents = formatting_utils.humanize_list(
                dependents, 'and')
            pluralized_dependents = formatting_utils.pluralize(
                dependents, "depends", "depend")
            messages.append('The {0!r} step for {1!r} needs to be run again, '
                            'but {2} {3} on it.\n'.format(
                                step,
                                part,
                                humanized_dependents,
                                pluralized_dependents))
            parts_names = ['{!s}'.format(d) for d in sorted(dependents)]
        else:
            parts_names = [part]
        super().__init__(step=step, part=part,
                         report=''.join(messages),
                         parts_names=' '.join(parts_names))


class SnapcraftEnvironmentError(SnapcraftError):
    fmt = '{message}'

    def __init__(self, message):
        super().__init__(message=message)


class ContainerError(SnapcraftError):
    fmt = '{message}'

    def __init__(self, message):
        super().__init__(message=message)


class ContainerConnectionError(ContainerError):
    fmt = ('{message}\n'
           'Refer to the documentation at '
           'https://linuxcontainers.org/lxd/getting-started-cli.')


class SnapdError(SnapcraftError):
    fmt = '{message}'

    def __init__(self, message):
        super().__init__(message=message)


class PrimeFileConflictError(SnapcraftError):

    fmt = (
        'The following files have been excluded by the `stage` keyword, '
        'but included by the `prime` keyword: {fileset!r}'
    )


class InvalidAppCommandError(SnapcraftError):

    fmt = (
        'The specified command {command!r} defined in the app {app!r} does '
        'not exist or is not executable'
    )

    def __init__(self, command, app):
        super().__init__(command=command, app=app)


class InvalidContainerRemoteError(SnapcraftError):

    fmt = (
        '{remote!r} is not a valid LXD remote name.\n'
        "Colons, spaces and slashes can't be used.\n"
    )

    def __init__(self, remote):
        super().__init__(remote=remote)


class InvalidDesktopFileError(SnapcraftError):

    fmt = (
        'Invalid desktop file {filename!r}: {message}'
    )

    def __init__(self, filename, message):
        super().__init__(filename=filename, message=message)


class SnapcraftPartMissingError(SnapcraftError):

    fmt = (
        'Cannot find the definition for part {part_name!r}.\n'
        'It may be a remote part, run `snapcraft update` '
        'to refresh the remote parts cache.'
    )


class PartNotInCacheError(SnapcraftError):

    fmt = (
        'Cannot find the part name {part_name!r} in the cache. Please '
        'run `snapcraft update` and try again.\nIf it is indeed missing, '
        'consider going to https://wiki.ubuntu.com/snapcraft/parts '
        'to add it.'
    )


class PluginError(SnapcraftError):

    fmt = 'Issue while loading part: {message}'

    def __init__(self, message):
        super().__init__(message=message)


class PluginNotDefinedError(SnapcraftError):

    fmt = ("Issues while validating snapcraft.yaml: the 'plugin' keyword is "
           "missing for the {part_name} part.")


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


class PluginOutdatedError(SnapcraftError):

    fmt = 'This plugin is outdated: {message}'

    def __init__(self, message):
        super().__init__(message=message)


class RequiredCommandFailure(SnapcraftError):

    fmt = '{command!r} failed.'


class RequiredCommandNotFound(SnapcraftError):

    fmt = '{cmd_list[0]!r} not found.'


class RequiredPathDoesNotExist(SnapcraftError):

    fmt = 'Required path does not exist: {path!r}'


class SnapcraftPathEntryError(SnapcraftError):

    fmt = 'The path {value!r} set for {key!r} in {app!r} does not exist.'


class InvalidPullPropertiesError(SnapcraftError):

    fmt = (
        'Invalid pull properties specified by {plugin_name!r} plugin: '
        '{properties}'
    )

    def __init__(self, plugin_name, properties):
        super().__init__(plugin_name=plugin_name, properties=properties)


class InvalidBuildPropertiesError(SnapcraftError):

    fmt = (
        'Invalid build properties specified by {plugin_name!r} plugin: '
        '{properties}'
    )

    def __init__(self, plugin_name, properties):
        super().__init__(plugin_name=plugin_name, properties=properties)


class StagePackageDownloadError(SnapcraftError):

    fmt = 'Error downloading stage packages for part {part_name!r}: {message}'

    def __init__(self, part_name, message):
        super().__init__(part_name=part_name, message=message)
