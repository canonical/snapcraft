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

from testtools.matchers import Equals

from snapcraft.internal import errors
from snapcraft.internal.meta import _errors as meta_errors
from snapcraft.tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        ('MissingStateCleanError', {
            'exception': errors.MissingStateCleanError,
            'kwargs': {'step': 'test-step'},
            'expected_message': (
                "Failed to clean: "
                "Missing state for 'test-step'. "
                "To clean the project, run `snapcraft clean`."
                )}),
        ('StepOutdatedError dependents', {
            'exception': errors.StepOutdatedError,
            'kwargs': {
                'step': 'test-step',
                'part': 'test-part',
                'dependents': ['test-dependent']
            },
            'expected_message': (
                "Failed to reuse files from previous build: "
                "The 'test-step' step of 'test-part' is out of date:\n"
                "The 'test-step' step for 'test-part' needs to be run again, "
                "but 'test-dependent' depends on it.\n"
                "To continue, clean that part's "
                "'test-step' step, run "
                "`snapcraft clean test-dependent -s test-step`.")}),
        ('StepOutdatedError dirty_properties', {
            'exception': errors.StepOutdatedError,
            'kwargs': {
                'step': 'test-step',
                'part': 'test-part',
                'dirty_properties': ['test-property1', 'test-property2']
            },
            'expected_message': (
                "Failed to reuse files from previous build: "
                "The 'test-step' step of 'test-part' is out of date:\n"
                "The 'test-property1' and 'test-property2' part properties "
                "appear to have changed.\n"
                "To continue, clean that part's "
                "'test-step' step, run "
                "`snapcraft clean test-part -s test-step`.")}),
        ('StepOutdatedError dirty_project_options', {
            'exception': errors.StepOutdatedError,
            'kwargs': {
                'step': 'test-step',
                'part': 'test-part',
                'dirty_project_options': ['test-option']
            },
            'expected_message': (
                "Failed to reuse files from previous build: "
                "The 'test-step' step of 'test-part' is out of date:\n"
                "The 'test-option' project option appears to have changed.\n"
                "To continue, clean that part's "
                "'test-step' step, run "
                "`snapcraft clean test-part -s test-step`.")}),
        ('SnapcraftEnvironmentError', {
            'exception': errors.SnapcraftEnvironmentError,
            'kwargs': {'message': 'test-message'},
            'expected_message': 'test-message'}),
        ('ContainerError', {
            'exception': errors.ContainerError,
            'kwargs': {'message': 'test-message'},
            'expected_message': 'test-message'}),
        ('ContainerConnectionError', {
            'exception': errors.ContainerConnectionError,
            'kwargs': {'message': 'test-message'},
            'expected_message': (
                'test-message\n'
                'Refer to the documentation at '
                'https://linuxcontainers.org/lxd/getting-started-cli.')}),
        ('ContainerRunError string', {
            'exception': errors.ContainerRunError,
            'kwargs': {
                'command': 'test-command',
                'exit_code': '1'
            },
            'expected_message': (
                "The following command failed to run: "
                "'test-command' exited with 1\n")}),
        ('ContainerRunError list', {
            'exception': errors.ContainerRunError,
            'kwargs': {
                'command': ['test-command', 'test-argument'],
                'exit_code': '1'
            },
            'expected_message': (
                "The following command failed to run: "
                "'test-command test-argument' exited with 1\n")}),
        ('ContainerSnapcraftCmdError string', {
            'exception': errors.ContainerSnapcraftCmdError,
            'kwargs': {
                'command': 'test-command',
                'exit_code': '1'
            },
            'expected_message': (
                "Snapcraft command failed in the container: "
                "'test-command' exited with 1\n")}),
        ('ContainerSnapcraftCmdError list', {
            'exception': errors.ContainerSnapcraftCmdError,
            'kwargs': {
                'command': ['test-command', 'test-argument'],
                'exit_code': '1'
            },
            'expected_message': (
                "Snapcraft command failed in the container: "
                "'test-command test-argument' exited with 1\n")}),
        ('SnapdError', {
            'exception': errors.SnapdError,
            'kwargs': {'message': 'test-message'},
            'expected_message': 'test-message'}),
        ('PrimeFileConflictError', {
            'exception': errors.PrimeFileConflictError,
            'kwargs': {'fileset': {'test-file'}},
            'expected_message': (
                "Failed to filter files: "
                "The following files have been excluded by the `stage` "
                "keyword, but included by the `prime` keyword: "
                "{'test-file'}. "
                "Edit the `snapcraft.yaml` to make sure that the files "
                "included in `prime` are also included in `stage`.")}),
        ('InvalidAppCommandError', {
            'exception': errors.InvalidAppCommandError,
            'kwargs': {
                'command': 'test-command',
                'app': 'test-app'
            },
            'expected_message': (
                "Failed to generate snap metadata: "
                "The specified command 'test-command' defined in the app "
                "'test-app' does not exist or is not executable")}),
        ('InvalidContainerRemoteError', {
            'exception': errors.InvalidContainerRemoteError,
            'kwargs': {'remote': 'test-remote'},
            'expected_message': (
                "Failed to use LXD remote: "
                "'test-remote' is not a valid name.\n"
                "Use a LXD remote without colons, spaces and slashes in the "
                "name.\n")}),

        ('', {
            'exception': errors.MissingDesktopFileError,
            'kwargs': {
                'filename': 'test-file',
                'message': 'test-message'
            },
            'expected_message': (
                "Failed to generate desktop file: "
                "MissingDesktopFileError 'test-file': test-message.")}),
        ('SnapcraftPartMissingError', {
            'exception': errors.SnapcraftPartMissingError,
            'kwargs': {'part_name': 'test-part'},
            'expected_message': (
                "Failed to get part information: "
                "Cannot find the definition for part 'test-part'"
                "If it is a remote part, run `snapcraft update` "
                "to refresh the remote parts cache. "
                "If it is a local part, make sure that it is defined in the "
                "`snapcraft.yaml`.")}),
        ('PartNotInCacheError', {
            'exception': errors.PartNotInCacheError,
            'kwargs': {'part_name': 'test-part'},
            'expected_message': (
                "Failed to get remote part information: "
                "Cannot find the part name 'test-part' in the cache. "
                "If it is an existing remote part, run `snapcraft update` "
                "and try again. If it has not been defined, consider going to "
                "https://wiki.ubuntu.com/snapcraft/parts to add it.")}),
        ('PluginError', {
            'exception': errors.PluginError,
            'kwargs': {'message': 'test-message'},
            'expected_message': 'Failed to load plugin: test-message'}),
        ('SnapcraftPartConflictError', {
            'exception': errors.SnapcraftPartConflictError,
            'kwargs': {
                'part_name': 'test-part',
                'other_part_name': 'test-other-part',
                'conflict_files': ('test-file1', 'test-file2')
            },
            'expected_message': (
                "Failed to stage: "
                "Parts 'test-other-part' and 'test-part' have the following "
                "files, but with different contents:\n"
                "    test-file1\n"
                "    test-file2\n"
                "\n"
                "Snapcraft offers some capabilities to solve this by use of "
                "the following keywords:\n"
                "    - `filesets`\n"
                "    - `stage`\n"
                "    - `snap`\n"
                "    - `organize`\n"
                "\n"
                "To learn more about these part keywords, run "
                "`snapcraft help plugins`.")}),
        ('MissingCommandError', {
            'exception': errors.MissingCommandError,
            'kwargs': {
                'required_commands': ['test-command1', 'test-command2']},
            'expected_message': (
                "Failed to run command: "
                "One or more packages are missing, please install:"
                " ['test-command1', 'test-command2']")}),
        ('InvalidWikiEntryError', {
            'exception': errors.InvalidWikiEntryError,
            'kwargs': {'error': 'test-error'},
            'expected_message': "Invalid wiki entry: 'test-error'"}),
        ('PluginOutdatedError', {
            'exception': errors.PluginOutdatedError,
            'kwargs': {'message': 'test-message'},
            'expected_message': 'This plugin is outdated: test-message'}),
        ('RequiredCommandFailure', {
            'exception': errors.RequiredCommandFailure,
            'kwargs': {'command': 'test-command'},
            'expected_message': "'test-command' failed."}),
        ('RequiredCommandNotFound', {
            'exception': errors.RequiredCommandNotFound,
            'kwargs': {'cmd_list': ['test-command', 'test-argument']},
            'expected_message': "'test-command' not found."}),
        ('RequiredPathDoesNotExist', {

            'exception': errors.RequiredPathDoesNotExist,
            'kwargs': {'path': 'test-path'},
            'expected_message': "Required path does not exist: 'test-path'"}),
        ('SnapcraftPathEntryError', {
            'exception': errors.SnapcraftPathEntryError,
            'kwargs': {
                'value': 'test-path',
                'key': 'test-key',
                'app': 'test-app'
            },
            'expected_message': (
                "Failed to generate snap metadata: "
                "The path 'test-path' set for 'test-key' in 'test-app' does "
                "not exist. Make sure that the files are in the `prime` "
                "directory.")}),
        ('InvalidPullPropertiesError', {
            'exception': errors.InvalidPullPropertiesError,
            'kwargs': {
                'plugin_name': 'test-plugin',
                'properties': ['test-property1', 'test-property2']},
            'expected_message': (
                "Failed to load plugin: "
                "Invalid pull properties specified by 'test-plugin' plugin: "
                "['test-property1', 'test-property2']")}),
        ('InvalidBuildPropertiesError', {
            'exception': errors.InvalidBuildPropertiesError,
            'kwargs': {
                'plugin_name': 'test-plugin',
                'properties': ['test-property1', 'test-property2']},
            'expected_message': (
                "Failed to load plugin: "
                "Invalid build properties specified by 'test-plugin' plugin: "
                "['test-property1', 'test-property2']")}),
        ('StagePackageDownloadError', {
            'exception': errors.StagePackageDownloadError,
            'kwargs': {
                'part_name': 'test-part',
                'message': 'test-message'
            },
            'expected_message': (
                "Failed to fetch stage packages: "
                "Error downloading packages for part "
                "'test-part': test-message.")}),
        ('InvalidContainerImageInfoError', {
            'exception': errors.InvalidContainerImageInfoError,
            'kwargs': {'image_info': 'test-image-info'},
            'expected_message': (
                'Error parsing the container image info: test-image-info')}),
        # meta errors.
        ('AdoptedPartMissingError', {
            'exception': meta_errors.AdoptedPartMissingError,
            'kwargs': {'part': 'test-part'},
            'expected_message': (
                "Failed to generate snap metadata: "
                "'adopt-info' refers to a part named 'test-part', but it is "
                "not defined in the 'snapcraft.yaml' file.")}),
        ('AdoptedPartNotParsingInfo', {
            'exception': meta_errors.AdoptedPartNotParsingInfo,
            'kwargs': {'part': 'test-part'},
            'expected_message': (
                "Failed to generate snap metadata: "
                "'adopt-info' refers to part 'test-part', but that part is "
                "lacking the 'parse-info' property.")}),
        ('MissingSnapcraftYamlKeysError', {
            'exception': meta_errors.MissingSnapcraftYamlKeysError,
            'kwargs': {'keys': ['test-key1', 'test-key2']},
            'expected_message': (
                "Failed to generate snap metadata: "
                "Missing required key(s) in snapcraft.yaml: "
                "'test-key1' and 'test-key2'. Either specify the missing "
                "key(s), or use 'adopt-info' to get them from a part.")}),
        ('MissingMetadataFileError', {
            'exception': errors.MissingMetadataFileError,
            'kwargs': {'part_name': 'test-part', 'path': 'test/path'},
            'expected_message': (
                "Failed to generate snap metadata: "
                "Part 'test-part' has a 'parse-info' referring to metadata "
                "file 'test/path', which does not exist.")}),
        ('UnhandledMetadataFileTypeError', {
            'exception': errors.UnhandledMetadataFileTypeError,
            'kwargs': {'path': 'test/path'},
            'expected_message': (
                "Failed to extract metadata from 'test/path': "
                "This type of file is not supported for supplying "
                "metadata.")}),
        ('InvalidExtractorValueError', {
            'exception': errors.InvalidExtractorValueError,
            'kwargs': {'path': 'test/path', 'extractor_name': 'extractor'},
            'expected_message': (
                "Failed to extract metadata from 'test/path': "
                "Extractor 'extractor' didn't return ExtractedMetadata as "
                "expected.")}),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)),
            Equals(self.expected_message))
