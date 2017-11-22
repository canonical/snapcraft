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
from snapcraft.tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
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
                "To continue, please clean that part's "
                "'test-step' step by running:\n"
                "snapcraft clean test-dependent -s test-step\n")}),
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
                "To continue, please clean that part's "
                "'test-step' step by running:\n"
                "snapcraft clean test-part -s test-step\n")}),
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
                "To continue, please clean that part's "
                "'test-step' step by running:\n"
                "snapcraft clean test-part -s test-step\n")}),
        ('PrimeFileConflictError', {
            'exception': errors.PrimeFileConflictError,
            'kwargs': {'fileset': {'test-file': 'test-value'}},
            'expected_message': (
                "Failed to filter files: "
                "The following files have been excluded by the `stage` "
                "keyword, but included by the `prime` keyword: "
                "{'test-file': 'test-value'}. "
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
        ('SnapcraftPartMissingError', {
            'exception': errors.SnapcraftPartMissingError,
            'kwargs': {'part_name': 'test-part'},
            'expected_message': (
                "Failed to get part information: "
                "Cannot find the definition for part 'test-part'. "
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
        (' StagePackageDownloadError', {
            'exception': errors. StagePackageDownloadError,
            'kwargs': {
                'part_name': 'test-part',
                'message': 'test-message'
            },
            'expected_message': (
                "Failed to fetch stage packages: "
                "Error downloading packages for part "
                "'test-part': test-message.")})
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)),
            Equals(self.expected_message))
