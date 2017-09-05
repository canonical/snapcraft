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
import fileinput
import subprocess

import testscenarios
from testtools.matchers import Contains, Equals

import integration_tests
import snapcraft


def _construct_scenarios():
    main_scenarios = {
        'build-snap-grammar': True,
        'build-snap-grammar-try': True,
        'build-snap-grammar-try-skip': False,
        'build-snap-grammar-try-else': True,
        'build-snap-grammar-on': False,
        'build-snap-grammar-on-else': True,
    }

    # Just some combinations
    channel_scenarios = [
        '', '/stable', '/latest/stable']

    all_scenarios = []
    for project, expected_install in main_scenarios.items():
        for channel in channel_scenarios:
            d = dict(project=project, hello_installed=expected_install,
                     channel=channel)
            scenario = ('{}{}'.format(project, channel), d)
            all_scenarios.append(scenario)

    return all_scenarios


class BuildSnapGrammarTestCase(testscenarios.WithScenarios,
                               integration_tests.TestCase):

    scenarios = _construct_scenarios()

    def setUp(self):
        super().setUp()
        if self._hello_is_installed():
            self.fail(
                'This integration test cannot run if you already have the '
                "'hello' snap installed. Please uninstall it "
                "by running 'sudo snap remove hello'.")

    def tearDown(self):
        super().tearDown()

        # Remove hello. This is safe since the test fails if hello was already
        # installed.
        try:
            subprocess.check_output(
                ['sudo', 'snap', 'remove', 'hello'],
                stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            self.fail("unable to remove 'hello': {}".format(e.output))

    def _hello_is_installed(self):
        return snapcraft.repo.snaps.SnapPackage.is_snap_installed('hello')

    def _add_channel_information_to_hello(self):
        replacement = '- hello{}'.format(self.channel)
        with fileinput.input('snapcraft.yaml', inplace=True) as input_file:
            for line in input_file:
                print(line.replace('- hello', replacement), end='')

    def test_grammar(self):
        self.copy_project_to_cwd(self.project)
        self._add_channel_information_to_hello()

        self.run_snapcraft('pull')

        self.assertThat(self._hello_is_installed(),
                        Equals(self.hello_installed))


class BuildSnapGrammarErrorsTestCase(integration_tests.TestCase):

    def test_on_other_arch_else_fail(self):
        """Test that 'on' fails with an error if it hits an 'else fail'."""

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['pull'], 'build-snap-grammar-fail')

        self.assertThat(exception.output, Contains(
            "Unable to satisfy 'on other-arch', failure forced"))
