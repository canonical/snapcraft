# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

import os
import subprocess

import testscenarios

import examples_tests


class TestSnapcraftExamples(
        testscenarios.WithScenarios, examples_tests.ExampleTestCase):

    scenarios = [
        ('downloader-with-wiki-parts', {
            'example_dir': 'downloader-with-wiki-parts',
            'name': 'downloader',
            'version': '1.0',
            }),
        ('godd', {
            'example_dir': 'godd',
            'name': 'godd',
            'version': '1.0',
            }),
        ('git', {
            'example_dir': 'git',
            'name': 'git',
            'version': '2.8.0',
            }),
        ('py2-project', {
            'example_dir': 'py2-project',
            'name': 'spongeshaker',
            'version': '0',
            }),
        ('py3-project', {
            'example_dir': 'py3-project',
            'name': 'spongeshaker',
            'version': '0',
            }),
        ('ros', {
            'example_dir': 'ros',
            'name': 'ros-example',
            'version': '1.0',
            'external_tests_commands': [
                # check that the hardcoded /usr/bin/python in rosversion
                # is changed to using /usr/bin/env python
                ("sed -n '/env/p;1q' snap/usr/bin/rosversion",
                 b'#!/usr/bin/env python\n',)],
            }),
    ]

    def test_example(self):
        # Build snap will raise an exception in case of error.
        self.build_snap(self.example_dir)
        # Install snap will raise an exception in case of error.
        self.install_snap(self.example_dir, self.name, self.version)

        if getattr(self, 'external_tests_commands', None):
            self._run_external_commands(
                self.external_tests_commands,
                os.path.join('examples', self.example_dir))

    def _run_external_commands(self, external_tests_commands, cwd=None):
        for command, expected_result in external_tests_commands:
            with self.subTest(command):
                output = subprocess.check_output(command, cwd=cwd, shell=True)
                self.assertEqual(output, expected_result)
