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

import logging
import os
import os.path
from unittest import mock

import fixtures

import snapcraft
from snapcraft.main import main
from snapcraft import tests


class PullCommandTestCase(tests.TestCase):

    yaml_template = """name: pull-test
version: 1.0
summary: test pull
description: if the pull is succesful the state file will be updated
confinement: strict
grade: stable

parts:
{parts}"""

    yaml_part = """  pull{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1, yaml_part=None):
        if not yaml_part:
            yaml_part = self.yaml_part

        parts = '\n'.join([yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))

        parts = []
        for i in range(n):
            part_dir = os.path.join(self.parts_dir, 'pull{}'.format(i))
            state_dir = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_dir': state_dir,
            })

        return parts

    def test_pull_invalid_part(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml()

        raised = self.assertRaises(
            SystemExit,
            main, ['pull', 'no-pull', ])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            "The part named 'no-pull' is not defined in 'snapcraft.yaml'\n")

    def test_pull_defaults(self):
        parts = self.make_snapcraft_yaml()

        main(['pull'])

        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the pull0 part')

        self.verify_state('pull0', parts[0]['state_dir'], 'pull')

    def test_pull_one_part_only_from_3(self):
        parts = self.make_snapcraft_yaml(n=3)

        main(['pull', 'pull1'])

        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the pull1 part')

        self.verify_state('pull1', parts[1]['state_dir'], 'pull')

        for i in [0, 2]:
            self.assertFalse(os.path.exists(parts[i]['part_dir']),
                             'Pulled wrong part')
            self.assertFalse(os.path.exists(parts[i]['state_dir']),
                             'Expected for only to be a state file for pull1')

    @mock.patch('snapcraft.repo.Ubuntu.get')
    @mock.patch('snapcraft.repo.Ubuntu.unpack')
    def test_pull_stage_packages_without_geoip(self, mock_get, mock_unpack):
        yaml_part = """  pull{:d}:
        plugin: nil
        stage-packages: ['mir']"""

        self.make_snapcraft_yaml(n=3, yaml_part=yaml_part)

        project_options = mock.Mock(spec=snapcraft.ProjectOptions)

        project_options = main(['pull', 'pull1'])

        self.assertFalse(project_options.use_geoip)

    @mock.patch('snapcraft.repo.Ubuntu.get')
    @mock.patch('snapcraft.repo.Ubuntu.unpack')
    def test_pull_stage_packages_with_geoip(self, mock_get, mock_unpack):
        yaml_part = """  pull{:d}:
        plugin: nil
        stage-packages: ['mir']"""

        self.make_snapcraft_yaml(n=3, yaml_part=yaml_part)

        project_options = main(['pull', 'pull1', '--enable-geoip'])

        self.assertTrue(project_options.use_geoip)
