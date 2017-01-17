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

import fixtures

from snapcraft.main import main
from snapcraft import tests


class StageCommandTestCase(tests.TestCase):

    yaml_template = """name: stage-test
version: 1.0
summary: test stage
description: if the build is succesful the state file will be updated
confinement: strict
grade: stable

parts:
{parts}"""

    yaml_part = """  stage{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))

        parts = []
        for i in range(n):
            part_dir = os.path.join(self.parts_dir, 'stage{}'.format(i))
            state_dir = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_dir': state_dir,
            })

        return parts

    def test_stage_invalid_part(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        raised = self.assertRaises(
            SystemExit,
            main, ['stage', 'no-stage', ])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            "The part named 'no-stage' is not defined in 'snapcraft.yaml'\n")

    def test_stage_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['stage'])

        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'stage')

    def test_stage_one_part_only_from_3(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml(n=3)

        main(['stage', 'stage1'])

        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the stage1 part')

        self.verify_state('stage1', parts[1]['state_dir'], 'stage')

        for i in [0, 2]:
            self.assertFalse(os.path.exists(parts[i]['part_dir']),
                             'Pulled wrong part')
            self.assertFalse(os.path.exists(parts[i]['state_dir']),
                             'Expected for only to be a state file for build1')

    def test_stage_ran_twice_is_a_noop(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['stage'])

        self.assertEqual(
            'Preparing to pull stage0 \n'
            'Pulling stage0 \n'
            'Preparing to build stage0 \n'
            'Building stage0 \n'
            'Staging stage0 \n',
            fake_logger.output)

        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'stage')

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main(['stage'])

        self.assertEqual(
            'Skipping pull stage0 (already ran)\n'
            'Skipping build stage0 (already ran)\n'
            'Skipping stage stage0 (already ran)\n',
            fake_logger.output)
