# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

from snapcraft import (
    common,
    tests,
)
from snapcraft.commands import stage


class StageCommandTestCase(tests.TestCase):

    yaml_template = """name: stage-test
version: 1.0
vendor: To Be Removed <vendor@example.com>
summary: test stage
description: if the build is succesful the state file will be updated
icon: icon.png

parts:
{parts}"""

    yaml_part = """  stage{:d}:
    plugin: nil
    source: ."""

    def make_snapcraft_yaml(self, n=1):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))
        open('icon.png', 'w').close()

        parts = []
        for i in range(n):
            part_dir = os.path.join(common.get_partsdir(), 'stage{}'.format(i))
            state_file = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_file': state_file,
            })

        return parts

    def test_stage_invalid_part(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        with self.assertRaises(EnvironmentError) as raised:
            stage.main(['no-stage', ])

        self.assertEqual(
            raised.exception.__str__(),
            "The part named 'no-stage' is not defined in 'snapcraft.yaml'")

    def test_stage_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        stage.main()

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(common.get_partsdir()),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')
        self.assertTrue(os.path.exists(parts[0]['state_file']),
                        'Expected a state file for the build0 part')

        with open(parts[0]['state_file']) as sf:
            state = sf.readlines()
        self.assertEqual(len(state), 1, 'Expected only one line in the state '
                         'file for the build0 part')
        self.assertEqual(state[0], 'stage', "Expected the state file for "
                         "build0 to be 'stage'")

    def test_stage_one_part_only_from_3(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml(n=3)

        stage.main(['stage1', ])

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(common.get_partsdir()),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the stage1 part')
        self.assertTrue(os.path.exists(parts[1]['state_file']),
                        'Expected a state file for the stage1 part')

        with open(parts[1]['state_file']) as sf:
            state = sf.readlines()
        self.assertEqual(len(state), 1, 'Expected only one line in the state '
                         'file for the stage1 part')
        self.assertEqual(state[0], 'stage', "Expected the state file for "
                         " stage1 to be 'stage'")

        for i in [0, 2]:
            self.assertFalse(os.path.exists(parts[i]['part_dir']),
                             'Pulled wrong part')
            self.assertFalse(os.path.exists(parts[i]['state_file']),
                             'Expected for only to be a state file for build1')

    def test_stage_ran_twice_is_a_noop(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        stage.main()

        self.assertEqual(
            'Pulling stage0 \n'
            'Building stage0 \n'
            'Staging stage0 \n',
            fake_logger.output)

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(common.get_partsdir()),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')
        self.assertTrue(os.path.exists(parts[0]['state_file']),
                        'Expected a state file for the build0 part')

        with open(parts[0]['state_file']) as sf:
            state = sf.readlines()
        self.assertEqual(len(state), 1, 'Expected only one line in the state '
                         'file for the build0 part')
        self.assertEqual(state[0], 'stage', "Expected the state file for "
                         "build0 to be 'stage'")

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        stage.main()

        self.assertEqual(
            'Skipping pull stage0  (already ran)\n'
            'Skipping build stage0  (already ran)\n'
            'Skipping stage stage0  (already ran)\n',
            fake_logger.output)
