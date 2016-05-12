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

from snapcraft.main import main
from snapcraft import tests


class StripCommandTestCase(tests.TestCase):

    yaml_template = """name: strip-test
version: 1.0
summary: test strip
description: if the strip is succesful the state file will be updated
confinement: enabled

parts:
{parts}"""

    yaml_part = """  strip{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))

        parts = []
        for i in range(n):
            part_dir = os.path.join(self.parts_dir, 'strip{}'.format(i))
            state_dir = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_dir': state_dir,
            })

        return parts

    def test_strip_invalid_part(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        with self.assertRaises(SystemExit) as raised:
            main(['strip', 'no-strip', ])

        self.assertEqual(
            "The part named 'no-strip' is not defined in 'snapcraft.yaml'",
            str(raised.exception))

    def test_strip_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['strip'])

        self.assertTrue(os.path.exists(self.snap_dir),
                        'Expected a snap directory')
        self.assertTrue(
            os.path.exists(
                os.path.join(self.snap_dir, 'meta', 'snap.yaml')),
            'Expected a snap.yaml')
        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'strip')

    def test_strip_one_part_only_from_3(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml(n=3)

        main(['strip', 'strip1'])

        self.assertFalse(
            os.path.exists(
                os.path.join(self.snap_dir, 'meta', 'snap.yaml')),
            'There should not be a snap.yaml')
        self.assertTrue(os.path.exists(self.snap_dir),
                        'Expected a snap directory')
        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the strip1 part')

        self.verify_state('strip1', parts[1]['state_dir'], 'strip')

        for i in [0, 2]:
            self.assertFalse(os.path.exists(parts[i]['part_dir']),
                             'Pulled wrong part')
            self.assertFalse(os.path.exists(parts[i]['state_dir']),
                             'Expected for only to be a state file for build1')

    def test_strip_ran_twice_is_a_noop(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['strip'])

        self.assertEqual(
            'Preparing to pull strip0 \n'
            'Pulling strip0 \n'
            'Preparing to build strip0 \n'
            'Building strip0 \n'
            'Staging strip0 \n'
            'Stripping strip0 \n',
            fake_logger.output)

        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'strip')

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main(['strip'])

        self.assertEqual(
            'Skipping pull strip0 (already ran)\n'
            'Skipping build strip0 (already ran)\n'
            'Skipping stage strip0 (already ran)\n'
            'Skipping strip strip0 (already ran)\n',
            fake_logger.output)
