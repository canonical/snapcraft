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


class PrimeCommandTestCase(tests.TestCase):

    yaml_template = """name: prime-test
version: 1.0
summary: test prime
description: if the prime is succesful the state file will be updated
confinement: strict
grade: stable

parts:
{parts}"""

    yaml_part = """  prime{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))

        parts = []
        for i in range(n):
            part_dir = os.path.join(self.parts_dir, 'prime{}'.format(i))
            state_dir = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_dir': state_dir,
            })

        return parts

    def test_prime_invalid_part(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        raised = self.assertRaises(
            SystemExit,
            main, ['prime', 'no-prime', ])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            "The part named 'no-prime' is not defined in 'snapcraft.yaml'\n")

    def test_prime_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['prime'])

        self.assertTrue(os.path.exists(self.prime_dir),
                        'Expected a prime directory')
        self.assertTrue(
            os.path.exists(
                os.path.join(self.prime_dir, 'meta', 'snap.yaml')),
            'Expected a snap.yaml')
        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'prime')

    def test_prime_one_part_only_from_3(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml(n=3)

        main(['prime', 'prime1'])

        self.assertFalse(
            os.path.exists(
                os.path.join(self.prime_dir, 'meta', 'snap.yaml')),
            'There should not be a snap.yaml')
        self.assertTrue(os.path.exists(self.prime_dir),
                        'Expected a prime directory')
        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the prime1 part')

        self.verify_state('prime1', parts[1]['state_dir'], 'prime')

        for i in [0, 2]:
            self.assertFalse(os.path.exists(parts[i]['part_dir']),
                             'Pulled wrong part')
            self.assertFalse(os.path.exists(parts[i]['state_dir']),
                             'Expected for only to be a state file for build1')

    def test_prime_ran_twice_is_a_noop(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['prime'])

        self.assertEqual(
            'Preparing to pull prime0 \n'
            'Pulling prime0 \n'
            'Preparing to build prime0 \n'
            'Building prime0 \n'
            'Staging prime0 \n'
            'Priming prime0 \n',
            fake_logger.output)

        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'prime')

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main(['prime'])

        self.assertEqual(
            'Skipping pull prime0 (already ran)\n'
            'Skipping build prime0 (already ran)\n'
            'Skipping stage prime0 (already ran)\n'
            'Skipping prime prime0 (already ran)\n',
            fake_logger.output)
