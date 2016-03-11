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
from snapcraft.commands import pull


class PullCommandTestCase(tests.TestCase):

    yaml_template = """name: pull-test
version: 1.0
summary: test pull
description: if the pull is succesful the state file will be updated
icon: icon.png

parts:
{parts}"""

    yaml_part = """  pull{:d}:
    plugin: nil
    source: ."""

    def make_snapcraft_yaml(self, n=1):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))
        open('icon.png', 'w').close()

        parts = []
        for i in range(n):
            part_dir = os.path.join(common.get_partsdir(), 'pull{}'.format(i))
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

        with self.assertRaises(EnvironmentError) as raised:
            pull.main(['no-pull', ])

        self.assertEqual(
            raised.exception.__str__(),
            "The part named 'no-pull' is not defined in 'snapcraft.yaml'")

    def test_pull_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        pull.main()

        self.assertTrue(os.path.exists(common.get_partsdir()),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the pull0 part')

        self.verify_state('pull0', parts[0]['state_dir'], 'pull')

    def test_pull_one_part_only_from_3(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml(n=3)

        pull.main(['pull1', ])

        self.assertTrue(os.path.exists(common.get_partsdir()),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the pull1 part')

        self.verify_state('pull1', parts[1]['state_dir'], 'pull')

        for i in [0, 2]:
            self.assertFalse(os.path.exists(parts[i]['part_dir']),
                             'Pulled wrong part')
            self.assertFalse(os.path.exists(parts[i]['state_dir']),
                             'Expected for only to be a state file for pull1')
