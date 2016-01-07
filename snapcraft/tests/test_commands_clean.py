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

import os

from snapcraft import (
    common,
    tests,
)
from snapcraft.commands import clean


class CleanCommandTestCase(tests.TestCase):

    yaml_template = """name: clean-test
version: 1.0
summary: test clean
description: if the clean is succesful the state file will be updated
icon: icon.png

parts:
{parts}"""

    yaml_part = """  clean{:d}:
    plugin: nil
    source: ."""

    def make_snapcraft_yaml(self, n=1, create=True):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))
        open('icon.png', 'w').close()

        parts = []
        for i in range(n):
            part_dir = os.path.join(common.get_partsdir(), 'clean{}'.format(i))
            state_file = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_file': state_file,
            })
            if create:
                os.makedirs(part_dir)
                open(state_file, 'w').close()

        if create:
            os.makedirs(common.get_stagedir())
            os.makedirs(common.get_snapdir())

        return parts

    def test_clean_all(self):
        self.make_snapcraft_yaml(n=3)

        clean.main()

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_clean_all_when_all_parts_specified(self):
        self.make_snapcraft_yaml(n=3)

        clean.main(['clean0', 'clean1', 'clean2'])

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_partial_clean(self):
        parts = self.make_snapcraft_yaml(n=3)

        clean.main(['clean0', 'clean2', ])

        for i in [0, 2]:
            self.assertFalse(
                os.path.exists(parts[i]['part_dir']),
                'Expected for {!r} to be wiped'.format(parts[i]['part_dir']))
            self.assertFalse(
                os.path.exists(parts[i]['state_file']),
                'Expected for {!r} to be wiped'.format(parts[i]['state_file']))

        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the clean1 part')
        self.assertTrue(os.path.exists(parts[1]['state_file']),
                        'Expected a state file for the clean1 part')

        self.assertTrue(os.path.exists(common.get_partsdir()))
        self.assertTrue(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_everything_is_clean(self):
        """Don't crash if everything is already clean."""
        self.make_snapcraft_yaml(n=3, create=False)

        clean.main()

    def test_part_to_remove_not_defined_exits_with_error(self):
        self.make_snapcraft_yaml(n=3)

        with self.assertRaises(EnvironmentError) as raised:
            clean.main(['no-clean', ])

        self.assertEqual(
            raised.exception.__str__(),
            "The part named 'no-clean' is not defined in 'snapcraft.yaml'")
