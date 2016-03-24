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

from unittest import mock

from snapcraft import (
    common,
    pluginhandler,
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
            part_name = 'clean{}'.format(i)
            handler = pluginhandler.load_plugin(part_name, 'nil')
            parts.append({
                'part_dir': handler.code.partdir,
            })

            if create:
                handler.makedirs()
                open(os.path.join(
                    handler.code.installdir, part_name), 'w').close()

                handler.mark_done('pull')
                handler.mark_done('build')

                handler.stage()
                handler.strip()

        return parts

    def test_clean_all(self):
        self.make_snapcraft_yaml(n=3)

        clean.main()

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_local_plugin_not_removed(self):
        self.make_snapcraft_yaml(n=3)

        local_plugin = os.path.join(common.get_local_plugindir(), 'foo.py')
        os.makedirs(os.path.dirname(local_plugin))
        open(local_plugin, 'w').close()

        clean.main()

        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))
        self.assertTrue(os.path.exists(common.get_partsdir()))
        self.assertTrue(os.path.isfile(local_plugin))

    def test_clean_all_when_all_parts_specified(self):
        self.make_snapcraft_yaml(n=3)

        clean.main(['clean0', 'clean1', 'clean2'])

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_partial_clean(self):
        parts = self.make_snapcraft_yaml(n=3)

        clean.main(['clean0', 'clean2'])

        for i in [0, 2]:
            self.assertFalse(
                os.path.exists(parts[i]['part_dir']),
                'Expected for {!r} to be wiped'.format(parts[i]['part_dir']))

        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the clean1 part')

        self.assertTrue(os.path.exists(common.get_partsdir()))
        self.assertTrue(os.path.exists(common.get_stagedir()))
        self.assertTrue(os.path.exists(common.get_snapdir()))

        # Now clean it the rest of the way
        clean.main(['clean1'])

        for i in range(0, 3):
            self.assertFalse(
                os.path.exists(parts[i]['part_dir']),
                'Expected for {!r} to be wiped'.format(parts[i]['part_dir']))

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
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

    @mock.patch.object(pluginhandler.PluginHandler, 'clean')
    def test_per_step_cleaning(self, mock_clean):
        self.make_snapcraft_yaml(n=3)

        clean.main(['--step=foo'])

        expected_staged_state = {
            'clean0': pluginhandler.StageState({'clean0'}, set()),
            'clean1': pluginhandler.StageState({'clean1'}, set()),
            'clean2': pluginhandler.StageState({'clean2'}, set()),
        }

        expected_stripped_state = {
            'clean0': pluginhandler.StripState({'clean0'}, set()),
            'clean1': pluginhandler.StripState({'clean1'}, set()),
            'clean2': pluginhandler.StripState({'clean2'}, set()),
        }

        mock_clean.assert_called_with(
            expected_staged_state, expected_stripped_state, 'foo')

    def test_clean_dependent_parts(self):
        yaml = """name: clean-test
version: 1.0
summary: test clean
description: test clean

parts:
  main:
    plugin: nil
    source: .

  dependent:
    plugin: nil
    source: .
    after: [main]"""

        super().make_snapcraft_yaml(yaml)

        part_dirs = {}
        for part in ['main', 'dependent']:
            part_dirs[part] = os.path.join(common.get_partsdir(), part)
            os.makedirs(part_dirs[part])

        os.makedirs(common.get_stagedir())
        os.makedirs(common.get_snapdir())

        # Cleaning only `main`. Since `dependent` depends upon main, we expect
        # that it will be cleaned as well. Otherwise it won't be using the new
        # `main` when it is built.
        clean.main(['main'])

        self.assertFalse(os.path.exists(part_dirs['main']),
                         'Expected part directory for main to be cleaned')
        self.assertFalse(
            os.path.exists(part_dirs['dependent']),
            'Expected part directory for dependent to be cleaned as it '
            'depends upon main')

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_clean_nested_dependent_parts(self):
        yaml = """name: clean-test
version: 1.0
summary: test clean
description: test clean

parts:
  main:
    plugin: nil
    source: .

  dependent:
    plugin: nil
    source: .
    after: [main]

  dependent-dependent:
    plugin: nil
    source: .
    after: [dependent]"""

        super().make_snapcraft_yaml(yaml)

        part_dirs = {}
        for part in ['main', 'dependent', 'dependent-dependent']:
            part_dirs[part] = os.path.join(common.get_partsdir(), part)
            os.makedirs(part_dirs[part])

        os.makedirs(common.get_stagedir())
        os.makedirs(common.get_snapdir())

        # Cleaning only `main`. Since `dependent` depends upon main, we expect
        # that it will be cleaned as well. Otherwise it won't be using the new
        # `main` when it is built.
        clean.main(['main'])

        self.assertFalse(os.path.exists(part_dirs['main']),
                         'Expected part directory for main to be cleaned')
        self.assertFalse(
            os.path.exists(part_dirs['dependent']),
            'Expected part directory for dependent to be cleaned as it '
            'depends upon main')

        self.assertFalse(
            os.path.exists(part_dirs['dependent-dependent']),
            'Expected part directory for dependent-dependent to be cleaned as '
            'it depends upon dependent, which depends upon main')

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))
