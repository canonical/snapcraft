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
import shutil

from unittest import mock

from snapcraft.main import main
from snapcraft import (
    common,
    pluginhandler,
    tests,
)


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

        main(['clean'])

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_local_plugin_not_removed(self):
        self.make_snapcraft_yaml(n=3)

        local_plugin = os.path.join(common.get_local_plugindir(), 'foo.py')
        os.makedirs(os.path.dirname(local_plugin))
        open(local_plugin, 'w').close()

        main(['clean'])

        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))
        self.assertTrue(os.path.exists(common.get_partsdir()))
        self.assertTrue(os.path.isfile(local_plugin))

    def test_clean_all_when_all_parts_specified(self):
        self.make_snapcraft_yaml(n=3)

        main(['clean', 'clean0', 'clean1', 'clean2'])

        self.assertFalse(os.path.exists(common.get_partsdir()))
        self.assertFalse(os.path.exists(common.get_stagedir()))
        self.assertFalse(os.path.exists(common.get_snapdir()))

    def test_partial_clean(self):
        parts = self.make_snapcraft_yaml(n=3)

        main(['clean', 'clean0', 'clean2'])

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
        main(['clean', 'clean1'])

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

        main(['clean'])

    def test_part_to_remove_not_defined_exits_with_error(self):
        self.make_snapcraft_yaml(n=3)

        with self.assertRaises(SystemExit) as raised:
            main(['clean', 'no-clean'])

        self.assertEqual(
            raised.exception.__str__(),
            "The part named 'no-clean' is not defined in 'snapcraft.yaml'")

    @mock.patch.object(pluginhandler.PluginHandler, 'clean')
    def test_per_step_cleaning(self, mock_clean):
        self.make_snapcraft_yaml(n=3)

        main(['clean', '--step=foo'])

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


class CleanCommandReverseDependenciesTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml("""name: clean-test
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

  nested-dependent:
    plugin: nil
    source: .
    after: [dependent]""")

        self.part_dirs = {}
        for part in ['main', 'dependent', 'nested-dependent']:
            self.part_dirs[part] = os.path.join(common.get_partsdir(), part)
            os.makedirs(os.path.join(self.part_dirs[part], 'state'))
            open(os.path.join(self.part_dirs[part], 'state', 'pull'),
                 'w').close()

        os.makedirs(common.get_stagedir())
        os.makedirs(common.get_snapdir())

    def assert_clean(self, parts, common=False):
        for part in parts:
            self.assertFalse(
                os.path.exists(self.part_dirs[part]),
                'Expected part directory for {!r} to be cleaned'.format(part))

        if common:
            self.assertFalse(os.path.exists(common.get_partsdir()),
                             'Expected parts/ directory to be removed')
            self.assertFalse(os.path.exists(common.get_stagedir()),
                             'Expected stage/ directory to be removed')
            self.assertFalse(os.path.exists(common.get_snapdir()),
                             'Expected snap/ directory to be removed')

    def test_clean_dependent_parts(self):
        main(['clean', 'dependent', 'nested-dependent'])

        self.assert_clean(['dependent', 'nested-dependent'])
        self.assertTrue(
            os.path.exists(self.part_dirs['main']),
            'Expected part directory for main to be untouched by the clean')

    def test_clean_part_with_clean_dependent(self):
        main(['clean', 'nested-dependent'])
        self.assert_clean(['nested-dependent'])

        # Not specifying nested-dependent here should be okay since it's
        # already clean.
        main(['clean', 'dependent'])
        self.assert_clean(['dependent', 'nested-dependent'])

    def test_clean_part_unspecified_uncleaned_dependent_raises(self):
        # Not specifying nested-dependent here should result in clean raising
        # an exception, saying that it has dependents. Note the use of '-d',
        # so we get a RuntimeError instead of SystemExit.
        with self.assertRaises(RuntimeError) as raised:
            main(['-d', 'clean', 'dependent'])

        self.assertEqual(
            str(raised.exception),
            "Requested clean of 'dependent' but 'nested-dependent' depends "
            "upon it. Please add each to the clean command if that's what you "
            "intended.")

    def test_clean_nested_dependent_parts(self):
        main(['clean', 'main', 'dependent', 'nested-dependent'])
        self.assert_clean(['main', 'dependent', 'nested-dependent'])

    def test_clean_part_with_clean_dependent_uncleaned_nested_dependent(self):
        shutil.rmtree(self.part_dirs['dependent'])
        self.assert_clean(['dependent'])

        # Not specifying dependent here should be okay since it's already
        # clean.
        main(['clean', 'main', 'nested-dependent'])
        self.assert_clean(['main', 'dependent', 'nested-dependent'])

    def test_clean_part_with_clean_nested_dependent(self):
        shutil.rmtree(self.part_dirs['nested-dependent'])
        self.assert_clean(['nested-dependent'])

        # Not specifying nested-dependent here should be okay since it's
        # already clean.
        main(['clean', 'main', 'dependent'])
        self.assert_clean(['main', 'dependent', 'nested-dependent'])

    def test_clean_part_unspecified_uncleaned_dependent_with_nest_raises(self):
        # Not specifying dependent here should result in clean raising
        # an exception, saying that it has dependents.  Note the use of '-d',
        # so we get a RuntimeError instead of SystemExit.
        with self.assertRaises(RuntimeError) as raised:
            main(['-d', 'clean', 'main'])

        self.assertEqual(
            str(raised.exception),
            "Requested clean of 'main' but 'dependent' depends upon it. "
            "Please add each to the clean command if that's what you "
            "intended.")

    def test_clean_part_unspecified_uncleaned_nested_dependent_raises(self):
        # Not specifying nested-dependent here should result in clean raising
        # an exception, saying that it has dependents.  Note the use of '-d',
        # so we get a RuntimeError instead of SystemExit.
        with self.assertRaises(RuntimeError) as raised:
            main(['-d', 'clean', 'main', 'dependent'])

        self.assertEqual(
            str(raised.exception),
            "Requested clean of 'dependent' but 'nested-dependent' depends "
            "upon it. Please add each to the clean command if that's what you "
            "intended.")
