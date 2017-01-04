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
import shutil

from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft.internal import (
    pluginhandler,
    project_loader,
    states,
)
from snapcraft import tests


class CleanCommandTestCase(tests.TestCase):

    yaml_template = """name: clean-test
version: 1.0
summary: test clean
description: if the clean is succesful the state file will be updated
icon: icon.png
confinement: strict
grade: stable

parts:
{parts}"""

    yaml_part = """  clean{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1, create=True):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))
        open('icon.png', 'w').close()

        parts = []
        for i in range(n):
            part_name = 'clean{}'.format(i)
            handler = pluginhandler.load_plugin(
                part_name, plugin_name='nil',
                part_properties={'plugin': 'nil'},
                part_schema=project_loader.Validator().part_schema)
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
                handler.prime()

        return parts

    def test_clean_all(self):
        self.make_snapcraft_yaml(n=3)

        main(['clean'])

        self.assertFalse(os.path.exists(self.parts_dir))
        self.assertFalse(os.path.exists(self.stage_dir))
        self.assertFalse(os.path.exists(self.snap_dir))

    def test_local_plugin_not_removed(self):
        self.make_snapcraft_yaml(n=3)

        local_plugin = os.path.join(self.local_plugins_dir, 'foo.py')
        os.makedirs(os.path.dirname(local_plugin))
        open(local_plugin, 'w').close()

        main(['clean'])

        self.assertFalse(os.path.exists(self.stage_dir))
        self.assertFalse(os.path.exists(self.snap_dir))
        self.assertTrue(os.path.exists(self.parts_dir))
        self.assertTrue(os.path.isfile(local_plugin))

    def test_clean_all_when_all_parts_specified(self):
        self.make_snapcraft_yaml(n=3)

        main(['clean', 'clean0', 'clean1', 'clean2'])

        self.assertFalse(os.path.exists(self.parts_dir))
        self.assertFalse(os.path.exists(self.stage_dir))
        self.assertFalse(os.path.exists(self.snap_dir))

    def test_partial_clean(self):
        parts = self.make_snapcraft_yaml(n=3)

        main(['clean', 'clean0', 'clean2'])

        for i in [0, 2]:
            self.assertFalse(
                os.path.exists(parts[i]['part_dir']),
                'Expected for {!r} to be wiped'.format(parts[i]['part_dir']))

        self.assertTrue(os.path.exists(parts[1]['part_dir']),
                        'Expected a part directory for the clean1 part')

        self.assertTrue(os.path.exists(self.parts_dir))
        self.assertTrue(os.path.exists(self.stage_dir))
        self.assertTrue(os.path.exists(self.snap_dir))

        # Now clean it the rest of the way
        main(['clean', 'clean1'])

        for i in range(0, 3):
            self.assertFalse(
                os.path.exists(parts[i]['part_dir']),
                'Expected for {!r} to be wiped'.format(parts[i]['part_dir']))

        self.assertFalse(os.path.exists(self.parts_dir))
        self.assertFalse(os.path.exists(self.stage_dir))
        self.assertFalse(os.path.exists(self.snap_dir))

    def test_everything_is_clean(self):
        """Don't crash if everything is already clean."""
        self.make_snapcraft_yaml(n=3, create=False)

        main(['clean'])

    def test_part_to_remove_not_defined_exits_with_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml(n=3)

        raised = self.assertRaises(
            SystemExit,
            main, ['clean', 'no-clean'])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            "The part named 'no-clean' is not defined in 'snapcraft.yaml'\n")

    @mock.patch.object(pluginhandler.PluginHandler, 'clean')
    def test_per_step_cleaning(self, mock_clean):
        self.make_snapcraft_yaml(n=3)

        main(['clean', '--step=foo'])

        expected_staged_state = {
            'clean0': states.StageState({'clean0'}, set()),
            'clean1': states.StageState({'clean1'}, set()),
            'clean2': states.StageState({'clean2'}, set()),
        }

        expected_primed_state = {
            'clean0': states.PrimeState({'clean0'}, set()),
            'clean1': states.PrimeState({'clean1'}, set()),
            'clean2': states.PrimeState({'clean2'}, set()),
        }

        mock_clean.assert_called_with(
            expected_staged_state, expected_primed_state, 'foo')

    @mock.patch.object(pluginhandler.PluginHandler, 'clean')
    def test_cleaning_with_strip_does_prime_and_warns(self, mock_clean):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml(n=3)

        main(['clean', '--step=strip'])

        expected_staged_state = {
            'clean0': states.StageState({'clean0'}, set()),
            'clean1': states.StageState({'clean1'}, set()),
            'clean2': states.StageState({'clean2'}, set()),
        }

        expected_primed_state = {
            'clean0': states.PrimeState({'clean0'}, set()),
            'clean1': states.PrimeState({'clean1'}, set()),
            'clean2': states.PrimeState({'clean2'}, set()),
        }

        self.assertEqual('DEPRECATED: Use `prime` instead of `strip` as '
                         'the step to clean\n', fake_logger.output)
        mock_clean.assert_called_with(
            expected_staged_state, expected_primed_state, 'prime')


class CleanCommandReverseDependenciesTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml("""name: clean-test
version: 1.0
summary: test clean
description: test clean
confinement: strict
grade: stable

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]

  nested-dependent:
    plugin: nil
    after: [dependent]""")

        self.part_dirs = {}
        for part in ['main', 'dependent', 'nested-dependent']:
            self.part_dirs[part] = os.path.join(self.parts_dir, part)
            os.makedirs(os.path.join(self.part_dirs[part], 'state'))
            open(os.path.join(self.part_dirs[part], 'state', 'pull'),
                 'w').close()

        os.makedirs(self.stage_dir)
        os.makedirs(self.snap_dir)

    def assert_clean(self, parts):
        for part in parts:
            self.assertFalse(
                os.path.exists(self.part_dirs[part]),
                'Expected part directory for {!r} to be cleaned'.format(part))

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
        raised = self.assertRaises(
            RuntimeError,
            main, ['-d', 'clean', 'dependent'])

        self.assertEqual(
            str(raised),
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
        raised = self.assertRaises(
            RuntimeError,
            main, ['-d', 'clean', 'main'])

        self.assertEqual(
            str(raised),
            "Requested clean of 'main' but 'dependent' depends upon it. "
            "Please add each to the clean command if that's what you "
            "intended.")

    def test_clean_part_unspecified_uncleaned_nested_dependent_raises(self):
        # Not specifying nested-dependent here should result in clean raising
        # an exception, saying that it has dependents.  Note the use of '-d',
        # so we get a RuntimeError instead of SystemExit.
        raised = self.assertRaises(
            RuntimeError,
            main, ['-d', 'clean', 'main', 'dependent'])

        self.assertEqual(
            str(raised),
            "Requested clean of 'dependent' but 'nested-dependent' depends "
            "upon it. Please add each to the clean command if that's what you "
            "intended.")
