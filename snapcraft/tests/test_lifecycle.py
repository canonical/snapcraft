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

import fixtures
from unittest import mock

import snapcraft
from snapcraft.internal import (
    pluginhandler,
    lifecycle,
)
from snapcraft import tests


class ExecutionTestCases(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = snapcraft.ProjectOptions()

    def make_snapcraft_yaml(self, parts, snap_type=''):
        yaml = """name: test
version: 0
summary: test
description: test
{type}

{parts}
"""

        super().make_snapcraft_yaml(yaml.format(parts=parts, type=snap_type))

    def test_exception_when_dependency_is_required(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")

        with self.assertRaises(RuntimeError) as raised:
            lifecycle.execute('pull', self.project_options,
                              part_names=['part2'])

        self.assertEqual(
            raised.exception.__str__(),
            "Requested 'pull' of 'part2' but there are unsatisfied "
            "prerequisites: 'part1'")

    def test_no_exception_when_dependency_is_required_but_already_staged(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")

        def _fake_should_step_run(self, step, force=False):
            return self.name != 'part1'

        with mock.patch.object(pluginhandler.PluginHandler,
                               'should_step_run',
                               _fake_should_step_run):
            lifecycle.execute('pull', self.project_options,
                              part_names=['part2'])

        self.assertEqual(
            'Preparing to pull part2 \n'
            'Pulling part2 \n',
            self.fake_logger.output)

    def test_dependency_recursed_correctly(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")

        snap_info = lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': ''
        }
        self.assertEqual(snap_info, expected_snap_info)

        self.assertEqual(
            'Preparing to pull part1 \n'
            'Pulling part1 \n'
            '\'part2\' has prerequisites that need to be staged: part1\n'
            'Skipping pull part1 (already ran)\n'
            'Preparing to build part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Preparing to pull part2 \n'
            'Pulling part2 \n',
            self.fake_logger.output)

    def test_os_type_returned_by_lifecycle(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""", 'type: os')

        snap_info = lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': 'os'
        }
        self.assertEqual(snap_info, expected_snap_info)

    def test_dirty_strip_restrips_single_part(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Strip it.
        lifecycle.execute('strip', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return self.name == 'part1' and step == 'strip'

        # Should automatically clean and re-strip if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            lifecycle.execute('strip', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Skipping strip part2 (already ran)',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Cleaning snapping area for part1 (out of date)',
                'Stripping part1',
            ],
            part1_output)

    def test_dirty_strip_restrips_multiple_part(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Strip it.
        lifecycle.execute('strip', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'strip'

        # Should automatically clean and re-strip if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            lifecycle.execute('strip', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Cleaning snapping area for part2 (out of date)',
                'Stripping part2',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Cleaning snapping area for part1 (out of date)',
                'Stripping part1',
            ],
            part1_output)

    def test_dirty_stage_restages_single_part(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Stage it.
        lifecycle.execute('stage', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return self.name == 'part1' and step == 'stage'

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            lifecycle.execute('stage', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping cleaning snapping area for part1 (out of date) '
                '(already clean)',
                'Cleaning staging area for part1 (out of date)',
                'Staging part1',
            ],
            part1_output)

    def test_dirty_stage_restages_multiple_parts(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Stage it.
        lifecycle.execute('stage', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'stage'

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            lifecycle.execute('stage', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping cleaning snapping area for part2 (out of date) '
                '(already clean)',
                'Cleaning staging area for part2 (out of date)',
                'Staging part2',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping cleaning snapping area for part1 (out of date) '
                '(already clean)',
                'Cleaning staging area for part1 (out of date)',
                'Staging part1',
            ],
            part1_output)

    def test_dirty_stage_part_with_built_dependent_raises(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        # Stage dependency
        lifecycle.execute('stage', self.project_options, part_names=['part1'])
        # Build dependent
        lifecycle.execute('build', self.project_options, part_names=['part2'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'stage'

        # Should raise a RuntimeError about the fact that stage is dirty but
        # it has dependents that need it.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            with self.assertRaises(RuntimeError) as raised:
                lifecycle.execute('stage', self.project_options,
                                  part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
            ],
            part1_output)

        self.assertEqual(
            "The 'stage' step for 'part1' needs to be run again, but 'part2' "
            "depends upon it. Please clean the build step of 'part2' first.",
            str(raised.exception))

    def test_dirty_stage_part_with_unbuilt_dependent(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        # Stage dependency (dependent is unbuilt)
        lifecycle.execute('stage', self.project_options, part_names=['part1'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'stage'

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            lifecycle.execute('stage', self.project_options,
                              part_names=['part1'])

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Skipping cleaning snapping area for part1 (out of date) '
            '(already clean)\n'
            'Cleaning staging area for part1 (out of date)\n'
            'Staging part1 \n',
            self.fake_logger.output)

    def test_dirty_stage_restrips(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Strip it.
        lifecycle.execute('strip', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'stage'

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            lifecycle.execute('strip', self.project_options)

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Cleaning snapping area for part1 (out of date)\n'
            'Cleaning staging area for part1 (out of date)\n'
            'Staging part1 \n'
            'Stripping part1 \n',
            self.fake_logger.output)

    def test_dirty_build_raises(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Build it.
        lifecycle.execute('build', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'build'

        # Should catch that the part needs to be rebuilt and raise an error.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            with self.assertRaises(RuntimeError) as raised:
                lifecycle.execute('build', self.project_options)

        self.assertEqual(
            'Skipping pull part1 (already ran)\n',
            self.fake_logger.output)

        self.assertEqual(
            "The 'build' step of 'part1' is out of date. Please clean that "
            "part's 'build' step in order to rebuild", str(raised.exception))

    def test_dirty_pull_raises(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Pull it.
        lifecycle.execute('pull', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_is_dirty(self, step):
            return step == 'pull'

        # Should catch that the part needs to be re-pulled and raise an error.
        with mock.patch.object(pluginhandler.PluginHandler, 'is_dirty',
                               _fake_is_dirty):
            with self.assertRaises(RuntimeError) as raised:
                lifecycle.execute('pull', self.project_options)

        self.assertEqual('', self.fake_logger.output)

        self.assertEqual(
            "The 'pull' step of 'part1' is out of date. Please clean that "
            "part's 'pull' step in order to rebuild", str(raised.exception))

    @mock.patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    @mock.patch('snapcraft.repo.install_build_packages')
    def test_pull_is_dirty_if_target_arch_changes(
            self, mock_install_build_packages, mock_enable_cross_compilation):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Pull it with amd64
        lifecycle.execute('pull', snapcraft.ProjectOptions(
            target_deb_arch='amd64'))

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Pull it again with armhf. Should catch that the part needs to be
        # re-pulled due to the change in target architecture and raise an
        # error.
        with self.assertRaises(RuntimeError) as raised:
            lifecycle.execute('pull', snapcraft.ProjectOptions(
                target_deb_arch='armhf'))

        self.assertEqual("Setting target machine to 'armhf'\n",
                         self.fake_logger.output)

        self.assertEqual(
            "The 'pull' step of 'part1' is out of date. Please clean that "
            "part's 'pull' step in order to rebuild", str(raised.exception))


class HumanizeListTestCases(tests.TestCase):

    def test_no_items(self):
        items = []
        output = lifecycle._humanize_list(items)
        self.assertEqual(output, '')

    def test_one_item(self):
        items = ['foo']
        output = lifecycle._humanize_list(items)
        self.assertEqual(output, "'foo'")

    def test_two_items(self):
        items = ['foo', 'bar']
        output = lifecycle._humanize_list(items)
        self.assertEqual(output, "'bar' and 'foo'",
                         "Expected 'bar' before 'foo' due to sorting")

    def test_three_items(self):
        items = ['foo', 'bar', 'baz']
        output = lifecycle._humanize_list(items)
        self.assertEqual(output, "'bar', 'baz', and 'foo'")

    def test_four_items(self):
        items = ['foo', 'bar', 'baz', 'qux']
        output = lifecycle._humanize_list(items)
        self.assertEqual(output, "'bar', 'baz', 'foo', and 'qux'")
