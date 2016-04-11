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
from snapcraft import (
    common,
    pluginhandler,
    tests,
)


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
            snapcraft.lifecycle.execute(
                'pull', self.project_options, part_names=['part2'])

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
            snapcraft.lifecycle.execute(
                'pull', self.project_options, part_names=['part2'])

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

        snap_info = snapcraft.lifecycle.execute('pull', self.project_options)

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

        snap_info = snapcraft.lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': 'os'
        }
        self.assertEqual(snap_info, expected_snap_info)
