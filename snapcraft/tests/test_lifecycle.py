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

from snapcraft import (
    common,
    lifecycle,
    tests,
)


class ExecutionTestCases(tests.TestCase):

    def test_exception_when_dependency_is_required(self):
        self.make_snapcraft_yaml("""name: after
version: 0
summary: test stage
description: if the build is succesful the state file will be updated
icon: icon.png

parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")
        open('icon.png', 'w').close()

        with self.assertRaises(RuntimeError) as raised:
            lifecycle.execute('pull', part_names=['part2'])

        self.assertEqual(
            raised.exception.__str__(),
            "Requested 'pull' of 'part2' but there are unsatisfied "
            "prerequisites: 'part1'")

    def test_dependency_recursed_correctly(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: after
version: 0
summary: test stage
description: if the build is succesful the state file will be updated
icon: icon.png

parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")
        open('icon.png', 'w').close()

        snap_info = lifecycle.execute('pull')

        expected_snap_info = {
            'name': 'after',
            'version': 0,
            'arch': [common.get_arch()],
            'type': ''
        }
        self.assertEqual(snap_info, expected_snap_info)

        self.assertEqual(
            'Pulling part1 \n'
            '\'part2\' has prerequisites that need to be staged: part1\n'
            'Skipping pull part1  (already ran)\n'
            'Building part1 \n'
            'Staging part1 \n'
            'Pulling part2 \n',
            fake_logger.output)

    def test_os_type_returned_by_lifecycle(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: after
version: 0
summary: test stage
description: check and see if we return type 'os'
type: os

parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")
        open('icon.png', 'w').close()

        snap_info = lifecycle.execute('pull')

        expected_snap_info = {
            'name': 'after',
            'version': 0,
            'arch': [common.get_arch()],
            'type': 'os'
        }
        self.assertEqual(snap_info, expected_snap_info)
