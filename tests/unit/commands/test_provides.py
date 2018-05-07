# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import re
import textwrap

from testtools.matchers import Equals, MatchesRegex
import snapcraft.internal.errors

from . import CommandBaseTestCase


class ProvidesCommandTestCase(CommandBaseTestCase):

    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(textwrap.dedent("""\
            name: my-snap-name
            version: '0.1'
            summary: summary
            description: description

            grade: devel
            confinement: devmode

            parts:
              part1:
                plugin: dump
                override-pull: |
                  mkdir dir
                  touch dir/file1

              part2:
                plugin: dump
                override-pull: |
                  mkdir dir
                  touch dir/file2

              part3:
                plugin: nil
                override-stage: |
                  touch file3
            """))

        self.run_command(['prime'])

    def test_provides(self):
        for part_number in ('1', '2'):
            part = 'part{}'.format(part_number)
            file_name = 'file{}'.format(part_number)
            result = self.run_command(
                    ['provides', os.path.join('stage', 'dir', file_name)])
            self.expectThat(result.output, Equals(
                'This path was provided by the following part:\n{}\n'.format(
                    part)))

            result = self.run_command(
                    ['provides', os.path.join(
                        self.prime_dir, 'dir', file_name)])
            self.expectThat(result.output, Equals(
                'This path was provided by the following part:\n{}\n'.format(
                    part)))

        result = self.run_command(
            ['provides', os.path.join('prime', 'dir')])

        # Using MatchesRegex since the order is non-deterministic
        self.expectThat(result.output, MatchesRegex(
            '.*provided by the following parts.*part1', re.DOTALL))
        self.expectThat(result.output, MatchesRegex(
            '.*provided by the following parts.*part2', re.DOTALL))

    def test_provides_file_outside_stage_or_prime(self):
        file_path = os.path.join(
                self.parts_dir, 'part1', 'src', 'dir', 'file1')
        raised = self.assertRaises(
            snapcraft.internal.errors.ProvidesInvalidFilePathError,
            self.run_command, ['provides', file_path])

        self.assertThat(
            raised.path, Equals(file_path))

    def test_provides_untracked_file(self):
        file_path = os.path.join('stage', 'file3')
        raised = self.assertRaises(
            snapcraft.internal.errors.UntrackedFileError,
            self.run_command, ['provides', file_path])

        self.assertThat(
            raised.path, Equals(file_path))

    def test_provides_no_such_file(self):
        file_path = os.path.join('stage', 'foo')
        raised = self.assertRaises(
            snapcraft.internal.errors.NoSuchFileError,
            self.run_command, ['provides', file_path])

        self.assertThat(
            raised.path, Equals(file_path))
