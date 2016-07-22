# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import subprocess

from testtools.matchers import (
    DirExists,
    EndsWith,
    Not
)

import integration_tests


class CleanDependentsTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_dir = 'dependencies'
        self.run_snapcraft('prime', self.project_dir)

        # Need to use the state directory here instead of partdir due to
        # bug #1567054.
        self.part_dirs = {
            'p1': os.path.join(self.project_dir, 'parts', 'p1', 'state'),
            'p2': os.path.join(self.project_dir, 'parts', 'p2', 'state'),
            'p3': os.path.join(self.project_dir, 'parts', 'p3', 'state'),
            'p4': os.path.join(self.project_dir, 'parts', 'p4', 'state'),
        }

        self.partsdir = os.path.join(self.project_dir, 'parts')
        self.stagedir = os.path.join(self.project_dir, 'stage')
        self.snapdir = os.path.join(self.project_dir, 'prime')

    def assert_clean(self, parts, common=False):
        for part in parts:
            self.expectThat(
                self.part_dirs[part], Not(DirExists()),
                'Expected part directory for {!r} to be cleaned'.format(part))

        if common:
            self.expectThat(self.partsdir, Not(DirExists()),
                            'Expected parts/ directory to be cleaned')
            self.expectThat(self.stagedir, Not(DirExists()),
                            'Expected stage/ directory to be cleaned')
            self.expectThat(self.snapdir, Not(DirExists()),
                            'Expected snap/ directory to be cleaned')

    def assert_not_clean(self, parts, common=False):
        for part in parts:
            self.expectThat(
                self.part_dirs[part], DirExists(),
                'Expected part directory for {!r} to be uncleaned'.format(
                    part))

        if common:
            self.expectThat(self.partsdir, DirExists(),
                            'Expected parts/ directory to be uncleaned')
            self.expectThat(self.stagedir, DirExists(),
                            'Expected stage/ directory to be uncleaned')
            self.expectThat(self.snapdir, DirExists(),
                            'Expected snap/ directory to be uncleaned')

    def test_clean_nested_dependent(self):
        # Test that p3 (which has dependencies but no dependents) cleans with
        # no extra parameters.
        self.run_snapcraft(['clean', 'p3'], self.project_dir)
        self.assert_clean(['p3'])
        self.assert_not_clean(['p1', 'p2', 'p4'], True)

        # Now run prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_dependent(self):
        # Test that p2 (which has both dependencies and dependents) cleans with
        # its dependents (p3 and p4) also specified.
        self.run_snapcraft(['clean', 'p2', 'p3', 'p4'], self.project_dir)
        self.assert_clean(['p2', 'p3', 'p4'])
        self.assert_not_clean(['p1'], True)

        # Now run prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_main(self):
        # Test that p1 (which has no dependencies but dependents) cleans with
        # its dependents (p2 and, as an extension, p3 and p4) also specified.
        self.run_snapcraft(['clean', 'p1', 'p2', 'p3', 'p4'], self.project_dir)
        self.assert_clean(['p1', 'p2', 'p3', 'p4'], True)

        # Now run prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_dependent_without_nested_dependents_raises(self):
        # Test that p2 (which has both dependencies and dependents) fails to
        # clean if its dependents (p3 and p4) are not also specified
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ['clean', 'p2'],
            self.project_dir)
        self.assertThat(
            exception.output,
            EndsWith(
                "Requested clean of 'p2' but 'p3' and 'p4' depend upon it. Please "
                "add each to the clean command if that's what you intended.\n"))
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_dependent_without_nested_dependent_raises(self):
        # Test that p2 (which has both dependencies and dependents) fails to
        # clean if one of its dependents (p4) is not also specified
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['clean', 'p2', 'p3'], self.project_dir)
        self.assertThat(
            exception.output,
            EndsWith(
                "Requested clean of 'p2' but 'p3' and 'p4' depend upon it. Please "
                "add each to the clean command if that's what you intended.\n"))
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_main_without_any_dependent_raises(self):
        # Test that p1 (which has no dependencies but dependents) fails to
        # clean if none of its dependents are also specified.
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ['clean', 'p1'],
            self.project_dir)
        self.assertThat(
            exception.output,
            EndsWith(
                "Requested clean of 'p1' but 'p2' depends upon it. Please add "
                "each to the clean command if that's what you intended.\n"))
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_main_without_dependent_raises(self):
        # Test that p1 (which has no dependencies but dependents) fails to
        # clean if its dependent (p2) is not also specified
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['clean', 'p1', 'p3', 'p4'], self.project_dir)
        self.assertThat(
            exception.output,
            EndsWith(
                "Requested clean of 'p1' but 'p2' depends upon it. Please add "
                "each to the clean command if that's what you intended.\n"))
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)

    def test_clean_main_without_nested_dependent_raises(self):
        # Test that p1 (which has no dependencies but dependents) fails to
        # clean if its nested dependent (p3, by way of p2) is not also
        # specified
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['clean', 'p1', 'p2'], self.project_dir)
        self.assertThat(
            exception.output,
            EndsWith(
                "Requested clean of 'p2' but 'p3' and 'p4' depend upon it. Please "
                "add each to the clean command if that's what you intended.\n"))
        self.assert_not_clean(['p1', 'p2', 'p3', 'p4'], True)
