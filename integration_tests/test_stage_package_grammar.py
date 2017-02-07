# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import integration_tests
from testtools.matchers import (
    Contains,
    FileExists,
    Not
)


class StagePackageGrammarTestCase(integration_tests.TestCase):

    def test_simple(self):
        """Test that 'simple' fetches stage package."""

        self.run_snapcraft(['prime', 'simple'], 'stage-package-grammar')

        self.assertThat(
            os.path.join('prime', 'usr', 'bin', 'hello'),
            FileExists())

    def test_try(self):
        """Test that 'try' fetches stage package."""

        self.run_snapcraft(['prime', 'try'], 'stage-package-grammar')

        self.assertThat(
            os.path.join('prime', 'usr', 'bin', 'hello'),
            FileExists())

    def test_try_skipped(self):
        """Test that 'try-skipped' fetches nothing."""

        self.run_snapcraft(['prime', 'try-skipped'], 'stage-package-grammar')

        self.assertThat(
            os.path.join('prime', 'usr', 'bin', 'hello'),
            Not(FileExists()))

    def test_try_else(self):
        """Test that 'try-else' fetches stage package."""

        self.run_snapcraft(['prime', 'try-else'], 'stage-package-grammar')

        self.assertThat(
            os.path.join('prime', 'usr', 'bin', 'hello'),
            FileExists())

    def test_on_other_arch(self):
        """Test that 'on-other-arch' fetches nothing."""

        self.run_snapcraft(['prime', 'on-other-arch'], 'stage-package-grammar')

        self.assertThat(
            os.path.join('prime', 'usr', 'bin', 'hello'),
            Not(FileExists()))

    def test_on_other_arch_else(self):
        """Test that 'on-other-arch-else' fetches stage package."""

        self.run_snapcraft(
            ['prime', 'on-other-arch-else'], 'stage-package-grammar')

        self.assertThat(
            os.path.join('prime', 'usr', 'bin', 'hello'),
            FileExists())

    def test_on_other_arch_else_fail(self):
        """Test that 'on-other-arch-else-fail' fails with an error."""

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['prime', 'on-other-arch-else-fail'], 'stage-package-grammar')

        self.assertThat(exception.output, Contains(
            "Unable to satisfy 'on other-arch', failure forced"))
