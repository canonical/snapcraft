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
import yaml
from typing import List

import snapcraft

from snapcraft.tests import integration
from testtools.matchers import Contains, Equals, HasLength


def _snap_architectures() -> List[str]:
    with open(os.path.join('prime', 'meta', 'snap.yaml'), 'r') as f:
        return yaml.load(f)['architectures']


class ArchitecturesGrammarTestCase(integration.TestCase):

    def test_simple(self):
        """Test that grammar handles standalone architectures."""

        self.run_snapcraft(['prime'], 'multiarch')

        architectures = _snap_architectures()
        self.assertThat(architectures, HasLength(2))
        self.assertThat(architectures, Contains('armhf'))
        self.assertThat(architectures, Contains('amd64'))

    def test_try(self):
        """Test that 'try' uses valid architectures."""

        self.run_snapcraft(['prime'], 'architectures-grammar-try')

        self.assertThat(_snap_architectures(), Equals(['armhf']))

    def test_try_skipped(self):
        """Test that 'try' skips invalid architectures."""

        self.run_snapcraft(['prime'], 'architectures-grammar-try-skip')

        # This means that the resulting snap should have an arch equal to the
        # host
        self.assertThat(
            _snap_architectures(),
            Equals([snapcraft.ProjectOptions().deb_arch]))

    def test_try_else(self):
        """Test that 'try' moves to the 'else' branch if body is invalid"""

        self.run_snapcraft(['prime'], 'architectures-grammar-try-else')

        self.assertThat(_snap_architectures(), Equals(['armhf']))

    def test_on_other_arch(self):
        """Test that 'on' does nothing when running on another arch."""

        self.run_snapcraft(['prime'], 'architectures-grammar-on')

        # This means that the resulting snap should have an arch equal to the
        # host
        self.assertThat(
            _snap_architectures(),
            Equals([snapcraft.ProjectOptions().deb_arch]))

    def test_on_other_arch_else(self):
        """Test that 'on' moves to the 'else' branch if on other arch."""

        self.run_snapcraft(['prime'], 'architectures-grammar-on-else')

        self.assertThat(_snap_architectures(), Equals(['armhf']))

    def test_on_other_arch_else_fail(self):
        """Test that 'on' fails with an error if it hits an 'else fail'."""

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['prime'], 'architectures-grammar-fail')

        self.assertThat(exception.output, Contains(
            "Unable to satisfy 'on other-arch', failure forced"))
