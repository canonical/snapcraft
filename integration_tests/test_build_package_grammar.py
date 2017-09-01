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

import subprocess

import snapcraft

import integration_tests
from testtools.matchers import Contains


class BuildPackageGrammarTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if self._hello_is_installed():
            self.fail(
                'This integration test cannot run if you already have the '
                "'hello' package installed. Please uninstall it before "
                'running this test.')

    def tearDown(self):
        super().tearDown()

        # Remove hello. This is safe since the test fails if hello was already
        # installed.
        try:
            subprocess.check_output(
                ['sudo', 'apt', 'remove', 'hello', '-y'],
                stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            self.fail("unable to remove 'hello': {}".format(e.output))

    def _hello_is_installed(self):
        return snapcraft.repo.Ubuntu.is_package_installed('hello')

    def test_simple(self):
        """Test that grammar installs standalone build package."""

        self.run_snapcraft(['pull'], 'build-package-grammar')

        self.assertTrue(self._hello_is_installed())

    def test_try(self):
        """Test that 'try' installs valid build packages."""

        self.run_snapcraft(['pull'], 'build-package-grammar-try')

        self.assertTrue(self._hello_is_installed())

    def test_try_skipped(self):
        """Test that 'try' skips invalid build packages."""

        self.run_snapcraft(['pull'], 'build-package-grammar-try-skip')

        self.assertFalse(self._hello_is_installed())

    def test_try_else(self):
        """Test that 'try' moves to the 'else' branch if body is invalid"""

        self.run_snapcraft(['pull'], 'build-package-grammar-try-else')

        self.assertTrue(self._hello_is_installed())

    def test_on_other_arch(self):
        """Test that 'on' fetches nothing when running on another arch."""

        self.run_snapcraft(['pull'], 'build-package-grammar-on')

        self.assertFalse(self._hello_is_installed())

    def test_on_other_arch_else(self):
        """Test that 'on' moves to the 'else' branch if on other arch."""

        self.run_snapcraft(['pull'], 'build-package-grammar-on-else')

        self.assertTrue(self._hello_is_installed())

    def test_on_other_arch_else_fail(self):
        """Test that 'on' fails with an error if it hits an 'else fail'."""

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['pull'], 'build-package-grammar-fail')

        self.assertThat(exception.output, Contains(
            "Unable to satisfy 'on other-arch', failure forced"))

    def test_global_build_package_on_other_arch_else(self):
        """Test that grammar works in global build packages as well."""

        self.run_snapcraft(['pull'], 'build-package-grammar-global')

        self.assertTrue(self._hello_is_installed())
