# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
from textwrap import dedent

import testscenarios
from testtools.matchers import (
    Contains,
    Equals,
    FileContains,
    FileExists,
    Not,
)

import snapcraft
import integration_tests


class PrimeTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def test_prime_includes_stage_fileset(self):
        self.run_snapcraft('prime', 'prime-from-stage')
        self.assertThat(
            os.path.join(self.prime_dir, 'without-a'),
            FileExists())
        self.assertThat(
            os.path.join(self.prime_dir, 'without-b'),
            Not(FileExists()))
        self.assertThat(
            os.path.join(self.prime_dir, 'without-c'),
            FileExists())

    def test_prime_includes_stage_excludes_fileset(self):
        self.run_snapcraft('prime', 'prime-from-stage')
        self.assertThat(
            os.path.join(self.prime_dir, 'with-a'),
            Not(FileExists()))
        self.assertThat(
            os.path.join(self.prime_dir, 'with-b'),
            FileExists())
        self.assertThat(
            os.path.join(self.prime_dir, 'with-c'),
            FileExists())

    def test_prime_with_non_ascii_desktop_file(self):
        # Originally, in this test we forced LC_ALL=C. However, now that we
        # are using the click python library we can't do it because it fails
        # to run any command when the system language is ascii.
        # --20170518 - elopio
        self.run_snapcraft('prime', 'desktop-with-non-ascii')

        desktop_path = os.path.join(
            self.prime_dir, 'meta', 'gui', 'test-app.desktop')

        self.expectThat(
            desktop_path, FileContains(matcher=Contains('non ascíí')))

    def _prime_invalid_part(self, debug):
        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['prime', 'invalid-part-name'],
            'prime-from-stage', debug=debug)

        self.assertThat(exception.returncode, Equals(2))
        self.assertThat(exception.output, Contains(
            "part named 'invalid-part-name' is not defined"))

        return exception.output

    def test_prime_invalid_part_no_traceback_without_debug(self):
        self.assertThat(
            self._prime_invalid_part(False), Not(Contains("Traceback")))

    def test_prime_invalid_part_does_traceback_with_debug(self):
        self.assertThat(
            self._prime_invalid_part(True), Contains("Traceback"))


class PrimedAssetsTestCase(testscenarios.WithScenarios,
                           integration_tests.TestCase):

    scenarios = [
        ('setup', dict(project_dir='assets-with-gui-in-setup')),
        ('snap', dict(project_dir='assets-with-gui-in-snap')),
    ]

    def test_assets_in_meta(self):
        self.run_snapcraft('prime', self.project_dir)

        gui_dir = os.path.join(self.prime_dir, 'meta', 'gui')
        expected_desktop = dedent("""\
            [Desktop Entry]
            Name=My App
            Exec=my-app
            Type=Application
            """)
        self.expectThat(os.path.join(gui_dir, 'icon.png'), FileExists())
        self.expectThat(os.path.join(gui_dir, 'my-app.desktop'),
                        FileContains(expected_desktop))
