# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from textwrap import dedent

import testscenarios
from testtools.matchers import Contains, FileContains, FileExists, Not

from tests import integration


class PrimeTestCase(integration.TestCase):
    def test_prime_includes_stage_fileset(self):
        self.run_snapcraft("prime", "prime-from-stage")
        self.assertThat(os.path.join(self.prime_dir, "without-a"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "without-b"), Not(FileExists()))
        self.assertThat(os.path.join(self.prime_dir, "without-c"), FileExists())

    def test_prime_includes_stage_excludes_fileset(self):
        self.run_snapcraft("prime", "prime-from-stage")
        self.assertThat(os.path.join(self.prime_dir, "with-a"), Not(FileExists()))
        self.assertThat(os.path.join(self.prime_dir, "with-b"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "with-c"), FileExists())

    def test_prime_with_non_ascii_desktop_file(self):
        # Originally, in this test we forced LC_ALL=C. However, now that we
        # are using the click python library we can't do it because it fails
        # to run any command when the system language is ascii.
        # --20170518 - elopio
        self.run_snapcraft("prime", "desktop-with-non-ascii")

        desktop_path = os.path.join(self.prime_dir, "meta", "gui", "test-app.desktop")

        self.expectThat(desktop_path, FileContains(matcher=Contains("non ascíí")))


class PrimedAssetsTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        ("setup", dict(project_dir="assets-with-gui-in-setup")),
        ("snap", dict(project_dir="assets-with-gui-in-snap")),
    ]

    def test_assets_in_meta(self):
        self.run_snapcraft("prime", self.project_dir)

        gui_dir = os.path.join(self.prime_dir, "meta", "gui")
        expected_desktop = dedent(
            """\
            [Desktop Entry]
            Name=My App
            Exec=my-app
            Type=Application
            """
        )
        self.expectThat(os.path.join(gui_dir, "icon.png"), FileExists())
        self.expectThat(
            os.path.join(gui_dir, "my-app.desktop"), FileContains(expected_desktop)
        )
