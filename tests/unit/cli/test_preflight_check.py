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

import logging
import os

import fixtures
from testtools.matchers import Equals

import snapcraft
from snapcraft.cli._preflight_check import conduct_preflight_check

from tests import unit


class PreflightChecksTest(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = self.useFixture(fixtures.FakeLogger(level=logging.WARN))

    def assert_check_passes(self):
        _run_check()
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_no_snap_dir(self):
        self.assert_check_passes()

    def test_snapcraft_yaml(self):
        os.makedirs("snap")
        open(os.path.join("snap", "snapcraft.yaml"), "w").close()
        self.assert_check_passes()

    def test_global_state(self):
        state_dir = os.path.join("snap", ".snapcraft")
        os.makedirs(state_dir)
        open(os.path.join(state_dir, "state"), "w").close()
        self.assert_check_passes()

    def test_icons(self):
        gui_dir = os.path.join("snap", "gui")
        os.makedirs(gui_dir)
        open(os.path.join(gui_dir, "icon.png"), "w").close()
        open(os.path.join(gui_dir, "another-icon.svg"), "w").close()
        self.assert_check_passes()

    def test_plugin(self):
        plugins_dir = os.path.join("snap", "plugins")
        os.makedirs(plugins_dir)
        open(os.path.join(plugins_dir, "plugin1.py"), "w").close()
        open(os.path.join(plugins_dir, "data-file"), "w").close()
        self.assert_check_passes()

    def test_hooks(self):
        plugins_dir = os.path.join("snap", "hooks")
        os.makedirs(plugins_dir)
        open(os.path.join(plugins_dir, "configure"), "w").close()
        open(os.path.join(plugins_dir, "random-hook-2"), "w").close()
        self.assert_check_passes()

    def test_unexpected_things(self):
        dir1 = os.path.join("snap", "dir1")
        dir2 = os.path.join("snap", "dir2")
        gui_dir = os.path.join("snap", "gui")
        state_dir = os.path.join("snap", ".snapcraft")
        fake_plugins_dir = os.path.join(dir1, "plugins")
        fake_hooks_dir = os.path.join(dir1, "hooks")
        fake_gui_dir = os.path.join(dir2, "gui")
        os.makedirs(dir1)
        os.makedirs(dir2)
        os.makedirs(gui_dir)
        os.makedirs(state_dir)
        os.makedirs(fake_plugins_dir)
        os.makedirs(fake_hooks_dir)
        os.makedirs(fake_gui_dir)
        open(os.path.join(dir1, "foo"), "w").close()
        open(os.path.join(dir2, "bar"), "w").close()
        open(os.path.join(gui_dir, "icon.jpg"), "w").close()
        open(os.path.join(state_dir, "baz"), "w").close()

        _run_check()
        self.assertThat(
            self.fake_logger.output,
            Equals(
                "The snap/ directory is meant specifically for snapcraft, but it "
                "contains the following non-snapcraft-related paths, which is "
                "unsupported and will cause unexpected behavior:\n"
                "- .snapcraft/baz\n"
                "- dir1\n"
                "- dir1/foo\n"
                "- dir1/hooks\n"
                "- dir1/plugins\n"
                "- dir2\n"
                "- dir2/bar\n"
                "- dir2/gui\n"
                "- gui/icon.jpg\n"
            ),
        )


def _run_check():
    project = snapcraft.project.Project()
    conduct_preflight_check(project)
