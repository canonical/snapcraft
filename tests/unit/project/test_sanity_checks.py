# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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
from snapcraft.project._sanity_checks import conduct_project_sanity_check

from tests import fixture_setup, unit


class ProjectSanityChecksTest(unit.TestCase):
    scenarios = [
        ("managed", dict(is_managed_host=True, snap_asset_dir="snap")),
        ("unmanaged", dict(is_managed_host=False, snap_asset_dir="snap")),
        (
            "managed build-aux",
            dict(
                is_managed_host=True, snap_asset_dir=os.path.join("build-aux", "snap")
            ),
        ),
        (
            "unmanaged build-aux",
            dict(
                is_managed_host=False, snap_asset_dir=os.path.join("build-aux", "snap")
            ),
        ),
    ]

    def setUp(self):
        super().setUp()
        self.fake_logger = self.useFixture(fixtures.FakeLogger(level=logging.WARN))
        self.useFixture(
            fixtures.EnvironmentVariable("HOME", os.path.join(self.path, "fake-home"))
        )

    def assert_check_passes(self):
        self.run_check()
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_no_snap_dir(self):
        self.assert_check_passes()

    def test_snapcraft_yaml(self):
        os.makedirs(self.snap_asset_dir)
        open(os.path.join(self.snap_asset_dir, "snapcraft.yaml"), "w").close()
        self.assert_check_passes()

    def test_global_state(self):
        state_dir = os.path.join(self.snap_asset_dir, ".snapcraft")
        os.makedirs(state_dir)
        open(os.path.join(state_dir, "state"), "w").close()
        self.assert_check_passes()

    def test_icons(self):
        gui_dir = os.path.join(self.snap_asset_dir, "gui")
        os.makedirs(gui_dir)
        open(os.path.join(gui_dir, "icon.png"), "w").close()
        open(os.path.join(gui_dir, "another-icon.svg"), "w").close()
        self.assert_check_passes()

    def test_plugin(self):
        plugins_dir = os.path.join(self.snap_asset_dir, "plugins")
        os.makedirs(plugins_dir)
        open(os.path.join(plugins_dir, "plugin1.py"), "w").close()
        open(os.path.join(plugins_dir, "data-file"), "w").close()
        self.assert_check_passes()

    def test_hooks(self):
        plugins_dir = os.path.join(self.snap_asset_dir, "hooks")
        os.makedirs(plugins_dir)
        open(os.path.join(plugins_dir, "configure"), "w").close()
        open(os.path.join(plugins_dir, "random-hook-2"), "w").close()
        self.assert_check_passes()

    def test_keys(self):
        keys_dir = os.path.join(self.snap_asset_dir, "keys")
        os.makedirs(keys_dir)
        open(os.path.join(keys_dir, "fake-key.asc"), "w").close()
        open(os.path.join(keys_dir, "key2.asc"), "w").close()
        self.assert_check_passes()

    def test_local(self):
        local_dir = os.path.join(self.snap_asset_dir, "local")
        local_subdir = os.path.join(local_dir, "subdir")
        os.makedirs(local_subdir)
        open(os.path.join(local_dir, "file1"), "w").close()
        open(os.path.join(local_subdir, "file2"), "w").close()
        self.assert_check_passes()

    def test_unexpected_things(self):
        dir1 = os.path.join(self.snap_asset_dir, "dir1")
        dir2 = os.path.join(self.snap_asset_dir, "dir2")
        gui_dir = os.path.join(self.snap_asset_dir, "gui")
        keys_dir = os.path.join(self.snap_asset_dir, "keys")
        state_dir = os.path.join(self.snap_asset_dir, ".snapcraft")
        fake_gui_dir = os.path.join(dir2, "gui")
        fake_hooks_dir = os.path.join(dir1, "hooks")
        fake_local_dir = os.path.join(dir2, "local")
        fake_plugins_dir = os.path.join(dir1, "plugins")
        os.makedirs(dir1)
        os.makedirs(dir2)
        os.makedirs(gui_dir)
        os.makedirs(keys_dir)
        os.makedirs(state_dir)
        os.makedirs(fake_gui_dir)
        os.makedirs(fake_hooks_dir)
        os.makedirs(fake_local_dir)
        os.makedirs(fake_plugins_dir)
        open(os.path.join(dir1, "foo"), "w").close()
        open(os.path.join(dir2, "bar"), "w").close()
        open(os.path.join(gui_dir, "icon.jpg"), "w").close()
        open(os.path.join(keys_dir, "invalid.key"), "w").close()
        open(os.path.join(state_dir, "baz"), "w").close()

        self.run_check()
        self.assertThat(
            self.fake_logger.output,
            Equals(
                (
                    "The {snap_asset_dir!r} directory is meant specifically for snapcraft, but it "
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
                    "- dir2/local\n"
                    "- gui/icon.jpg\n"
                    "- keys/invalid.key\n"
                    "\n"
                    "If you must store these files within the {snap_asset_dir!r} directory, move them "
                    "to '{snap_asset_dir}/local', which is ignored by snapcraft.\n"
                ).format(snap_asset_dir=self.snap_asset_dir)
            ),
        )

    def run_check(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            os.path.dirname(self.snap_asset_dir)
        )
        snapcraft_yaml.update_part("fake", dict(plugin="nil"))
        self.useFixture(snapcraft_yaml)
        project = snapcraft.project.Project(
            is_managed_host=self.is_managed_host,
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path,
        )
        conduct_project_sanity_check(project)
