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
import textwrap

import fixtures
from testtools.matchers import Equals

import snapcraft
from snapcraft.project._sanity_checks import (
    conduct_project_sanity_check,
    conduct_environment_sanity_check,
    conduct_config_sanity_check,
)
from snapcraft.project import errors, _schema
from snapcraft.internal import project_loader

from tests import unit


class ProjectSanityChecksTest(unit.TestCase):
    scenarios = [
        ("managed", dict(is_managed_host=True)),
        ("unmanaged", dict(is_managed_host=False)),
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

    def test_local(self):
        local_dir = os.path.join("snap", "local")
        local_subdir = os.path.join(local_dir, "subdir")
        os.makedirs(local_subdir)
        open(os.path.join(local_dir, "file1"), "w").close()
        open(os.path.join(local_subdir, "file2"), "w").close()
        self.assert_check_passes()

    def test_unexpected_things(self):
        dir1 = os.path.join("snap", "dir1")
        dir2 = os.path.join("snap", "dir2")
        gui_dir = os.path.join("snap", "gui")
        state_dir = os.path.join("snap", ".snapcraft")
        fake_plugins_dir = os.path.join(dir1, "plugins")
        fake_hooks_dir = os.path.join(dir1, "hooks")
        fake_gui_dir = os.path.join(dir2, "gui")
        fake_local_dir = os.path.join(dir2, "local")
        os.makedirs(dir1)
        os.makedirs(dir2)
        os.makedirs(gui_dir)
        os.makedirs(state_dir)
        os.makedirs(fake_plugins_dir)
        os.makedirs(fake_hooks_dir)
        os.makedirs(fake_gui_dir)
        os.makedirs(fake_local_dir)
        open(os.path.join(dir1, "foo"), "w").close()
        open(os.path.join(dir2, "bar"), "w").close()
        open(os.path.join(gui_dir, "icon.jpg"), "w").close()
        open(os.path.join(state_dir, "baz"), "w").close()

        self.run_check()
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
                "- dir2/local\n"
                "- gui/icon.jpg\n\n"
                "If you must store these files within the snap/ directory, move them "
                "to snap/local/, which is ignored by snapcraft.\n"
            ),
        )

    def run_check(self):
        project = snapcraft.project.Project(is_managed_host=self.is_managed_host)
        conduct_project_sanity_check(project)


class EnvironmentSanityChecksTest(unit.TestCase):
    def test_command_chain_with_none_adapter(self):
        project = snapcraft.project.Project()
        yaml_data = {
            "apps": {
                "test-app": {"command-chain": ["test-command-chain"], "adapter": "none"}
            }
        }

        try:
            conduct_environment_sanity_check(
                project, yaml_data, _schema.Validator().schema
            )
        except Exception:
            self.fail("No exception was expected")

    def test_command_chain_with_full_adapter(self):
        project = snapcraft.project.Project()
        yaml_data = {
            "apps": {
                "test-app": {"command-chain": ["test-command-chain"], "adapter": "full"}
            }
        }

        try:
            conduct_environment_sanity_check(
                project, yaml_data, _schema.Validator().schema
            )
        except Exception:
            self.fail("No exception was expected")

    # FIXME: This test should fail once the default adapter changes to "full", just
    # remove it
    def test_command_chain_without_adapter_raises(self):
        project = snapcraft.project.Project()
        yaml_data = {"apps": {"test-app": {"command-chain": ["test-command-chain"]}}}

        raised = self.assertRaises(
            errors.CommandChainWithLegacyAdapterError,
            conduct_environment_sanity_check,
            project,
            yaml_data,
            _schema.Validator().schema,
        )

        self.assertThat(raised.app_name, Equals("test-app"))

    def test_command_chain_with_legacy_adapter_raises(self):
        project = snapcraft.project.Project()
        yaml_data = {
            "apps": {
                "test-app": {
                    "command-chain": ["test-command-chain"],
                    "adapter": "legacy",
                }
            }
        }

        raised = self.assertRaises(
            errors.CommandChainWithLegacyAdapterError,
            conduct_environment_sanity_check,
            project,
            yaml_data,
            _schema.Validator().schema,
        )

        self.assertThat(raised.app_name, Equals("test-app"))


class ConfigSanityChecksTest(unit.TestCase):
    scenarios = [
        ("appdata", dict(suffix="appdata.xml")),
        ("metainfo", dict(suffix="metainfo.xml")),
    ]

    def make_snapcraft_project(self, body):
        yaml = textwrap.dedent(
            """\
            name: test
            base: core18
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable
            {body}
            """
        )

        yaml_path = self.make_snapcraft_yaml(yaml.format(body=body))
        project = snapcraft.project.Project(snapcraft_yaml_file_path=yaml_path)
        return project_loader.load_config(project)

    def test_common_id_valid(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                apps:
                  test-app:
                    command: test
                    common-id: com.test.app.desktop
                parts:
                  test-part:
                    plugin: nil
                    parse-info: [foo/bar/com.test.app.{}]
                """.format(
                    self.suffix
                )
            )
        )
        try:
            conduct_config_sanity_check(project_config)
        except errors.CommonIdMismatchError as e:
            self.fail(str(e))

    def test_common_id_invalid(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                apps:
                  test-app:
                    command: test
                    common-id: com.tset.app.desktop
                parts:
                  test-part:
                    plugin: nil
                    parse-info: [foo/bar/com.test.app.{}]
                """.format(
                    self.suffix
                )
            )
        )
        raised = self.assertRaises(
            errors.CommonIdMismatchError, conduct_config_sanity_check, project_config
        )
        self.assertThat(
            str(raised),
            Equals(
                "Common ID 'com.tset.app.desktop' specified in app 'test-app' doesn't match any metadata file."
            ),
        )
