# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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
from unittest import mock

import fixtures
from testtools.matchers import (
    Contains,
    DirExists,
    Equals,
    FileContains,
    FileExists,
    Not,
)

import snapcraft
from snapcraft.internal import errors, lifecycle, pluginhandler, project_loader, steps
from snapcraft.internal.lifecycle._runner import _replace_in_part
from snapcraft.project import Project
from tests import fixture_setup

from . import LifecycleTestBase


class ExecutionTestCase(LifecycleTestBase):
    def test_replace_in_parts(self):
        class Options:
            def __init__(self):
                self.source = "$SNAPCRAFT_PART_INSTALL"

        class Plugin:
            def __init__(self):
                self.options = Options()

        class Part:
            def __init__(self):
                self.plugin = Plugin()
                self.part_source_dir = "/tmp"
                self.part_source_work_dir = "/tmp"
                self.part_build_dir = "/tmp"
                self.part_build_work_dir = "/tmp"
                self.part_install_dir = "/tmp"

        part = Part()
        new_part = _replace_in_part(part)

        self.assertThat(new_part.plugin.options.source, Equals(part.part_install_dir))

    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_dependency_is_staged_when_required(self, mock_install_build_snaps):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                """
            )
        )

        lifecycle.execute(steps.PULL, project_config, part_names=["part2"])

        self.assertThat(
            self.fake_logger.output,
            Contains("'part2' has dependencies that need to be staged: part1"),
        )

    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_no_exception_when_dependency_is_required_but_already_staged(
        self, mock_install_build_snaps
    ):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                """
            )
        )

        def _fake_should_step_run(self, step, force=False):
            return self.name != "part1"

        with mock.patch.object(
            pluginhandler.PluginHandler, "should_step_run", _fake_should_step_run
        ):
            lifecycle.execute(steps.PULL, project_config, part_names=["part2"])

        self.assertThat(self.fake_logger.output, Contains("Pulling part2"))
        self.assertThat(self.fake_logger.output, Not(Contains("Pulling part1")))

    def test_dirty_stage_part_with_built_dependent_raises(self):
        # Set the option to error on dirty/outdated steps
        with snapcraft.config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(
                snapcraft.config.OutdatedStepAction.ERROR
            )

        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after: [part1]
                """
            )
        )

        # Stage dependency
        lifecycle.execute(steps.STAGE, project_config, part_names=["part1"])
        # Build dependent
        lifecycle.execute(steps.BUILD, project_config, part_names=["part2"])

        def _fake_dirty_report(self, step):
            if step == steps.STAGE:
                return pluginhandler.DirtyReport(
                    dirty_properties={"foo"}, dirty_project_options={"bar"}
                )
            return None

        # Should stage no problem
        with mock.patch.object(
            pluginhandler.PluginHandler, "get_dirty_report", _fake_dirty_report
        ):
            lifecycle.execute(steps.STAGE, project_config, part_names=["part1"])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Should raise an error since part2 is now dirty
        raised = self.assertRaises(
            errors.StepOutdatedError, lifecycle.execute, steps.BUILD, project_config
        )

        output = self.fake_logger.output.split("\n")
        part1_output = [line.strip() for line in output if "part1" in line]
        self.assertThat(part1_output, Equals(["Skipping pull part1 (already ran)"]))

        self.assertThat(raised.step, Equals(steps.PULL))
        self.assertThat(raised.part, Equals("part2"))
        self.assertThat(raised.report, Equals("A dependency has changed: 'part1'\n"))

    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_dirty_build_raises(self, mock_install_build_snaps):
        # Set the option to error on dirty/outdated steps
        with snapcraft.config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(
                snapcraft.config.OutdatedStepAction.ERROR
            )

        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                """
            )
        )

        # Build it.
        lifecycle.execute(steps.BUILD, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.BUILD:
                return pluginhandler.DirtyReport(dirty_properties={"foo", "bar"})
            return None

        # Should catch that the part needs to be rebuilt and raise an error.
        with mock.patch.object(
            pluginhandler.PluginHandler, "get_dirty_report", _fake_dirty_report
        ):
            raised = self.assertRaises(
                errors.StepOutdatedError, lifecycle.execute, steps.BUILD, project_config
            )

        self.assertThat(
            self.fake_logger.output, Equals("Skipping pull part1 (already ran)\n")
        )

        self.assertThat(raised.step, Equals(steps.BUILD))
        self.assertThat(raised.part, Equals("part1"))
        self.assertThat(
            raised.report,
            Equals("The 'bar' and 'foo' part properties appear to have changed.\n"),
        )
        self.assertThat(raised.parts_names, Equals("part1"))

    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_dirty_pull_raises(self, mock_install_build_snaps):
        # Set the option to error on dirty/outdated steps
        with snapcraft.config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(
                snapcraft.config.OutdatedStepAction.ERROR
            )

        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                """
            )
        )

        # Pull it.
        lifecycle.execute(steps.PULL, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.PULL:
                return pluginhandler.DirtyReport(dirty_project_options={"foo", "bar"})
            return None

        # Should catch that the part needs to be re-pulled and raise an error.
        with mock.patch.object(
            pluginhandler.PluginHandler, "get_dirty_report", _fake_dirty_report
        ):
            raised = self.assertRaises(
                errors.StepOutdatedError, lifecycle.execute, steps.PULL, project_config
            )

        self.assertThat(self.fake_logger.output, Equals(""))

        self.assertThat(raised.step, Equals(steps.PULL))
        self.assertThat(raised.part, Equals("part1"))
        self.assertThat(
            raised.report,
            Equals("The 'bar' and 'foo' project options appear to have changed.\n"),
        )

    @mock.patch.object(snapcraft.BasePlugin, "enable_cross_compilation")
    @mock.patch("snapcraft.repo.Repo.install_build_packages")
    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_pull_is_dirty_if_target_arch_changes(
        self,
        mock_install_build_snaps,
        mock_install_build_packages,
        mock_enable_cross_compilation,
    ):
        # Set the option to error on dirty/outdated steps
        with snapcraft.config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(
                snapcraft.config.OutdatedStepAction.ERROR
            )

        mock_install_build_packages.return_value = []
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                """
            )
        )

        project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path,
            target_deb_arch="amd64",
        )
        project_config = project_loader.load_config(project)
        # Pull it with amd64
        lifecycle.execute(steps.PULL, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path,
            target_deb_arch="armhf",
        )
        project_config = project_loader.load_config(project)
        # Pull it again with armhf. Should catch that the part needs to be
        # re-pulled due to the change in target architecture and raise an
        # error.
        raised = self.assertRaises(
            errors.StepOutdatedError, lifecycle.execute, steps.PULL, project_config
        )

        self.assertThat(
            self.fake_logger.output, Contains("Setting target machine to 'armhf'")
        )

        self.assertThat(raised.step, Equals(steps.PULL))
        self.assertThat(raised.part, Equals("part1"))
        self.assertThat(
            raised.report,
            Equals("The 'deb_arch' project option appears to have changed.\n"),
        )

    def test_prime_excludes_internal_snapcraft_dir(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  test-part:
                    plugin: nil
                """
            )
        )
        lifecycle.execute(steps.PRIME, project_config)
        self.assertThat(
            os.path.join(steps.PRIME.name, "snap", ".snapcraft"), Not(DirExists())
        )

    def test_non_prime_and_no_version(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, version=None)
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part",
            {"plugin": "nil", "override-build": "snapcraftctl set-version 1.0"},
        )
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        project_config = project_loader.load_config(project)

        # This should not fail
        lifecycle.execute(steps.PULL, project_config)


class CleanTestCase(LifecycleTestBase):
    def test_clean_removes_global_state(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  test-part:
                    plugin: nil
                """
            )
        )
        lifecycle.execute(steps.PULL, project_config)
        lifecycle.clean(project_config.project, parts=None)
        self.assertThat(os.path.join("snap", ".snapcraft"), Not(DirExists()))

    @mock.patch("snapcraft.internal.mountinfo.MountInfo.for_root")
    def test_clean_leaves_prime_alone_for_tried(self, mock_for_root):
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  test-part:
                    plugin: nil
                """
            )
        )
        lifecycle.execute(steps.PRIME, project_config)
        lifecycle.clean(project_config.project, parts=None)
        self.assertThat(
            steps.PRIME.name,
            DirExists(),
            "Expected prime directory to remain after cleaning for tried snap",
        )


class RecordSnapcraftYamlTestCase(LifecycleTestBase):
    def test_prime_without_build_info_does_not_record(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_INFO", None))
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  test-part:
                    plugin: nil
                """
            )
        )
        lifecycle.execute(steps.PRIME, project_config)
        for file_name in ("snapcraft.yaml", "manifest.yaml"):
            self.assertThat(
                os.path.join(steps.PRIME.name, "snap", file_name), Not(FileExists())
            )

    def test_prime_with_build_info_records_snapcraft_yaml(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_INFO", "1"))
        project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  test-part:
                    plugin: nil
                """
            ),
            snap_type="type: app",
        )
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent(
            """\
            name: test
            base: core18
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable
            type: app

            parts:
              test-part:
                plugin: nil

            """
        )

        self.assertThat(
            os.path.join(steps.PRIME.name, "snap", "snapcraft.yaml"),
            FileContains(expected),
        )
