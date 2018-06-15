
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

import contextlib
import logging
import os
import re
import shutil
import subprocess
import textwrap
from unittest import mock

import fixtures
from testtools.matchers import (
    Contains,
    DirExists,
    Equals,
    FileContains,
    FileExists,
    MatchesRegex,
    Not,
)

import snapcraft
from snapcraft import config, storeapi
from snapcraft.file_utils import calculate_sha3_384
from snapcraft.internal import (errors, pluginhandler, lifecycle,
                                project_loader, steps)
from snapcraft.internal.lifecycle._runner import _replace_in_part
from snapcraft.project import Project
from tests import fixture_setup, unit
from tests.fixture_setup.os_release import FakeOsRelease


class BaseLifecycleTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def make_snapcraft_project(self, parts, snap_type=''):
        yaml = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            {type}

            {parts}
            """)

        self.snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            yaml.format(parts=parts, type=snap_type))
        project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path)
        return project_loader.load_config(project)


class ExecutionTestCase(BaseLifecycleTestCase):

    def test_replace_in_parts(self):
        class Options:
            def __init__(self):
                self.source = '$SNAPCRAFT_PART_INSTALL'

        class Plugin:
            def __init__(self):
                self.options = Options()
                self.sourcedir = '/tmp'
                self.builddir = '/tmp'
                self.installdir = '/tmp'

        class Part:
            def __init__(self):
                self.plugin = Plugin()

        part = Part()
        new_part = _replace_in_part(part)

        self.assertThat(
            new_part.plugin.options.source, Equals(part.plugin.installdir))

    def test_dependency_is_staged_when_required(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                """))

        lifecycle.execute(steps.PULL, project_config, part_names=['part2'])

        self.assertThat(
            self.fake_logger.output,
            Equals("'part2' has prerequisites that need to be staged: part1\n"
                   'Pulling part1 \n'
                   'Building part1 \n'
                   'Staging part1 \n'
                   'Pulling part2 \n'))

    def test_no_exception_when_dependency_is_required_but_already_staged(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                """))

        def _fake_should_step_run(self, step, force=False):
            return self.name != 'part1'

        with mock.patch.object(pluginhandler.PluginHandler,
                               'should_step_run',
                               _fake_should_step_run):
            lifecycle.execute(steps.PULL, project_config, part_names=['part2'])

        self.assertThat(self.fake_logger.output, Equals('Pulling part2 \n'))

    def test_dependency_recursed_correctly(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                  part3:
                    plugin: nil
                    after:
                      - part2
                """))

        snap_info = lifecycle.execute(steps.PULL, project_config)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [project_config.project.deb_arch],
            'type': ''
        }
        self.assertThat(snap_info, Equals(expected_snap_info))

        self.assertThat(
            self.fake_logger.output, Equals(
                'Pulling part1 \n'
                '\'part2\' has prerequisites that need to be staged: part1\n'
                'Skipping pull part1 (already ran)\n'
                'Building part1 \n'
                'Staging part1 \n'
                'Pulling part2 \n'
                '\'part3\' has prerequisites that need to be staged: part2\n'
                'Skipping pull part2 (already ran)\n'
                'Building part2 \n'
                'Staging part2 \n'
                'Pulling part3 \n',
            ))

    def test_os_type_returned_by_lifecycle(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                """),
            'type: os')

        snap_info = lifecycle.execute(steps.PULL, project_config)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [project_config.project.deb_arch],
            'type': 'os'
        }
        self.assertThat(snap_info, Equals(expected_snap_info))

    def test_dirty_prime_reprimes_single_part(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

        # Strip it.
        lifecycle.execute(steps.PRIME, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if self.name == 'part1' and step == steps.PRIME:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should automatically clean and re-prime if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(steps.PRIME, project_config)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertThat(
            part2_output,
            Equals([
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Skipping prime part2 (already ran)',
            ]))

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Re-priming part1 (out of date)',
            ]))

    def test_dirty_prime_reprimes_multiple_part(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

        # Strip it.
        lifecycle.execute(steps.PRIME, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.PRIME:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should automatically clean and re-prime if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(steps.PRIME, project_config)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertThat(
            part2_output,
            Equals([
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Re-priming part2 (out of date)',
            ]))

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Re-priming part1 (out of date)',
            ]))

    def test_dirty_stage_restages_single_part(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

        # Stage it.
        lifecycle.execute(steps.STAGE, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if self.name == 'part1' and step == steps.STAGE:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(steps.STAGE, project_config)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertThat(
            part2_output,
            Equals([
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
            ]))

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Cleaning later steps and re-staging part1 (out of date)',
            ]))

    def test_dirty_stage_restages_multiple_parts(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

        # Stage it.
        lifecycle.execute(steps.STAGE, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.STAGE:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(steps.STAGE, project_config)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertThat(
            part2_output,
            Equals([
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Cleaning later steps and re-staging part2 (out of date)',
            ]))

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Cleaning later steps and re-staging part1 (out of date)',
            ]))

    def test_dirty_stage_part_with_built_dependent_raises(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after: [part1]
                """))

        # Stage dependency
        lifecycle.execute(steps.STAGE, project_config, part_names=['part1'])
        # Build dependent
        lifecycle.execute(steps.BUILD, project_config, part_names=['part2'])

        def _fake_dirty_report(self, step):
            if step == steps.STAGE:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should stage no problem
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(
                steps.STAGE, project_config, part_names=['part1'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Should raise an error since part2 is now dirty
        raised = self.assertRaises(
            errors.StepOutdatedError, lifecycle.execute, steps.BUILD,
            project_config)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)'
            ]))

        self.assertThat(raised.step, Equals(steps.PULL))
        self.assertThat(raised.part, Equals('part2'))
        self.assertThat(
            raised.report, Equals("A dependency has changed: 'part1'\n"))

    def test_dirty_steps_can_be_automatically_cleaned(self):
        # Set the option to automatically clean dirty steps
        with config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(
                config.OutdatedStepAction.CLEAN)

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after: [part1]
                """))

        # Stage dependency
        lifecycle.execute(steps.STAGE, project_config, part_names=['part1'])
        # Build dependent
        lifecycle.execute(steps.BUILD, project_config, part_names=['part2'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.STAGE:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            try:
                lifecycle.execute(
                    steps.STAGE, project_config, part_names=['part1'])
            except errors.StepOutdatedError:
                self.fail('Expected the step to automatically be cleaned')

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Cleaning later steps and re-staging part1 (out of date)',
            ]))

        self.assertThat(
            part2_output,
            Equals([
                "The following part is now out of date: 'part2'",
            ]))

    def test_dirty_stage_part_with_unbuilt_dependent(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after: [part1]
                """))

        # Stage dependency (dependent is unbuilt)
        lifecycle.execute(steps.STAGE, project_config, part_names=['part1'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.STAGE:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(steps.STAGE, project_config,
                              part_names=['part1'])

        self.assertThat(
            self.fake_logger.output, Equals(
                'Skipping pull part1 (already ran)\n'
                'Skipping build part1 (already ran)\n'
                'Cleaning later steps and re-staging part1 (out of date)\n'))

    def test_dirty_stage_reprimes(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

        # Strip it.
        lifecycle.execute(steps.PRIME, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.STAGE:
                return pluginhandler.DirtyReport({'foo'}, {'bar'}, [])
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute(steps.PRIME, project_config)

        self.assertThat(
            self.fake_logger.output, Equals(
                'Skipping pull part1 (already ran)\n'
                'Skipping build part1 (already ran)\n'
                'Cleaning later steps and re-staging part1 (out of date)\n'
                'Priming part1 \n'))

    def test_dirty_build_raises(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

        # Build it.
        lifecycle.execute(steps.BUILD, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.BUILD:
                return pluginhandler.DirtyReport({'foo', 'bar'}, set(), [])
            return None

        # Should catch that the part needs to be rebuilt and raise an error.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            raised = self.assertRaises(
                errors.StepOutdatedError,
                lifecycle.execute,
                steps.BUILD, project_config)

        self.assertThat(
            self.fake_logger.output, Equals(
                'Skipping pull part1 (already ran)\n'))

        self.assertThat(raised.step, Equals(steps.BUILD))
        self.assertThat(raised.part, Equals('part1'))
        self.assertThat(
            raised.report, Equals(
                "The 'bar' and 'foo' part properties appear to have "
                "changed.\n"))
        self.assertThat(raised.parts_names, Equals('part1'))

    def test_dirty_pull_raises(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

        # Pull it.
        lifecycle.execute(steps.PULL, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == steps.PULL:
                return pluginhandler.DirtyReport(set(), {'foo', 'bar'}, [])
            return None

        # Should catch that the part needs to be re-pulled and raise an error.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            raised = self.assertRaises(
                errors.StepOutdatedError,
                lifecycle.execute,
                steps.PULL, project_config)

        self.assertThat(self.fake_logger.output, Equals(''))

        self.assertThat(raised.step, Equals(steps.PULL))
        self.assertThat(raised.part, Equals('part1'))
        self.assertThat(
            raised.report,
            Equals(
                "The 'bar' and 'foo' project options appear to have changed.\n"
            ))

    @mock.patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    @mock.patch('snapcraft.repo.Repo.install_build_packages')
    def test_pull_is_dirty_if_target_arch_changes(
            self, mock_install_build_packages, mock_enable_cross_compilation):
        mock_install_build_packages.return_value = []
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

        project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path,
            target_deb_arch='amd64')
        project_config = project_loader.load_config(project)
        # Pull it with amd64
        lifecycle.execute(steps.PULL, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path,
            target_deb_arch='armhf')
        project_config = project_loader.load_config(project)
        # Pull it again with armhf. Should catch that the part needs to be
        # re-pulled due to the change in target architecture and raise an
        # error.
        raised = self.assertRaises(
            errors.StepOutdatedError,
            lifecycle.execute,
            steps.PULL, project_config)

        self.assertThat(
            self.fake_logger.output,
            Equals("Setting target machine to 'armhf'\n"))

        self.assertThat(raised.step, Equals(steps.PULL))
        self.assertThat(raised.part, Equals('part1'))
        self.assertThat(
            raised.report,
            Equals(
                "The 'deb_arch' project option appears to have changed.\n"))

    def test_prime_excludes_internal_snapcraft_dir(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', '.snapcraft'),
            Not(DirExists()))

    def test_non_prime_and_no_version(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, version=None)
        snapcraft_yaml.data['adopt-info'] = 'test-part'
        snapcraft_yaml.update_part(
            'test-part', {
                'plugin': 'nil',
                'override-build': 'snapcraftctl set-version 1.0'})
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)
        project_config = project_loader.load_config(project)

        # This should not fail
        lifecycle.execute(steps.PULL, project_config)


class RedoTestCase(BaseLifecycleTestCase):

    def setUp(self):
        super().setUp()

        self.project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

    def test_redo_prime(self):
        # Prime both parts.
        lifecycle.execute(steps.PRIME, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now prime only part1 again, which should automatically clean and
        # redo the prime step
        lifecycle.execute(
            steps.PRIME, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Re-priming part1',
            ]))

    def test_redo_stage(self):
        # Stage both parts.
        lifecycle.execute(steps.STAGE, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now stage only part1 again, which should automatically clean and
        # redo the stage step
        lifecycle.execute(
            steps.STAGE, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Cleaning later steps and re-staging part1',
            ]))

    def test_redo_stage_cleans_later_steps(self):
        # Prime both parts.
        lifecycle.execute(steps.PRIME, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now stage only part1 again, which should redo the stage step,
        # cleaning all later steps
        lifecycle.execute(
            steps.STAGE, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Cleaning later steps and re-staging part1',
            ]))

        self.assertThat(
            os.path.join(self.parts_dir, 'part1', 'state', 'prime'),
            Not(FileExists()))

    def test_redo_build(self):
        # Build both parts.
        lifecycle.execute(steps.BUILD, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now build only part1 again, which should automatically clean and
        # redo the build step
        lifecycle.execute(
            steps.BUILD, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Cleaning later steps and re-building part1',
            ]))

    def test_redo_build_cleans_later_steps(self):
        # Prime both parts.
        lifecycle.execute(steps.PRIME, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now build only part1 again, which should redo the build step,
        # cleaning all later steps
        lifecycle.execute(
            steps.BUILD, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Cleaning later steps and re-building part1',
            ]))

        for step in ('prime', 'stage'):
            self.assertThat(
                os.path.join(self.parts_dir, 'part1', 'state', step),
                Not(FileExists()))

    def test_redo_pull(self):
        # Pull both parts.
        lifecycle.execute(steps.PULL, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now pull only part1 again, which should automatically clean and
        # redo the pull step
        lifecycle.execute(
            steps.PULL, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Cleaning later steps and re-pulling part1',
            ]))

    def test_redo_pull_cleans_later_steps(self):
        # Prime both parts.
        lifecycle.execute(steps.PRIME, self.project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Now pull only part1 again, which should automatically clean all steps
        # and redo the pull step
        lifecycle.execute(
            steps.PULL, self.project_config, part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]

        self.assertThat(
            part1_output,
            Equals([
                'Cleaning later steps and re-pulling part1',
            ]))

        for step in (steps.PRIME, steps.STAGE, steps.BUILD):
            self.assertThat(
                os.path.join(self.parts_dir, 'part1', 'state', step.name),
                Not(FileExists()))


class DirtyBuildScriptletTestCase(BaseLifecycleTestCase):

    scenarios = (
        ('prepare scriptlet', {'scriptlet': 'prepare'}),
        ('build scriptlet', {'scriptlet': 'build'}),
        ('install scriptlet', {'scriptlet': 'install'}),
    )

    @mock.patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    @mock.patch('snapcraft.repo.Repo.install_build_packages')
    def test_build_is_dirty_if_scriptlet_changes(
            self, mock_install_build_packages, mock_enable_cross_compilation):
        mock_install_build_packages.return_value = []
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                    {}: touch scriptlet
                """).format(self.scriptlet))

        # Build it
        lifecycle.execute(steps.BUILD, project_config)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Change prepare scriptlet
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                    {}: touch changed
                """).format(self.scriptlet))

        # Build it again. Should catch that the scriptlet changed and it needs
        # to be rebuilt.
        raised = self.assertRaises(
            errors.StepOutdatedError,
            lifecycle.execute, steps.BUILD, project_config)

        self.assertThat(raised.step, Equals(steps.BUILD))
        self.assertThat(raised.part, Equals('part1'))
        self.assertThat(
            raised.report,
            Equals(
                "The {!r} part property appears to have changed.\n".format(
                    self.scriptlet)))


class CleanTestCase(BaseLifecycleTestCase):

    def test_clean_removes_global_state(self):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PULL, project_config)
        lifecycle.clean(project_config.project, parts=None)
        self.assertThat(
            os.path.join('snap', '.snapcraft'),
            Not(DirExists()))

    @mock.patch('snapcraft.internal.mountinfo.MountInfo.for_root')
    def test_clean_leaves_prime_alone_for_tried(self, mock_for_root):
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)
        lifecycle.clean(project_config.project, parts=None)
        self.assertThat(
            steps.PRIME.name, DirExists(),
            'Expected prime directory to remain after cleaning for tried snap')


class RecordSnapcraftYamlTestCase(BaseLifecycleTestCase):

    def test_prime_without_build_info_does_not_record(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', None))
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)
        for file_name in ('snapcraft.yaml', 'manifest.yaml'):
            self.assertThat(
                os.path.join(steps.PRIME.name, 'snap', file_name),
                Not(FileExists()))

    def test_prime_with_build_info_records_snapcraft_yaml(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """),
            snap_type='type: app')
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            type: app

            parts:
              test-part:
                plugin: nil

            """)

        self.assertThat(
            os.path.join('prime', 'snap', 'snapcraft.yaml'),
            FileContains(expected))


class RecordManifestBaseTestCase(BaseLifecycleTestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft._get_version', return_value='3.0')
        patcher.start()
        self.addCleanup(patcher.stop)

        original_run_output = snapcraft.internal.common.run_output

        def fake_uname(cmd, *args, **kwargs):
            if 'uname' in cmd:
                return 'Linux test uname 4.10 x86_64'
            else:
                return original_run_output(cmd, *args, **kwargs)
        check_output_patcher = mock.patch(
            'snapcraft.internal.common.run_output', side_effect=fake_uname)
        check_output_patcher.start()
        self.addCleanup(check_output_patcher.stop)

        original_check_call = subprocess.check_call

        def _fake_dpkg_deb(command, *args, **kwargs):
            if 'dpkg-deb' not in command:
                return original_check_call(command, *args, **kwargs)

        check_call_patcher = mock.patch(
            'subprocess.check_call', side_effect=_fake_dpkg_deb)
        check_call_patcher.start()
        self.addCleanup(check_call_patcher.stop)

        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)
        self.fake_apt_cache.add_package(
            fixture_setup.FakeAptCachePackage(
                'patchelf', '0.9', installed=True))

        self.fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(self.fake_snapd)
        self.fake_snapd.snaps_result = []

        self.useFixture(FakeOsRelease())


class RecordManifestTestCase(RecordManifestBaseTestCase):

    def test_prime_with_build_info_records_manifest(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_installed_snaps(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_snapd.snaps_result = [
            {'name': 'test-snap-1',
             'revision': 'test-snap-1-revision'},
            {'name': 'test-snap-2',
             'revision': 'test-snap-2-revision'},
        ]

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps:
                - test-snap-1=test-snap-1-revision
                - test-snap-2=test-snap-2-revision
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_installed_packages(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        for name, version in [('test-package1', 'test-version1'),
                              ('test-package2', 'test-version2')]:
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(
                    name, version, installed=True))

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                - test-package1=test-version1
                - test-package2=test-version2
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_stage_packages(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        for name, version in [('test-package1', 'test-version1'),
                              ('test-package2', 'test-version2')]:
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(name, version))

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    stage-packages: [test-package1=test-version1, test-package2]
                """))  # NOQA

        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages:
                - test-package1=test-version1
                - test-package2=test-version2
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_global_build_packages(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        for name, version in [('test-package1', 'test-version1'),
                              ('test-package2', 'test-version2')]:
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(name, version))

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                build-packages: [test-package1=test-version1, test-package2]
                parts:
                  test-part:
                    plugin: nil
                """))

        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            build-packages:
            - test-package1=test-version1
            - test-package2=test-version2
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_source_details(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_apt_cache.add_package(
            fixture_setup.FakeAptCachePackage('git', 'testversion'))

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    source: test-source
                    source-type: git
                    source-commit: test-commit
                """))

        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                source: test-source
                source-branch: ''
                source-checksum: ''
                source-commit: test-commit
                source-tag: ''
                source-type: git
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages:
            - git=testversion
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_build_package_with_any_architecture(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_apt_cache.add_package(fixture_setup.FakeAptCachePackage(
            'test-package', 'test-version'))

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    build-packages: ['test-package:any']
                """))

        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages:
                - test-package:any
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages:
            - test-package=test-version
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_virtual_build_package(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_apt_cache.add_package(
            fixture_setup.FakeAptCachePackage(
                'test-provider-package', 'test-version',
                provides=['test-virtual-package']))

        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    build-packages: ['test-virtual-package']
                """))

        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages:
                - test-virtual-package
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages:
            - test-provider-package=test-version
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('snapcraft.plugins.nil.NilPlugin.get_manifest')
    def test_prime_with_plugin_manifest(self, fake_plugin_manifest):
        fake_plugin_manifest.return_value = {
            'test-plugin-manifest': 'test-value'}
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                test-plugin-manifest: test-value
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_image_info_records_manifest(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        test_image_info = (
            '{"architecture": "test-architecture", '
            '"created_at": "test-created-at", '
            '"fingerprint": "test-fingerprint"}')
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_IMAGE_INFO', test_image_info))
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            image-info:
              architecture: test-architecture
              created_at: test-created-at
              fingerprint: test-fingerprint
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_invalid_image_info_raises_exception(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_IMAGE_INFO', 'not-json'))
        project_config = self.make_snapcraft_project(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        raised = self.assertRaises(
                errors.InvalidContainerImageInfoError,
                lifecycle.execute, steps.PRIME, project_config)
        self.assertThat(raised.image_info, Equals('not-json'))


class RecordManifestWithDeprecatedSnapKeywordTestCase(
        RecordManifestBaseTestCase):

    scenarios = (
        ('using snap keyword', {'keyword': 'snap'}),
        ('using prime keyword', {'keyword': 'prime'})
    )

    def test_prime_step_records_prime_keyword(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        parts = (textwrap.dedent("""\
            parts:
                test-part:
                    plugin: nil
                    {}: [-*]
        """))
        project_config = self.make_snapcraft_project(
            parts.format(self.keyword))
        lifecycle.execute(steps.PRIME, project_config)

        expected = textwrap.dedent("""\
            snapcraft-version: '3.0'
            snapcraft-os-release-id: ubuntu
            snapcraft-os-release-version-id: '16.04'
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages:
                - patchelf=0.9
                installed-snaps: []
                plugin: nil
                prime:
                - -*
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures:
            - {}
            build-packages: []
            build-snaps: []
            """.format(project_config.project.deb_arch))
        self.assertThat(
            os.path.join(steps.PRIME.name, 'snap', 'manifest.yaml'),
            FileContains(expected))


class CoreSetupTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()

        self.core_path = os.path.join(self.path, 'core', 'current')
        patcher = mock.patch('snapcraft.internal.common.get_core_path')
        core_path_mock = patcher.start()
        core_path_mock.return_value = self.core_path
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(snapcraft.ProjectOptions,
                                    'get_core_dynamic_linker')
        get_linker_mock = patcher.start()
        get_linker_mock.return_value = '/lib/ld'
        self.addCleanup(patcher.stop)

        self.tempdir = os.path.join(self.path, 'tmpdir')
        patcher = mock.patch('snapcraft.internal.lifecycle._runner.'
                             'TemporaryDirectory')
        self.tempdir_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # Create a fake sudo that just echos
        bin_override = os.path.join(self.path, 'bin')
        os.mkdir(bin_override)

        fake_sudo_path = os.path.join(bin_override, 'sudo')
        self.witness_path = os.path.join(self.path, 'sudo_witness')
        with open(fake_sudo_path, 'w') as f:
            print('#!/bin/sh', file=f)
            print('echo $@ >> {}'.format(self.witness_path), file=f)
        os.chmod(fake_sudo_path, 0o755)

        self.useFixture(fixtures.EnvironmentVariable(
            'PATH', '{}:{}'.format(bin_override, os.path.expandvars('$PATH'))))

    @mock.patch.object(storeapi.StoreClient, 'download')
    def test_core_setup_with_env_var(self, download_mock):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_SETUP_CORE', '1'))

        project_config = self.make_snapcraft_project(confinement='classic')
        core_snap = self.create_core_snap(project_config.project.deb_arch)
        core_snap_hash = calculate_sha3_384(core_snap)
        download_mock.return_value = core_snap_hash
        self.tempdir_mock.side_effect = self._setup_tempdir_side_effect(
            core_snap)

        lifecycle.execute(steps.PULL, project_config)

        regex = (
            '.*'
            'mkdir -p {}\n'
            'unsquashfs -d {} .*{}\n').format(
                os.path.dirname(self.core_path),
                self.core_path,
                core_snap_hash)
        self.assertThat(
            self.witness_path,
            FileContains(matcher=MatchesRegex(regex, flags=re.DOTALL)))

        download_mock.assert_called_once_with(
            'core', 'stable', os.path.join(self.tempdir, 'core.snap'),
            project_config.project.deb_arch, '')

    @mock.patch.object(storeapi.StoreClient, 'download')
    @mock.patch('snapcraft.internal.common._DOCKERENV_FILE')
    def test_core_setup_if_docker_env(self, dockerenv_fake, download_mock):
        dockerenv_file = os.path.join(self.tempdir, 'dockerenv')
        os.makedirs(self.tempdir)
        open(dockerenv_file, 'w').close()
        dockerenv_fake.return_value = dockerenv_file

        project_config = self.make_snapcraft_project(confinement='classic')
        core_snap = self.create_core_snap(project_config.project.deb_arch)
        core_snap_hash = calculate_sha3_384(core_snap)
        download_mock.return_value = core_snap_hash
        self.tempdir_mock.side_effect = self._setup_tempdir_side_effect(
            core_snap)

        lifecycle.execute(steps.PULL, project_config)

        regex = (
            '.*'
            'mkdir -p {}\n'
            'unsquashfs -d {} .*{}\n').format(
                os.path.dirname(self.core_path),
                self.core_path,
                core_snap_hash)
        self.assertThat(
            self.witness_path,
            FileContains(matcher=MatchesRegex(regex, flags=re.DOTALL)))

        download_mock.assert_called_once_with(
            'core', 'stable', os.path.join(self.tempdir, 'core.snap'),
            project_config.project.deb_arch, '')

    def test_core_setup_skipped_if_not_classic(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_SETUP_CORE', '1'))

        project_config = self.make_snapcraft_project(confinement='strict')
        lifecycle.execute(steps.PULL, project_config)

        self.assertThat(self.witness_path, Not(FileExists()))

    def test_core_setup_skipped_if_core_exists(self):
        os.makedirs(self.core_path)
        open(os.path.join(self.core_path, 'fake-content'), 'w').close()

        project_config = self.make_snapcraft_project(confinement='classic')
        lifecycle.execute(steps.PULL, project_config)

    def make_snapcraft_project(self, *, confinement: str):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.data['confinement'] = confinement
        snapcraft_yaml.update_part(
            'test-part', dict(plugin='nil'))
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)
        return project_loader.load_config(project)

    def _setup_tempdir_side_effect(self, core_snap):
        @contextlib.contextmanager
        def _tempdir():
            os.makedirs(self.tempdir, exist_ok=True)
            shutil.move(core_snap, os.path.join(self.tempdir, 'core.snap'))
            yield self.tempdir

        return _tempdir

    def create_core_snap(self, deb_arch):
        core_path = os.path.join(self.path, 'core')
        snap_yaml_path = os.path.join(core_path, 'meta', 'snap.yaml')
        os.makedirs(os.path.dirname(snap_yaml_path))
        with open(snap_yaml_path, 'w') as f:
            print('name: core', file=f)
            print('version: 1', file=f)
            print('architectures: [{}]'.format(deb_arch), file=f)
            print('summary: summary', file=f)
            print('description: description', file=f)

        return lifecycle.pack(directory=core_path)


class SnapErrorsTestCase(BaseLifecycleTestCase):

    def test_mksquashfs_missing(self):
        with mock.patch('shutil.which') as which_mock:
            which_mock.return_value = None
            raised = self.assertRaises(
                errors.MissingCommandError,
                lifecycle.pack, mock.Mock())
        self.assertThat(str(raised), Contains('mksquashfs'))
