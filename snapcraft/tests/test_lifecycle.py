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

import contextlib
import fileinput
import logging
import os
import re
import shutil
from unittest import mock

import fixtures
from testtools.matchers import (
    FileContains,
    FileExists,
    MatchesRegex,
    Not,
)

import snapcraft
from snapcraft import storeapi
from snapcraft.file_utils import calculate_sha3_384
from snapcraft.internal import pluginhandler, lifecycle
from snapcraft import tests


class ExecutionTestCases(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = snapcraft.ProjectOptions()

    def make_snapcraft_yaml(self, parts, snap_type=''):
        yaml = """name: test
version: 0
summary: test
description: test
confinement: strict
grade: stable
{type}

{parts}
"""

        super().make_snapcraft_yaml(yaml.format(parts=parts, type=snap_type))

    def test__replace_in_parts(self):
        class Options:
            def __init__(self):
                self.source = '$SNAPCRAFT_PART_INSTALL'

        class Code:
            def __init__(self):
                self.options = Options()
                self.installdir = '/tmp'

        class Part:
            def __init__(self):
                self.code = Code()

        part = Part()
        new_part = lifecycle._replace_in_part(part)

        self.assertEqual(part.code.installdir, new_part.code.options.source)

    def test_exception_when_dependency_is_required(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")

        raised = self.assertRaises(
            RuntimeError,
            lifecycle.execute,
            'pull', self.project_options,
            part_names=['part2'])

        self.assertEqual(
            raised.__str__(),
            "Requested 'pull' of 'part2' but there are unsatisfied "
            "prerequisites: 'part1'")

    def test_no_exception_when_dependency_is_required_but_already_staged(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""")

        def _fake_should_step_run(self, step, force=False):
            return self.name != 'part1'

        with mock.patch.object(pluginhandler.PluginHandler,
                               'should_step_run',
                               _fake_should_step_run):
            lifecycle.execute('pull', self.project_options,
                              part_names=['part2'])

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Skipping stage part1 (already ran)\n'
            'Skipping prime part1 (already ran)\n'
            'Preparing to pull part2 \n'
            'Pulling part2 \n',
            self.fake_logger.output)

    def test_dependency_recursed_correctly(self):
        self.make_snapcraft_yaml("""parts:
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
""")

        snap_info = lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': ''
        }
        self.assertEqual(snap_info, expected_snap_info)

        self.assertEqual(
            'Preparing to pull part1 \n'
            'Pulling part1 \n'
            '\'part2\' has prerequisites that need to be staged: part1\n'
            'Preparing to build part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Preparing to pull part2 \n'
            'Pulling part2 \n'
            '\'part3\' has prerequisites that need to be staged: part2\n'
            'Preparing to build part2 \n'
            'Building part2 \n'
            'Staging part2 \n'
            'Preparing to pull part3 \n'
            'Pulling part3 \n',
            self.fake_logger.output)

    def test_os_type_returned_by_lifecycle(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after:
      - part1
""", 'type: os')

        snap_info = lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': 'os'
        }
        self.assertEqual(snap_info, expected_snap_info)

    def test_dirty_prime_reprimes_single_part(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Strip it.
        lifecycle.execute('prime', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if self.name == 'part1' and step == 'prime':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should automatically clean and re-prime if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute('prime', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Skipping prime part2 (already ran)',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Cleaning priming area for part1 (out of date)',
                'Priming part1',
            ],
            part1_output)

    def test_dirty_prime_reprimes_multiple_part(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Strip it.
        lifecycle.execute('prime', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'prime':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should automatically clean and re-prime if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute('prime', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Cleaning priming area for part2 (out of date)',
                'Priming part2',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Cleaning priming area for part1 (out of date)',
                'Priming part1',
            ],
            part1_output)

    def test_dirty_stage_restages_single_part(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Stage it.
        lifecycle.execute('stage', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if self.name == 'part1' and step == 'stage':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute('stage', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping cleaning priming area for part1 (out of date) '
                '(already clean)',
                'Cleaning staging area for part1 (out of date)',
                'Staging part1',
            ],
            part1_output)

    def test_dirty_stage_restages_multiple_parts(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
""")

        # Stage it.
        lifecycle.execute('stage', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'stage':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute('stage', self.project_options)

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        part2_output = [line.strip() for line in output if 'part2' in line]

        self.assertEqual(
            [
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping cleaning priming area for part2 (out of date) '
                '(already clean)',
                'Cleaning staging area for part2 (out of date)',
                'Staging part2',
            ],
            part2_output)

        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping cleaning priming area for part1 (out of date) '
                '(already clean)',
                'Cleaning staging area for part1 (out of date)',
                'Staging part1',
            ],
            part1_output)

    def test_dirty_stage_part_with_built_dependent_raises(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        # Stage dependency
        lifecycle.execute('stage', self.project_options, part_names=['part1'])
        # Build dependent
        lifecycle.execute('build', self.project_options, part_names=['part2'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'stage':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should raise a RuntimeError about the fact that stage is dirty but
        # it has dependents that need it.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            raised = self.assertRaises(
                RuntimeError,
                lifecycle.execute,
                'stage', self.project_options,
                part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        self.assertEqual(
            [
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
            ],
            part1_output)

        self.assertEqual(
            "The 'stage' step for 'part1' needs to be run again, but 'part2' "
            "depends upon it. Please clean the build step of 'part2' first.",
            str(raised))

    def test_dirty_stage_part_with_unbuilt_dependent(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        # Stage dependency (dependent is unbuilt)
        lifecycle.execute('stage', self.project_options, part_names=['part1'])

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'stage':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute('stage', self.project_options,
                              part_names=['part1'])

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Skipping cleaning priming area for part1 (out of date) '
            '(already clean)\n'
            'Cleaning staging area for part1 (out of date)\n'
            'Skipping cleaning priming area for part2 (out of date) '
            '(already clean)\n'
            'Skipping cleaning staging area for part2 (out of date) '
            '(already clean)\n'
            'Staging part1 \n',
            self.fake_logger.output)

    def test_dirty_stage_reprimes(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Strip it.
        lifecycle.execute('prime', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'stage':
                return pluginhandler.DirtyReport({'foo'}, {'bar'})
            return None

        # Should automatically clean and re-stage if that step is dirty
        # for the part.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            lifecycle.execute('prime', self.project_options)

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Cleaning priming area for part1 (out of date)\n'
            'Cleaning staging area for part1 (out of date)\n'
            'Staging part1 \n'
            'Priming part1 \n',
            self.fake_logger.output)

    def test_dirty_build_raises(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Build it.
        lifecycle.execute('build', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'build':
                return pluginhandler.DirtyReport({'foo', 'bar'}, set())
            return None

        # Should catch that the part needs to be rebuilt and raise an error.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            raised = self.assertRaises(
                RuntimeError,
                lifecycle.execute,
                'build', self.project_options)

        self.assertEqual(
            'Skipping pull part1 (already ran)\n',
            self.fake_logger.output)

        self.assertEqual(
            "The 'build' step of 'part1' is out of date:\n\n"
            "The 'bar' and 'foo' part properties appear to have changed.\n\n"
            "In order to continue, please clean that part's 'build' step "
            "by running: snapcraft clean part1 -s build\n",
            str(raised))

    def test_dirty_pull_raises(self):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Pull it.
        lifecycle.execute('pull', self.project_options)

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        def _fake_dirty_report(self, step):
            if step == 'pull':
                return pluginhandler.DirtyReport(set(), {'foo', 'bar'})
            return None

        # Should catch that the part needs to be re-pulled and raise an error.
        with mock.patch.object(pluginhandler.PluginHandler, 'get_dirty_report',
                               _fake_dirty_report):
            raised = self.assertRaises(
                RuntimeError,
                lifecycle.execute,
                'pull', self.project_options)

        self.assertEqual('', self.fake_logger.output)

        self.assertEqual(
            "The 'pull' step of 'part1' is out of date:\n\n"
            "The 'bar' and 'foo' project options appear to have changed.\n\n"
            "In order to continue, please clean that part's 'pull' step "
            "by running: snapcraft clean part1 -s pull\n",
            str(raised))

    @mock.patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    @mock.patch('snapcraft.repo.install_build_packages')
    def test_pull_is_dirty_if_target_arch_changes(
            self, mock_install_build_packages, mock_enable_cross_compilation):
        self.make_snapcraft_yaml("""parts:
  part1:
    plugin: nil
""")

        # Pull it with amd64
        lifecycle.execute('pull', snapcraft.ProjectOptions(
            target_deb_arch='amd64'))

        # Reset logging since we only care about the following
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Pull it again with armhf. Should catch that the part needs to be
        # re-pulled due to the change in target architecture and raise an
        # error.
        raised = self.assertRaises(
            RuntimeError,
            lifecycle.execute,
            'pull', snapcraft.ProjectOptions(
                target_deb_arch='armhf'))

        self.assertEqual("Setting target machine to 'armhf'\n",
                         self.fake_logger.output)

        self.assertEqual(
            "The 'pull' step of 'part1' is out of date:\n\n"
            "The 'deb_arch' project option appears to have changed.\n\n"
            "In order to continue, please clean that part's 'pull' step "
            "by running: snapcraft clean part1 -s pull\n",
            str(raised))


class CoreSetupTestCase(tests.TestCase):

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
        patcher = mock.patch('snapcraft.internal.lifecycle.TemporaryDirectory')
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
            'SNAPCRAFT_SETUP_CORE', '1'))
        self.useFixture(fixtures.EnvironmentVariable(
            'PATH', '{}:{}'.format(bin_override, os.path.expandvars('$PATH'))))

        self.project_options = snapcraft.ProjectOptions()

    @mock.patch.object(storeapi.StoreClient, 'download')
    def test_core_setup(self, download_mock):
        core_snap = self._create_core_snap()
        core_snap_hash = calculate_sha3_384(core_snap)
        download_mock.return_value = core_snap_hash
        self.tempdir_mock.side_effect = self._setup_tempdir_side_effect(
            core_snap)

        self._create_classic_confined_snapcraft_yaml()
        lifecycle.execute('pull', self.project_options)

        regex = (
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
            self.project_options.deb_arch, '')

    def test_core_setup_skipped_if_not_classic(self):
        lifecycle.init()
        lifecycle.execute('pull', self.project_options)

        self.assertThat(self.witness_path, Not(FileExists()))

    def test_core_setup_skipped_if_core_exists(self):
        os.makedirs(self.core_path)
        open(os.path.join(self.core_path, 'fake-content'), 'w').close()

        self._create_classic_confined_snapcraft_yaml()
        lifecycle.execute('pull', self.project_options)

        self.assertThat(self.witness_path, Not(FileExists()))

    def _create_classic_confined_snapcraft_yaml(self):
        snapcraft_yaml_path = lifecycle.init()
        # convert snapcraft.yaml into a classic confined snap
        with fileinput.FileInput(snapcraft_yaml_path, inplace=True) as f:
            for line in f:
                print(line.replace('confinement: devmode',
                                   'confinement: classic'),
                      end='')

    def _setup_tempdir_side_effect(self, core_snap):
        @contextlib.contextmanager
        def _tempdir():
            os.makedirs(self.tempdir, exist_ok=True)
            shutil.move(core_snap, os.path.join(self.tempdir, 'core.snap'))
            yield self.tempdir

        return _tempdir

    def _create_core_snap(self):
        core_path = os.path.join(self.path, 'core')
        snap_yaml_path = os.path.join(core_path, 'meta', 'snap.yaml')
        os.makedirs(os.path.dirname(snap_yaml_path))
        with open(snap_yaml_path, 'w') as f:
            print('name: core', file=f)
            print('version: 1', file=f)
            print('architectures: [{}]'.format(
                self.project_options.deb_arch), file=f)
            print('summary: summary', file=f)
            print('description: description', file=f)

        return lifecycle.snap(self.project_options, directory=core_path)
