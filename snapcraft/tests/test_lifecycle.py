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
import subprocess
import sys
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
from snapcraft import storeapi
from snapcraft.file_utils import calculate_sha3_384
from snapcraft.internal import errors, pluginhandler, lifecycle
from snapcraft import tests
from snapcraft.tests import fixture_setup


class BaseLifecycleTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = snapcraft.ProjectOptions()

    def make_snapcraft_yaml(self, parts, snap_type=''):
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

        super().make_snapcraft_yaml(yaml.format(parts=parts, type=snap_type))


class ExecutionTestCase(BaseLifecycleTestCase):

    def test__replace_in_parts(self):
        class Options:
            def __init__(self):
                self.source = '$SNAPCRAFT_PART_INSTALL'

        class Plugin:
            def __init__(self):
                self.options = Options()
                self.installdir = '/tmp'

        class Part:
            def __init__(self):
                self.plugin = Plugin()

        part = Part()
        new_part = lifecycle._replace_in_part(part)

        self.assertThat(
            new_part.plugin.options.source, Equals(part.plugin.installdir))

    def test_exception_when_dependency_is_required(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after:
                      - part1
                """))

        raised = self.assertRaises(
            RuntimeError,
            lifecycle.execute,
            'pull', self.project_options,
            part_names=['part2'])

        self.assertThat(
            raised.__str__(),
            Equals("Requested 'pull' of 'part2' but there are unsatisfied "
                   "prerequisites: 'part1'"))

    def test_no_exception_when_dependency_is_required_but_already_staged(self):
        self.make_snapcraft_yaml(
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
            lifecycle.execute('pull', self.project_options,
                              part_names=['part2'])

        self.assertThat(
            self.fake_logger.output,
            Equals('Skipping pull part1 (already ran)\n'
                   'Skipping build part1 (already ran)\n'
                   'Skipping stage part1 (already ran)\n'
                   'Skipping prime part1 (already ran)\n'
                   'Preparing to pull part2 \n'
                   'Pulling part2 \n'))

    def test_dependency_recursed_correctly(self):
        self.make_snapcraft_yaml(
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

        snap_info = lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': ''
        }
        self.assertThat(snap_info, Equals(expected_snap_info))

        self.assertThat(
            self.fake_logger.output, Equals(
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
            ))

    def test_os_type_returned_by_lifecycle(self):
        self.make_snapcraft_yaml(
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

        snap_info = lifecycle.execute('pull', self.project_options)

        expected_snap_info = {
            'name': 'test',
            'version': 0,
            'arch': [self.project_options.deb_arch],
            'type': 'os'
        }
        self.assertThat(snap_info, Equals(expected_snap_info))

    def test_dirty_prime_reprimes_single_part(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

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
                'Cleaning priming area for part1 (out of date)',
                'Priming part1',
            ]))

    def test_dirty_prime_reprimes_multiple_part(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

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

        self.assertThat(
            part2_output,
            Equals([
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping stage part2 (already ran)',
                'Cleaning priming area for part2 (out of date)',
                'Priming part2',
            ]))

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping stage part1 (already ran)',
                'Cleaning priming area for part1 (out of date)',
                'Priming part1',
            ]))

    def test_dirty_stage_restages_single_part(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

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
                'Skipping cleaning priming area for part1 (out of date) '
                '(already clean)',
                'Cleaning staging area for part1 (out of date)',
                'Staging part1',
            ]))

    def test_dirty_stage_restages_multiple_parts(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """))

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

        self.assertThat(
            part2_output,
            Equals([
                'Skipping pull part2 (already ran)',
                'Skipping build part2 (already ran)',
                'Skipping cleaning priming area for part2 (out of date) '
                '(already clean)',
                'Cleaning staging area for part2 (out of date)',
                'Staging part2',
            ]))

        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
                'Skipping cleaning priming area for part1 (out of date) '
                '(already clean)',
                'Cleaning staging area for part1 (out of date)',
                'Staging part1',
            ]))

    def test_dirty_stage_part_with_built_dependent_raises(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after: [part1]
                """))

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
                errors.StepOutdatedError,
                lifecycle.execute,
                'stage', self.project_options,
                part_names=['part1'])

        output = self.fake_logger.output.split('\n')
        part1_output = [line.strip() for line in output if 'part1' in line]
        self.assertThat(
            part1_output,
            Equals([
                'Skipping pull part1 (already ran)',
                'Skipping build part1 (already ran)',
            ]))

        self.assertThat(
            str(raised), Equals(
                "The 'stage' step of 'part1' is out of date:\n"
                "The 'stage' step for 'part1' needs to be run again, but "
                "'part2' depends on it.\n"
                "In order to continue, please clean that part's 'stage' step "
                "by running:\nsnapcraft clean part2 -s stage\n"))

    def test_dirty_stage_part_with_unbuilt_dependent(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                    after: [part1]
                """))

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

        self.assertThat(
            self.fake_logger.output, Equals(
                'Skipping pull part1 (already ran)\n'
                'Skipping build part1 (already ran)\n'
                'Skipping cleaning priming area for part1 (out of date) '
                '(already clean)\n'
                'Cleaning staging area for part1 (out of date)\n'
                'Skipping cleaning priming area for part2 (out of date) '
                '(already clean)\n'
                'Skipping cleaning staging area for part2 (out of date) '
                '(already clean)\n'
                'Staging part1 \n'))

    def test_dirty_stage_reprimes(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

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

        self.assertThat(
            self.fake_logger.output, Equals(
                'Skipping pull part1 (already ran)\n'
                'Skipping build part1 (already ran)\n'
                'Cleaning priming area for part1 (out of date)\n'
                'Cleaning staging area for part1 (out of date)\n'
                'Staging part1 \n'
                'Priming part1 \n'))

    def test_dirty_build_raises(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

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
                errors.StepOutdatedError,
                lifecycle.execute,
                'build', self.project_options)

        self.assertThat(
            self.fake_logger.output,
            Equals('Skipping pull part1 (already ran)\n'))

        self.assertThat(
            str(raised), Equals(
                "The 'build' step of 'part1' is out of date:\n"
                "The 'bar' and 'foo' part properties appear to have changed.\n"
                "In order to continue, please clean that part's 'build' step "
                "by running:\nsnapcraft clean part1 -s build\n"))

    def test_dirty_pull_raises(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

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
                errors.StepOutdatedError,
                lifecycle.execute,
                'pull', self.project_options)

        self.assertThat(self.fake_logger.output, Equals(''))

        self.assertThat(
            str(raised), Equals(
                "The 'pull' step of 'part1' is out of date:\n"
                "The 'bar' and 'foo' project options appear to have changed.\n"
                "In order to continue, please clean that part's 'pull' step "
                "by running:\nsnapcraft clean part1 -s pull\n"))

    @mock.patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    @mock.patch('snapcraft.repo.Repo.install_build_packages')
    def test_pull_is_dirty_if_target_arch_changes(
            self, mock_install_build_packages, mock_enable_cross_compilation):
        mock_install_build_packages.return_value = []
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  part1:
                    plugin: nil
                """))

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
            errors.StepOutdatedError,
            lifecycle.execute,
            'pull', snapcraft.ProjectOptions(
                target_deb_arch='armhf'))

        self.assertThat(
            self.fake_logger.output,
            Equals("Setting target machine to 'armhf'\n"))

        self.assertThat(
            str(raised), Equals(
                "The 'pull' step of 'part1' is out of date:\n"
                "The 'deb_arch' project option appears to have changed.\n"
                "In order to continue, please clean that part's 'pull' step "
                "by running:\nsnapcraft clean part1 -s pull\n"))

    def test_prime_excludes_internal_snapcraft_dir(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('prime', self.project_options)
        self.assertThat(
            os.path.join('prime', 'snap', '.snapcraft'),
            Not(DirExists()))


class CleanTestCase(BaseLifecycleTestCase):

    def test_clean_removes_global_state(self):
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('pull', self.project_options)
        lifecycle.clean(self.project_options, parts=None)
        self.assertThat(
            os.path.join('snap', '.snapcraft'),
            Not(DirExists()))


class RecordSnapcraftYamlTestCase(BaseLifecycleTestCase):

    def test_prime_without_build_info_does_not_record(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', None))
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('prime', self.project_options)
        for file_name in ('snapcraft.yaml', 'manifest.yaml'):
            self.assertThat(
                os.path.join('prime', 'snap', file_name),
                Not(FileExists()))

    def test_prime_with_build_info_records_snapcraft_yaml(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """),
            snap_type='type: app')
        lifecycle.execute('prime', self.project_options)

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
        original_check_output = subprocess.check_output

        def fake_uname(cmd, *args, **kwargs):
            if 'uname' in cmd:
                return 'Linux test uname 4.10 x86_64'.encode(
                    sys.getfilesystemencoding())
            else:
                return original_check_output(cmd, *args, **kwargs)
        check_output_patcher = mock.patch(
            'subprocess.check_output', side_effect=fake_uname)
        check_output_patcher.start()
        self.addCleanup(check_output_patcher.stop)

        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)

        self.fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(self.fake_snapd)
        self.fake_snapd.snaps_result = []


class RecordManifestTestCase(RecordManifestBaseTestCase):

    def test_prime_with_build_info_records_manifest(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: []
            build-snaps: []
            """.format(self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
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

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: []
                installed-snaps: {}
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: []
            build-snaps: []
            """.format('[test-snap-1=test-snap-1-revision, '
                       'test-snap-2=test-snap-2-revision]',
                       self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_installed_packages(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        for name, version in [('test-package1', 'test-version1'),
                              ('test-package2', 'test-version2')]:
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(
                    name, version, installed=True))

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: {}
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: []
            build-snaps: []
            """.format('[test-package1=test-version1, '
                       'test-package2=test-version2]',
                       self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    def test_prime_with_stage_packages(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        for name, version in [('test-package1', 'test-version1'),
                              ('test-package2', 'test-version2')]:
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(name, version))

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    stage-packages: [test-package1=test-version1, test-package2]
                """))  # NOQA

        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: [test-package1=test-version1, test-package2=test-version2]
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: []
            build-snaps: []
            """.format(self.project_options.deb_arch))  # NOQA
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_global_build_packages(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        for name, version in [('test-package1', 'test-version1'),
                              ('test-package2', 'test-version2')]:
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(name, version))

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                build-packages: [test-package1=test-version1, test-package2]
                parts:
                  test-part:
                    plugin: nil
                """))

        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            build-packages: [test-package1=test-version1, test-package2=test-version2]
            parts:
              test-part:
                build-packages: []
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-snaps: []
            """.format(self.project_options.deb_arch))  # NOQA
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_source_details(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_apt_cache.add_package(
            fixture_setup.FakeAptCachePackage('git', 'testversion'))

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    source: test-source
                    source-type: git
                    source-commit: test-commit
                """))

        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: []
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
            architectures: [{}]
            build-packages: [git=testversion]
            build-snaps: []
            """.format(self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_build_package_with_any_architecture(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_apt_cache.add_package(fixture_setup.FakeAptCachePackage(
            'test-package', 'test-version'))

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    build-packages: ['test-package:any']
                """))

        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: ['test-package:any']
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: [test-package=test-version]
            build-snaps: []
            """.format(self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('subprocess.check_call')
    def test_prime_with_virtual_build_package(self, _):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.fake_apt_cache.add_package(
            fixture_setup.FakeAptCachePackage(
                'test-provider-package', 'test-version',
                provides=['test-virtual-package']))

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                    build-packages: ['test-virtual-package']
                """))

        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: [test-virtual-package]
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: [test-provider-package=test-version]
            build-snaps: []
            """.format(self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))

    @mock.patch('snapcraft.plugins.nil.NilPlugin.get_manifest')
    def test_prime_with_plugin_manifest(self, fake_plugin_manifest):
        fake_plugin_manifest.return_value = {
            'test-plugin-manifest': 'test-value'}
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  test-part:
                    plugin: nil
                """))
        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: []
                stage: []
                stage-packages: []
                test-plugin-manifest: test-value
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: []
            build-snaps: []
            """.format(self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))


class RecordManifestWithDeprecatedSnapKeywordTestCase(
        RecordManifestBaseTestCase):

    scenarios = (
        ('using snap keyword', {'keyword': 'snap'}),
        ('using prime keyword', {'keyword': 'prime'})
    )

    def test_prime_step_records_prime_keyword(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        parts = ("""parts:
  test-part:
    plugin: nil
    {}: [-*]
""")
        self.make_snapcraft_yaml(parts.format(self.keyword))
        lifecycle.execute('prime', self.project_options)

        expected = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              test-part:
                build-packages: []
                installed-packages: []
                installed-snaps: []
                plugin: nil
                prime: [-*]
                stage: []
                stage-packages: []
                uname: Linux test uname 4.10 x86_64
            architectures: [{}]
            build-packages: []
            build-snaps: []
            """.format(self.project_options.deb_arch))
        self.assertThat(
            os.path.join('prime', 'snap', 'manifest.yaml'),
            FileContains(expected))


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


class SnapErrorsTestCase(BaseLifecycleTestCase):

    def test_mksquashfs_missing(self):
        with mock.patch('shutil.which') as which_mock:
            which_mock.return_value = None
            raised = self.assertRaises(
                errors.MissingCommandError,
                lifecycle.pack, self.project_options)
        self.assertThat(str(raised), Contains('mksquashfs'))
