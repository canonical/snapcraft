# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import yaml

from testtools.matchers import FileExists

import integration_tests


class PullPropertiesTestCase(integration_tests.TestCase):

    def test_pull(self):
        self.assert_expected_pull_state('local-plugin-pull-properties')

    def test_pull_legacy_pull_properties(self):
        self.assert_expected_pull_state('local-plugin-legacy-pull-properties')

    def assert_expected_pull_state(self, project_dir):
        self.run_snapcraft('pull', project_dir)

        state_file = os.path.join(
            self.parts_dir, 'x-local-plugin', 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        # Verify that the correct schema dependencies made it into the state.
        self.assertTrue('foo' in state.schema_properties)
        self.assertTrue('stage-packages' in state.schema_properties)

        # Verify that the contents of the dependencies made it in as well.
        self.assertTrue('foo' in state.properties)
        self.assertTrue(len(state.assets['stage-packages']) > 0)
        self.assertTrue('stage-packages' in state.properties)
        self.assertEqual('bar', state.properties['foo'])
        self.assertEqual(['curl'], state.properties['stage-packages'])


class AssetTrackingTestCase(integration_tests.TestCase):

    def _create_git_repo(self, name):
        def _call(cmd):
            subprocess.check_call(cmd, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)

        def _call_with_output(cmd):
            return subprocess.check_output(cmd).decode('utf-8').strip()

        os.makedirs(name)
        _call(['git', '-C', name, 'init'])
        _call(['git', '-C', name, 'config',
               '--local', 'user.name', 'Test User'])
        _call(['git', '-C', name, 'config',
               '--local', 'user.email', 'testuser@example.com'])
        with open(os.path.join(name, 'testing'), 'w') as fp:
            fp.write('testing')

        _call(['git', '-C', name, 'add', 'testing'])
        _call(['git', '-C', name, 'commit', '-am', 'testing'])
        _call(['git', '-C', name, 'branch', 'feature'])
        _call(['git', '-C', name, 'tag', 'feature-tag'])
        return _call_with_output(['git', '-C', name, 'rev-parse', 'HEAD'])

    def test_pull(self):
        project_dir = 'asset-tracking'
        self.run_snapcraft(['pull', 'asset-tracking'], project_dir)

        state_file = os.path.join(
            self.parts_dir, 'asset-tracking', 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        # Verify that the correct version of 'hello' is installed
        self.assertTrue(len(state.assets['stage-packages']) > 0)
        self.assertIn('hello=2.10-1', state.assets['stage-packages'])

        self.assertIn('source-details', state.assets)

    def test_pull_git(self):
        project_dir = 'asset-tracking'
        part = 'git-part'
        expected_commit = self._create_git_repo('git-source')
        self.run_snapcraft(['pull', part], project_dir)

        state_file = os.path.join(
            self.parts_dir, part, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)
        self.assertEqual(expected_commit,
                         state.assets['source-details']['commit'])

    def test_pull_git_branch(self):
        project_dir = 'asset-tracking'
        part = 'git-part-branch'
        expected_commit = self._create_git_repo('git-source')
        self.run_snapcraft(['pull', part], project_dir)

        state_file = os.path.join(
            self.parts_dir, part, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)
        self.assertEqual(expected_commit,
                         state.assets['source-details']['commit'])
        self.assertEqual('feature', state.assets['source-details']['branch'])

    def test_pull_git_tag(self):
        project_dir = 'asset-tracking'
        part = 'git-part-tag'
        self._create_git_repo('git-source')
        self.run_snapcraft(['pull', part], project_dir)

        state_file = os.path.join(
            self.parts_dir, part, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)
        self.assertEqual('feature-tag', state.assets['source-details']['tag'])
