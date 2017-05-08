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

from collections import namedtuple
import os
import yaml

import testscenarios
from testtools.matchers import (
    Equals,
    FileExists
)

import integration_tests
from snapcraft.tests import fixture_setup


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
        self.assertIn('build-packages', state.assets)
        self.assertTrue('stage-packages' in state.properties)
        self.assertEqual('bar', state.properties['foo'])
        self.assertEqual(['curl'], state.properties['stage-packages'])


class AssetTrackingTestCase(integration_tests.TestCase):

    def test_pull(self):
        self.copy_project_to_cwd('asset-tracking')
        stage_version = self.set_stage_package_version(
            'snapcraft.yaml', part='asset-tracking', package='hello')
        build_version = self.set_build_package_version(
            'snapcraft.yaml', part='asset-tracking', package='hello')

        self.run_snapcraft('pull')

        state_file = os.path.join(
            self.parts_dir, 'asset-tracking', 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        # Verify that the correct version of 'hello' is installed
        self.assertTrue(len(state.assets['stage-packages']) > 0)
        self.assertTrue(len(state.assets['build-packages']) > 0)
        self.assertIn(
            'hello={}'.format(stage_version), state.assets['stage-packages'])
        self.assertIn(
            'hello={}'.format(build_version), state.assets['build-packages'])
        self.assertIn('source-details', state.assets)

    def test_pull_global_build_packages_are_excluded(self):
        """
        Ensure global build-packages are not included in each part's
        build-packages data.
        """
        self.copy_project_to_cwd('build-package-global')
        version = self.set_build_package_version(
            os.path.join('snap', 'snapcraft.yaml'),
            part=None, package='hello')
        self.run_snapcraft('pull')

        state_file = os.path.join(
            self.parts_dir, 'empty-part', 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertTrue(len(state.assets['build-packages']) == 0)
        self.assertNotIn(
            'hello={}'.format(version), state.assets['build-packages'])


TestDetail = namedtuple('TestDetail', ['field', 'value'])


class GitAssetTrackingTestCase(testscenarios.WithScenarios,
                               integration_tests.TestCase):

    scenarios = [
        ('plain', {
            'part_name': 'git-part',
            'expected_details': None,
        }),
        ('branch', {
            'part_name': 'git-part-branch',
            'expected_details': TestDetail('branch', 'test-branch'),
        }),
        ('tag', {
            'part_name': 'git-part-tag',
            'expected_details': TestDetail('tag', 'feature-tag'),
        }),
    ]

    def test_pull_git(self):
        repo_fixture = fixture_setup.GitRepo()
        self.useFixture(repo_fixture)
        project_dir = 'asset-tracking-git'

        self.run_snapcraft(['pull', self.part_name], project_dir)

        state_file = os.path.join(
            self.parts_dir, self.part_name, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)

        # fall back to the commit if no other source option is provided
        # snapcraft.source.Git doesn't allow both a tag and a commit
        if self.expected_details:
            self.assertThat(
                state.assets['source-details'][self.expected_details.field],
                Equals(self.expected_details.value))
        else:
            self.assertThat(
                state.assets['source-details']['commit'],
                Equals(repo_fixture.commit))


class BazaarAssetTrackingTestCase(testscenarios.WithScenarios,
                                  integration_tests.TestCase):
    scenarios = [
        ('plain', {
            'part_name': 'bzr-part',
            'expected_details': None,
        }),
        ('tag', {
            'part_name': 'bzr-part-tag',
            'expected_details': TestDetail('tag', 'feature-tag'),
        }),
    ]

    def test_pull_bzr(self):
        repo_fixture = fixture_setup.BzrRepo('bzr-source')
        self.useFixture(repo_fixture)
        project_dir = 'asset-tracking-bzr'
        part = self.part_name
        self.run_snapcraft(['pull', part], project_dir)

        state_file = os.path.join(
            self.parts_dir, part, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)

        if self.expected_details:
            self.assertThat(
                state.assets['source-details'][self.expected_details.field],
                Equals(self.expected_details.value))
        else:
            self.assertThat(
                state.assets['source-details']['commit'],
                Equals(repo_fixture.commit))


class MercurialAssetTrackingTestCase(testscenarios.WithScenarios,
                                     integration_tests.TestCase):
    scenarios = [
        ('plain', {
            'part_name': 'hg-part',
            'expected_details': None,
        }),
        ('tag', {
            'part_name': 'hg-part-tag',
            'expected_details': TestDetail('tag', 'feature-tag'),
        }),
    ]

    def test_pull_hg(self):
        repo_fixture = fixture_setup.HgRepo('hg-source')
        self.useFixture(repo_fixture)
        project_dir = 'asset-tracking-hg'
        part = self.part_name
        self.run_snapcraft(['pull', part], project_dir)

        state_file = os.path.join(
            self.parts_dir, part, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)

        if self.expected_details:
            self.assertThat(
                state.assets['source-details'][self.expected_details.field],
                Equals(self.expected_details.value))
        else:
            self.assertThat(
                state.assets['source-details']['commit'],
                Equals(repo_fixture.commit))


class SubversionAssetTrackingTestCase(integration_tests.TestCase):

    def test_pull_svn(self):
        repo_fixture = fixture_setup.SvnRepo('svn-source')
        self.useFixture(repo_fixture)
        project_dir = 'asset-tracking-svn'
        part = 'svn-part'
        expected_commit = repo_fixture.commit
        self.run_snapcraft(['pull', part], project_dir)

        state_file = os.path.join(
            self.parts_dir, part, 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        self.assertIn('source-details', state.assets)
        self.assertEqual(expected_commit,
                         state.assets['source-details']['commit'])
