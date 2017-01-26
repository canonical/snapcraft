# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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

import subprocess
import shutil

import integration_tests


class GitSourceTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('git') is None:
            self.skipTest('git is not installed')

    def _init_and_config_git(self):
        subprocess.check_call(
            ['git', 'init', '.'], stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'config', '--local', 'user.name', '"Example Dev"'])
        subprocess.check_call(
            ['git', 'config', '--local', 'user.email', 'dev@example.com'])

    def _get_git_revno(self, path, revrange='-1'):
        return subprocess.check_output(
            'git -C {} log {} --oneline | cut -d\' \' -f2'.format(
                path, revrange),
            shell=True, universal_newlines=True).strip()

    def test_pull_git_head(self):
        self.copy_project_to_cwd('git-head')

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"2"', revno)

        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"2"', revno)

    def test_pull_git_tag(self):
        self.copy_project_to_cwd('git-tag')

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'tag', 'initial', 'HEAD@{1}'],
            stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"1"', revno)

        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"1"', revno)

    def test_pull_git_commit(self):
        self.copy_project_to_cwd('git-commit')

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)

        # The test uses "HEAD^" so we can only test it once
        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"1"', revno)

    def test_pull_git_branch(self):
        self.copy_project_to_cwd('git-branch')

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'branch', 'second', 'HEAD@{1}'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'checkout', 'second'],
            stderr=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"3"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'checkout', 'master'],
            stderr=subprocess.DEVNULL)

        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src', revrange='-2')
        self.assertEqual('"3"\n"1"', revno)

        self.run_snapcraft('pull')
        revno = self._get_git_revno('parts/git/src', revrange='-2')
        self.assertEqual('"3"\n"1"', revno)

    def test_pull_git_with_depth(self):
        """Regression test for LP: #1627772."""
        self.copy_project_to_cwd('git-depth')

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull')
