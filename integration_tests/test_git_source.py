# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
        project_dir = self.copy_project_to_tmp('git-head')
        os.chdir(project_dir)

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"2"', revno)

        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"2"', revno)

    def test_pull_git_tag(self):
        project_dir = self.copy_project_to_tmp('git-tag')
        os.chdir(project_dir)

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

        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"1"', revno)

        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"1"', revno)

    def test_pull_git_commit(self):
        project_dir = self.copy_project_to_tmp('git-commit')
        os.chdir(project_dir)

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)

        # The test uses "HEAD^" so we can only test it once
        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src')
        self.assertEqual('"1"', revno)

    def test_pull_git_branch(self):
        project_dir = self.copy_project_to_tmp('git-branch')
        os.chdir(project_dir)

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

        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src', revrange='-2')
        self.assertEqual('"3"\n"1"', revno)

        self.run_snapcraft('pull', project_dir)
        revno = self._get_git_revno('parts/git/src', revrange='-2')
        self.assertEqual('"3"\n"1"', revno)

    def test_pull_git_with_depth(self):
        """Regression test for LP: #1627772."""
        project_dir = self.copy_project_to_tmp('git-depth')
        os.chdir(project_dir)

        self._init_and_config_git()
        subprocess.check_call(
            ['git', 'commit', '-m', '"1"', '--allow-empty'],
            stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'commit', '-m', '"2"', '--allow-empty'],
            stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull', project_dir)
