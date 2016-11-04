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

from testtools.matchers import FileExists

import integration_tests


class HgSourceTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('hg') is None:
            self.skipTest('mercurial is not installed')

    def _get_hg_revno(self, path):
        return subprocess.check_output(
            ['hg', 'log', '--cwd', path, '--template',
             '"{desc}"', '-r', '-1'],
            universal_newlines=True).strip()

    def test_pull_hg_head(self):
        project_dir = self.copy_project_to_tmp('hg-head')
        os.chdir(project_dir)

        subprocess.check_call(['hg', 'init', '.'])
        open('1', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', '1', '--user', '"Example Dev"', '-A', '1'])
        open('2', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', '2', '--user', '"Example Dev"', '-A', '2'])

        self.run_snapcraft('pull', project_dir)
        revno = self._get_hg_revno('parts/mercurial/src')
        self.assertEqual('"2"', revno)

        self.run_snapcraft('pull', project_dir)
        revno = self._get_hg_revno('parts/mercurial/src')
        self.assertEqual('"2"', revno)

    def test_pull_hg_tag(self):
        project_dir = self.copy_project_to_tmp('hg-tag')
        os.chdir(project_dir)

        subprocess.check_call(['hg', 'init', '.'])
        open('1', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', '1', '--user', '"Example Dev"', '-A', '1'])
        subprocess.check_call(
            ['hg', 'tag', 'initial', '--user', '"Example Dev"'])
        open('2', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', '2', '--user', '"Example Dev"', '-A', '2'])

        self.run_snapcraft('pull', project_dir)
        revno = subprocess.check_output(
            'ls -1 parts/mercurial/src/ | wc -l ',
            shell=True, universal_newlines=True).strip()
        self.assertEqual('1', revno)

        self.run_snapcraft('pull', project_dir)
        revno = subprocess.check_output(
            'ls -1 parts/mercurial/src/ | wc -l ',
            shell=True, universal_newlines=True).strip()
        self.assertEqual('1', revno)

    def test_pull_hg_commit(self):
        project_dir = self.copy_project_to_tmp('hg-commit')
        os.chdir(project_dir)

        subprocess.check_call(['hg', 'init', '.'])
        open('1', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', '1', '--user', '"Example Dev"', '-A', '1'])
        open('2', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', '2', '--user', '"Example Dev"', '-A', '2'])

        self.run_snapcraft('pull', project_dir)
        revno = subprocess.check_output(
            'ls -1 parts/mercurial/src/ | wc -l ',
            shell=True, universal_newlines=True).strip()
        self.assertEqual('1', revno)

        self.run_snapcraft('pull', project_dir)
        revno = subprocess.check_output(
            'ls -1 parts/mercurial/src/ | wc -l ',
            shell=True, universal_newlines=True).strip()
        self.assertEqual('1', revno)

    def test_pull_hg_branch(self):
        project_dir = self.copy_project_to_tmp('hg-branch')
        os.chdir(project_dir)

        subprocess.check_call(['hg', 'init', '.'])
        subprocess.check_call(
            ['hg', 'branch', 'second'], stdout=subprocess.DEVNULL)
        open('second', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', 'second', '--user',
             '"Example Dev"', '-A', 'second'])
        subprocess.check_call(
            ['hg', 'branch', 'default'], stdout=subprocess.DEVNULL)
        open('default', 'w').close()
        subprocess.check_call(
            ['hg', 'commit', '-m', 'default', '--user',
             '"Example Dev"', '-A', 'default'])

        self.run_snapcraft('pull', project_dir)
        self.assertThat(
            os.path.join('parts', 'mercurial', 'src', 'second'),
            FileExists())

        self.run_snapcraft('pull', project_dir)
        self.assertThat(
            os.path.join('parts', 'mercurial', 'src', 'second'),
            FileExists())
