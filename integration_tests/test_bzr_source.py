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


class BzrSourceTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('bzr') is None:
            self.skipTest('bzr is not installed')

    def _init_and_config_bzr(self):
        subprocess.check_call(
            ['bzr', 'init', '.'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.check_call(
            ['bzr', 'whoami', '--branch', '"Example Dev <dev@example.com>"'])

    def _get_bzr_revno(self, path):
        return subprocess.check_output(
            ['bzr', 'revno', '-r', '-1', path],
            universal_newlines=True).strip()

    def test_pull_bzr_head(self):
        project_dir = self.copy_project_to_tmp('bzr-head')
        os.chdir(project_dir)

        self._init_and_config_bzr()
        subprocess.check_call(
                ['bzr', 'commit', '-m', '"1"', '--unchanged'],
                stderr=subprocess.DEVNULL)
        subprocess.check_call(
                ['bzr', 'commit', '-m', '"2"', '--unchanged'],
                stderr=subprocess.DEVNULL)
        # test initial branch
        self.run_snapcraft('pull', project_dir)
        revno = subprocess.check_output(
            ['bzr', 'revno', '-r', '-1', 'parts/bzr/src'],
            universal_newlines=True).strip()
        self.assertEqual('2', revno)
        # test pull doesn't fail
        self.run_snapcraft('pull', project_dir)
        revno = subprocess.check_output(
            ['bzr', 'revno', '-r', '-1', 'parts/bzr/src'],
            universal_newlines=True).strip()
        self.assertEqual('2', revno)

    def test_pull_bzr_tag(self):
        project_dir = self.copy_project_to_tmp('bzr-tag')
        os.chdir(project_dir)

        self._init_and_config_bzr()
        subprocess.check_call(
            ['bzr', 'commit', '-m', '"1"', '--unchanged'],
            stderr=subprocess.DEVNULL)
        subprocess.check_call(
            ['bzr', 'commit', '-m', '"2"', '--unchanged'],
            stderr=subprocess.DEVNULL)
        subprocess.check_call(
            ['bzr', 'tag', '-r', '1', 'initial'],
            stderr=subprocess.DEVNULL)
        # test initial branch
        self.run_snapcraft('pull', project_dir)
        revno = self._get_bzr_revno('parts/bzr/src')
        self.assertEqual('1', revno)
        # test pull doesn't fail
        self.run_snapcraft('pull', project_dir)
        revno = self._get_bzr_revno('parts/bzr/src')
        self.assertEqual('1', revno)

    def test_pull_bzr_commit(self):
        project_dir = self.copy_project_to_tmp('bzr-commit')
        os.chdir(project_dir)

        self._init_and_config_bzr()
        subprocess.check_call(
            ['bzr', 'commit', '-m', '"1"', '--unchanged'],
            stderr=subprocess.DEVNULL)
        subprocess.check_call(
            ['bzr', 'commit', '-m', '"2"', '--unchanged'],
            stderr=subprocess.DEVNULL)
        # test initial branch
        self.run_snapcraft('pull', project_dir)
        revno = self._get_bzr_revno('parts/bzr/src')
        self.assertEqual('1', revno)
        # test pull doesn't fail
        self.run_snapcraft('pull', project_dir)
        revno = self._get_bzr_revno('parts/bzr/src')
        self.assertEqual('1', revno)
