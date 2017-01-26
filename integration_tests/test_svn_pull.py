# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Canonical Ltd
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


class SubversionSourceTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('svn') is None:
            self.skipTest('svn is not installed')

    def _init_svn(self):
        subprocess.check_call(
            ['svnadmin', 'create', 'repo'], stdout=subprocess.DEVNULL)

    def test_pull_svn_checkout(self):
        self.copy_project_to_cwd('svn-pull')

        self._init_svn()

        subprocess.check_call(
            ['svn', 'checkout',
             'file:///{}'.format(os.path.join(self.path, 'repo')),
             'local'],
            stdout=subprocess.DEVNULL)

        open(os.path.join('local', 'file'), 'w').close()
        subprocess.check_call(
            ['svn', 'add', 'file'], stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['svn', 'commit', '-m', 'test'],
            stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['svn', 'update'],
            stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['rm', '-rf', 'local/'], stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull')
        part_src_path = os.path.join(self.parts_dir, 'svn', 'src')
        revno = subprocess.check_output(['svnversion', part_src_path]).strip()
        self.assertEqual(b'1', revno)
        self.assertThat(os.path.join(part_src_path, 'file'), FileExists())

    def test_pull_svn_update(self):
        self.copy_project_to_cwd('svn-pull-update')

        self._init_svn()

        subprocess.check_call(
            ['svn', 'checkout',
             'file:///{}'.format(os.path.join(self.path, 'repo')),
             'local'],
            stdout=subprocess.DEVNULL)

        open(os.path.join('local', 'file'), 'w').close()
        subprocess.check_call(
            ['svn', 'add', 'file'], stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['svn', 'commit', '-m', 'test'],
            stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['svn', 'update'],
            stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['rm', '-rf', 'local/'], stdout=subprocess.DEVNULL)

        part_src_path = os.path.join('parts', 'svn', 'src')
        subprocess.check_call(
            ['svn', 'checkout',
             'file:///{}'.format(os.path.join(self.path, 'repo')),
             part_src_path],
            stdout=subprocess.DEVNULL)

        subprocess.check_call(
            ['svn', 'checkout',
             'file:///{}'.format(os.path.join(self.path, 'repo')),
             'local'],
            stdout=subprocess.DEVNULL)
        open(os.path.join('local', 'filetwo'), 'w').close()
        subprocess.check_call(
            ['svn', 'add', 'filetwo'], stdout=subprocess.DEVNULL, cwd='local/')
        subprocess.check_call(
            ['svn', 'commit', '-m', 'testtwo'], stdout=subprocess.DEVNULL,
            cwd='local/')
        subprocess.check_call(
            ['svn', 'update'], stdout=subprocess.DEVNULL,
            cwd='local/')
        subprocess.check_call(
            ['rm', '-rf', 'local/'], stdout=subprocess.DEVNULL)

        self.run_snapcraft('pull')
        revno = subprocess.check_output(['svnversion', part_src_path]).strip()
        self.assertEqual(b'2', revno)
        self.assertThat(os.path.join(part_src_path, 'file'), FileExists())
        self.assertThat(os.path.join(part_src_path, 'filetwo'), FileExists())
