# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


class SubversionSourceTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('svn') is None:
            self.skipTest('svn is not installed')

    def _init_svn(self):
        subprocess.check_call(
            ['svnadmin', 'create', 'repo'], stdout=subprocess.DEVNULL)

    def test_pull_svn(self):
        self._init_svn()

        subprocess.check_call(
            ['svn', 'checkout', 'file:///$(pwd)/repo', 'local'],
            stdout=subprocess.DEVNULL)

    def test_commit_svn(self):
        self.test_pull_svn()

        os.chdir("local")

        subprocess.check_call(
            ['touch', 'test_file'],stdout=subprocess.DEVNULL)

        subprocess.check_call(
            ['svn', 'add', 'test_file'],stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['svn', 'commit', '-m', 'test'],stdout=subprocess.DEVNULL)

        os.chdir("..")

    def test_update_svn(self):
        self.test_commit_svn()

        subprocess.check_call(
            ['svn', 'checkout', 'file:///$(pwd)/repo', 'testupdate'],
            stdout=subprocess.DEVNULL)

        subprocess.check_call(
            ['cd', 'local'],stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['echo', 'test', '>', 'test_file'],stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['svn', 'commit', '-m', 'test2'],stdout=subprocess.DEVNULL)

        os.chdir("../testupdate")

        subprocess.check_call(
            ['svn', 'update', '.'],stdout=subprocess.DEVNULL)
