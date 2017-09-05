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
import shutil
import subprocess
from textwrap import dedent

from testtools.matchers import (
    Contains,
    Equals,
    FileExists
)

import integration_tests


class DebSourceTestCase(integration_tests.TestCase):

    def test_stage_deb(self):
        self.copy_project_to_cwd('deb-hello')
        self.run_snapcraft(['stage', 'deb'])

        self.assertThat(
            os.path.join(self.stage_dir, 'bin', 'hello'),
            FileExists())
        self.assertThat(
            os.path.join(self.stage_dir, 'usr', 'bin', 'world'),
            FileExists())

    # Regression test for LP: #1634813
    def test_stage_deb_with_symlink(self):
        self.copy_project_to_cwd('deb-with-symlink')
        self.run_snapcraft(['stage', 'deb-with-symlink'])

        target = os.path.join(self.stage_dir, 'target')
        symlink = os.path.join(self.stage_dir, 'symlink')
        self.assertThat(target, FileExists())
        self.assertThat(symlink, FileExists())
        self.assertTrue(os.path.islink(symlink))
        self.assertThat(os.readlink(symlink), Equals('target'))


class DebGenerateVersionTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('dch') is None:
            self.skipTest('dch is not installed')
        os.mkdir('snap')

        with open(os.path.join('snap', 'snapcraft.yaml'), 'w') as f:
            print(dedent("""\
                name: deb-test
                version: deb
                summary: test deb generated version
                description: test deb generated version
                architectures: [amd64]
                parts:
                    nil:
                        plugin: nil
                """), file=f)

    def mkchangelog(self):
        os.mkdir('debian')
        subprocess.check_call(
            ['dch', '--create', '--package', 'foobar', '-v', '2.0', '--empty'],
            subprocess.DEVNULL)
        # fake debian/rules to keep fdr clean happy
        fname = 'debian/rules'
        fhandle = open(fname, 'a')
        try:
            os.utime(fname, None)
            os.chmod(fname, 0o777)
        finally:
            fhandle.close()

    def test_version(self):
        self.mkchangelog()
        self.run_snapcraft('snap')
        self.assertThat('deb-test_2.0_amd64.snap', FileExists())

    # this is a catch-all test for all the catastrofic situations that could
    # happen if the debian dir (or its content) was missing or unusable, and
    # as such 'fdr clean' would fail
    def test_fdrclean_failure(self):
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ['snap'])
        self.assertThat(
            exception.output,
            Contains('fakeroot debian/rules clean\' returned non-zero')
        )

    def test_no_changelog(self):
        self.mkchangelog()
        path = os.path.join('debian', 'changelog')
        os.remove(path)
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ['snap'])
        self.assertThat(
            exception.output,
            Contains('No such file or directory: ')
        )

    def test_malformed_changelog(self):
        self.mkchangelog()
        path = os.path.join('debian', 'changelog')
        with open(path, 'w+') as f:
            f.write(str(os.urandom(512)))
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ['snap'])
        self.assertThat(
            exception.output,
            Contains('{} wasn\'t properly closed'.format(path))
        )
