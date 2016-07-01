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

import fixtures
import logging
import os
import stat
import tempfile
import unittest.mock

import snapcraft
from snapcraft import repo
from snapcraft import tests


class UbuntuTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        tempdirObj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdirObj.cleanup)
        self.tempdir = tempdirObj.name

    @unittest.mock.patch('snapcraft.repo._get_geoip_country_code_prefix')
    def test_sources_is_none_uses_default(self, mock_cc):
        project_options = snapcraft.ProjectOptions(use_geoip=True)
        mock_cc.return_value = 'ar'

        self.maxDiff = None
        sources_list = repo._format_sources_list('', project_options)

        expected_sources_list = \
            '''deb http://ar.archive.ubuntu.com/ubuntu/ xenial main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ xenial universe
deb http://ar.archive.ubuntu.com/ubuntu/ xenial-updates universe
deb http://ar.archive.ubuntu.com/ubuntu/ xenial multiverse
deb http://ar.archive.ubuntu.com/ubuntu/ xenial-updates multiverse
deb http://security.ubuntu.com/ubuntu xenial-security main restricted
deb http://security.ubuntu.com/ubuntu xenial-security universe
deb http://security.ubuntu.com/ubuntu xenial-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)

    def test_no_geoip_uses_default_archive(self):
        sources_list = repo._format_sources_list(
            repo._DEFAULT_SOURCES, snapcraft.ProjectOptions())

        expected_sources_list = \
            '''deb http://archive.ubuntu.com/ubuntu/ xenial main restricted
deb http://archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb http://archive.ubuntu.com/ubuntu/ xenial universe
deb http://archive.ubuntu.com/ubuntu/ xenial-updates universe
deb http://archive.ubuntu.com/ubuntu/ xenial multiverse
deb http://archive.ubuntu.com/ubuntu/ xenial-updates multiverse
deb http://security.ubuntu.com/ubuntu xenial-security main restricted
deb http://security.ubuntu.com/ubuntu xenial-security universe
deb http://security.ubuntu.com/ubuntu xenial-security multiverse
'''

        self.assertEqual(sources_list, expected_sources_list)

    @unittest.mock.patch('snapcraft.repo._get_geoip_country_code_prefix')
    def test_sources_amd64_vivid(self, mock_cc):
        project_options = snapcraft.ProjectOptions(use_geoip=True)
        self.maxDiff = None
        mock_cc.return_value = 'ar'

        sources_list = repo._format_sources_list(
            repo._DEFAULT_SOURCES, project_options, 'vivid')

        expected_sources_list = \
            '''deb http://ar.archive.ubuntu.com/ubuntu/ vivid main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ vivid universe
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates universe
deb http://ar.archive.ubuntu.com/ubuntu/ vivid multiverse
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates multiverse
deb http://security.ubuntu.com/ubuntu vivid-security main restricted
deb http://security.ubuntu.com/ubuntu vivid-security universe
deb http://security.ubuntu.com/ubuntu vivid-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)

    @unittest.mock.patch('snapcraft.repo._get_geoip_country_code_prefix')
    def test_sources_armhf_trusty(self, mock_cc):
        project_options = snapcraft.ProjectOptions(
            use_geoip=True, target_deb_arch='armhf')
        sources_list = repo._format_sources_list(
            repo._DEFAULT_SOURCES, project_options, 'trusty')

        expected_sources_list = \
            '''deb http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty multiverse
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates multiverse
deb http://ports.ubuntu.com/ubuntu-ports trusty-security main restricted
deb http://ports.ubuntu.com/ubuntu-ports trusty-security universe
deb http://ports.ubuntu.com/ubuntu-ports trusty-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)
        self.assertFalse(mock_cc.called)

    def test_fix_symlinks(self):
        os.makedirs(self.tempdir + '/a')
        open(self.tempdir + '/1', mode='w').close()

        os.symlink('a', self.tempdir + '/rel-to-a')
        os.symlink('/a', self.tempdir + '/abs-to-a')
        os.symlink('/b', self.tempdir + '/abs-to-b')
        os.symlink('1', self.tempdir + '/rel-to-1')
        os.symlink('/1', self.tempdir + '/abs-to-1')

        repo._fix_artifacts(debdir=self.tempdir)

        self.assertEqual(os.readlink(self.tempdir + '/rel-to-a'), 'a')
        self.assertEqual(os.readlink(self.tempdir + '/abs-to-a'), 'a')
        self.assertEqual(os.readlink(self.tempdir + '/abs-to-b'), '/b')
        self.assertEqual(os.readlink(self.tempdir + '/rel-to-1'), '1')
        self.assertEqual(os.readlink(self.tempdir + '/abs-to-1'), '1')

    def test_fix_suid(self):
        files = {
            'suid_file': (0o4765, 0o0765),
            'guid_file': (0o2777, 0o0777),
            'suid_guid_file': (0o6744, 0o0744),
            'suid_guid_sticky_file': (0o7744, 0o1744),
        }

        for key in files:
            with self.subTest(key=key):
                file = os.path.join(self.tempdir, key)
                open(file, mode='w').close()
                os.chmod(file, files[key][0])

                repo._fix_artifacts(debdir=self.tempdir)
                self.assertEqual(
                    stat.S_IMODE(os.stat(file).st_mode), files[key][1])

    def test_fix_pkg_config(self):
        pc_file = os.path.join(self.tempdir, 'granite.pc')

        with open(pc_file, 'w') as f:
            f.write('prefix=/usr\n')
            f.write('exec_prefix=${prefix}\n')
            f.write('libdir=${prefix}/lib\n')
            f.write('includedir=${prefix}/include\n')
            f.write('\n')
            f.write('Name: granite\n')
            f.write('Description: elementary\'s Application Framework\n')
            f.write('Version: 0.4\n')
            f.write('Libs: -L${libdir} -lgranite\n')
            f.write('Cflags: -I${includedir}/granite\n')
            f.write('Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 '
                    'gobject-2.0\n')
        repo._fix_artifacts(debdir=self.tempdir)

        with open(pc_file) as f:
            pc_file_content = f.read()
        expected_pc_file_content = """prefix={}/usr
exec_prefix=${{prefix}}
libdir=${{prefix}}/lib
includedir=${{prefix}}/include

Name: granite
Description: elementary's Application Framework
Version: 0.4
Libs: -L${{libdir}} -lgranite
Cflags: -I${{includedir}}/granite
Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
""".format(self.tempdir)

        self.assertEqual(pc_file_content, expected_pc_file_content)

    def test_fix_shebang(self):
        rootdir = 'root'

        files = [
            {
                'path': os.path.join(rootdir, 'bin', 'a'),
                'content': '#!/usr/bin/python\nimport this',
                'expected': '#!/usr/bin/env python\nimport this',
            },
            {
                'path': os.path.join(rootdir, 'sbin', 'b'),
                'content': '#!/usr/bin/python\nimport this',
                'expected': '#!/usr/bin/env python\nimport this',
            },
            {
                'path': os.path.join(rootdir, 'usr', 'bin', 'c'),
                'content': '#!/usr/bin/python\nimport this',
                'expected': '#!/usr/bin/env python\nimport this',
            },
            {
                'path': os.path.join(rootdir, 'usr', 'sbin', 'd'),
                'content': '#!/usr/bin/python\nimport this',
                'expected': '#!/usr/bin/env python\nimport this',
            },
            {
                'path': os.path.join(rootdir, 'opt', 'bin', 'e'),
                'content': '#!/usr/bin/python\nraise Exception()',
                'expected': '#!/usr/bin/python\nraise Exception()',
            },
            {
                'path': os.path.join(rootdir, 'bin', 'd'),
                'content': '#!/usr/bin/python3\nraise Exception()',
                'expected': '#!/usr/bin/python3\nraise Exception()',
            },
        ]

        for f in files:
            with self.subTest(key=f['path']):
                os.makedirs(os.path.dirname(f['path']), exist_ok=True)
                with open(f['path'], 'w') as fd:
                    fd.write(f['content'])

                repo._fix_shebangs(rootdir)

                with open(f['path'], 'r') as fd:
                    self.assertEqual(fd.read(), f['expected'])


class BuildPackagesTestCase(tests.TestCase):

    def test_invalid_package_requested(self):
        with self.assertRaises(EnvironmentError) as raised:
            repo.install_build_packages(['package-does-not-exist'])

        self.assertEqual(
            "Could not find a required package in 'build-packages': "
            '"The cache has no package named \'package-does-not-exist\'"',
            str(raised.exception))
