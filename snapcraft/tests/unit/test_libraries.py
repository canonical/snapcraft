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

import fixtures
import logging
import os
import subprocess
import tempfile
from textwrap import dedent

from testtools.matchers import Equals
from unittest import mock

from snapcraft.internal import (
    libraries,
    os_release,
)
from snapcraft.tests import unit


class TestLdLibraryPathParser(unit.TestCase):

    def _write_conf_file(self, contents):
        tmp = tempfile.NamedTemporaryFile(delete=False, mode='w')
        self.addCleanup(os.remove, tmp.name)

        tmp.write(contents)
        tmp.close()

        return tmp.name

    def test_extract_ld_library_paths(self):
        file_path = self._write_conf_file("""# This is a comment
/foo/bar
/colon:/separated,/comma\t/tab /space # This is another comment
/baz""")

        self.assertThat(
            libraries._extract_ld_library_paths(file_path),
            Equals(['/foo/bar', '/colon', '/separated', '/comma',
                    '/tab', '/space', '/baz']))


class TestGetLibraries(unit.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.internal.common.run_output')
        self.run_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        lines = [
            'foo.so.1 => /lib/foo.so.1 (0xdead)',
            'bar.so.2 => /usr/lib/bar.so.2 (0xbeef)',
            '/lib/baz.so.2 (0x1234)',
        ]
        self.run_output_mock.return_value = '\t' + '\n\t'.join(lines) + '\n'

        patcher = mock.patch('snapcraft.internal.libraries._get_system_libs')
        self.get_system_libs_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.get_system_libs_mock.return_value = frozenset()

        patcher = mock.patch('os.path.exists')
        self.path_exists_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.path_exists_mock.return_value = True

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_get_libraries(self):
        libs = libraries.get_dependencies('foo')
        self.assertThat(libs, Equals(['/lib/foo.so.1', '/usr/lib/bar.so.2']))

    def test_get_libraries_excludes_slash_snap(self):
        lines = [
            'foo.so.1 => /lib/foo.so.1 (0xdead)',
            'bar.so.2 => /usr/lib/bar.so.2 (0xbeef)',
            'barsnap.so.2 => /snap/snapcraft/current/bar.so.2 (0xbeef)',
            '/lib/baz.so.2 (0x1234)',
        ]
        self.run_output_mock.return_value = '\t' + '\n\t'.join(lines) + '\n'

        libs = libraries.get_dependencies('foo')
        self.assertThat(libs, Equals(['/lib/foo.so.1', '/usr/lib/bar.so.2']))

    def test_get_libraries_filtered_by_system_libraries(self):
        self.get_system_libs_mock.return_value = frozenset(['foo.so.1'])

        libs = libraries.get_dependencies('foo')
        self.assertThat(libs, Equals(['/usr/lib/bar.so.2']))

    def test_get_libraries_ldd_failure_logs_warning(self):
        self.run_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo', b'bar')

        self.assertThat(libraries.get_dependencies('foo'), Equals([]))
        self.assertThat(
            self.fake_logger.output,
            Equals("Unable to determine library dependencies for 'foo'\n"))


class TestSystemLibsOnNewRelease(unit.TestCase):

    def setUp(self):
        super().setUp()

        with open('os-release', 'w') as f:
            f.write(dedent("""\
                NAME="Ubuntu"
                VERSION="16.04.3 LTS (Xenial Xerus)"
                ID=ubuntu
                ID_LIKE=debian
                PRETTY_NAME="Ubuntu 16.04.3 LTS"
                VERSION_ID="16.04"
                HOME_URL="http://www.ubuntu.com/"
                SUPPORT_URL="http://help.ubuntu.com/"
                BUG_REPORT_URL="http://bugs.launchpad.net/ubuntu/"
                UBUNTU_CODENAME=xenial
            """))
        release = os_release.OsRelease(os_release_file='os-release')

        def _create_os_release(*args, **kwargs):
            return release

        patcher = mock.patch(
            'snapcraft.internal.os_release.OsRelease',
            wraps=_create_os_release)
        self.os_release_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.common.run_output')
        self.run_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        lines = [
            'foo.so.1 => /lib/foo.so.1 (0xdead)',
            'bar.so.2 => /usr/lib/bar.so.2 (0xbeef)',
            '/lib/baz.so.2 (0x1234)',
        ]
        self.run_output_mock.return_value = '\t' + '\n\t'.join(lines) + '\n'

    def test_fail_gracefully_if_system_libs_not_found(self):
        self.assertThat(libraries.get_dependencies('foo'), Equals([]))


class TestSystemLibsOnReleasesWithNoVersionId(unit.TestCase):

    def setUp(self):
        super().setUp()

        libraries._libraries = None

        with open('os-release', 'w') as f:
            f.write(dedent("""\
                NAME="Gentoo"
                ID=gentoo
                PRETTY_NAME="Gentoo/Linux"
                HOME_URL="http://www.gentoo.org/"
                SUPPORT_URL="http://www.gentoo.org/main/en/support.xml"
                BUG_REPORT_URL="https://bugs.gentoo.org/"
            """))
        release = os_release.OsRelease(os_release_file='os-release')

        def _create_os_release(*args, **kwargs):
            return release

        patcher = mock.patch(
            'snapcraft.internal.os_release.OsRelease',
            wraps=_create_os_release)
        self.os_release_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch('snapcraft.internal.libraries.repo.Repo.get_package_libraries',
                return_value=['/usr/lib/libc.so.6', '/lib/libpthreads.so.6'])
    def test_fail_gracefully_if_no_version_id_found(self, mock_package_libs):
        self.assertThat(libraries._get_system_libs(),
                        Equals(frozenset(['libc.so.6', 'libpthreads.so.6'])))
