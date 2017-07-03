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

import fixtures
import logging
import os
import subprocess
import tempfile

from unittest import mock

from snapcraft.internal import libraries
from snapcraft import tests


class TestLdLibraryPathParser(tests.TestCase):

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

        self.assertEqual(['/foo/bar', '/colon', '/separated', '/comma',
                          '/tab', '/space', '/baz'],
                         libraries._extract_ld_library_paths(file_path))


class TestGetLibraries(tests.TestCase):

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
        self.assertEqual(libs, ['/lib/foo.so.1', '/usr/lib/bar.so.2'])

    def test_get_libraries_excludes_slash_snap(self):
        lines = [
            'foo.so.1 => /lib/foo.so.1 (0xdead)',
            'bar.so.2 => /usr/lib/bar.so.2 (0xbeef)',
            'barsnap.so.2 => /snap/snapcraft/current/bar.so.2 (0xbeef)',
            '/lib/baz.so.2 (0x1234)',
        ]
        self.run_output_mock.return_value = '\t' + '\n\t'.join(lines) + '\n'

        libs = libraries.get_dependencies('foo')
        self.assertEqual(libs, ['/lib/foo.so.1', '/usr/lib/bar.so.2'])

    def test_get_libraries_filtered_by_system_libraries(self):
        self.get_system_libs_mock.return_value = frozenset(['foo.so.1'])

        libs = libraries.get_dependencies('foo')
        self.assertEqual(libs, ['/usr/lib/bar.so.2'])

    def test_get_libraries_ldd_failure_logs_warning(self):
        self.run_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo', b'bar')

        self.assertEqual(libraries.get_dependencies('foo'), [])
        self.assertEqual(
            "Unable to determine library dependencies for 'foo'\n",
            self.fake_logger.output)


class TestSystemLibsOnNewRelease(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('distro.version')
        distro_mock = patcher.start()
        distro_mock.return_value = '16.05'
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
        self.assertEqual(libraries.get_dependencies('foo'), [])
