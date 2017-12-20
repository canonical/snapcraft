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
import sys
from textwrap import dedent

from testtools.matchers import Equals
from unittest import mock

from snapcraft.internal import errors, elf, os_release
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
            elf._extract_ld_library_paths(file_path),
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

        patcher = mock.patch('snapcraft.internal.elf._get_system_libs')
        self.get_system_libs_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.get_system_libs_mock.return_value = frozenset()

        patcher = mock.patch('os.path.exists')
        self.path_exists_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.path_exists_mock.return_value = True

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

        self.stub_magic = ('ELF 64-bit LSB executable, x86-64, '
                           'version 1 (SYSV),dynamically linked, '
                           'interpreter /lib64/ld-linux-x86-64.so.2, '
                           'for GNU/Linux 2.6.32')

    def test_get_libraries(self):
        elf_file = elf.ElfFile(path='foo', magic=self.stub_magic)
        libs = elf_file.load_dependencies(base_path='/')
        self.assertThat(libs, Equals(frozenset(
            ['/lib/foo.so.1', '/usr/lib/bar.so.2'])))

    def test_get_libraries_excludes_slash_snap(self):
        lines = [
            'foo.so.1 => /lib/foo.so.1 (0xdead)',
            'bar.so.2 => /usr/lib/bar.so.2 (0xbeef)',
            'barsnap.so.2 => /snap/snapcraft/current/bar.so.2 (0xbeef)',
            '/lib/baz.so.2 (0x1234)',
        ]
        self.run_output_mock.return_value = '\t' + '\n\t'.join(lines) + '\n'

        elf_file = elf.ElfFile(path='foo', magic=self.stub_magic)
        libs = elf_file.load_dependencies(base_path='/')
        self.assertThat(libs, Equals(
            frozenset(['/lib/foo.so.1', '/usr/lib/bar.so.2'])))

    def test_get_libraries_filtered_by_system_libraries(self):
        self.get_system_libs_mock.return_value = frozenset(['foo.so.1'])

        elf_file = elf.ElfFile(path='foo', magic=self.stub_magic)
        libs = elf_file.load_dependencies(base_path='/')
        self.assertThat(libs, Equals(frozenset(['/usr/lib/bar.so.2'])))

    def test_get_libraries_ldd_failure_logs_warning(self):
        self.run_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo', b'bar')

        elf_file = elf.ElfFile(path='foo', magic=self.stub_magic)
        libs = elf_file.load_dependencies(base_path='/')
        self.assertThat(libs, Equals(set()))
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
        stub_magic = ('ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
                      'dynamically linked, interpreter '
                      '/lib64/ld-linux-x86-64.so.2, for GNU/Linux 2.6.32')
        elf_file = elf.ElfFile(path='foo', magic=stub_magic)
        libs = elf_file.load_dependencies(base_path='/')
        self.assertThat(libs, Equals(frozenset()))


class TestSystemLibsOnReleasesWithNoVersionId(unit.TestCase):

    def setUp(self):
        super().setUp()

        elf._libraries = None

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

    @mock.patch('snapcraft.internal.elf.repo.Repo.get_package_libraries',
                return_value=['/usr/lib/libc.so.6', '/lib/libpthreads.so.6'])
    def test_fail_gracefully_if_no_version_id_found(self, mock_package_libs):
        self.assertThat(elf._get_system_libs(),
                        Equals(frozenset(['libc.so.6', 'libpthreads.so.6'])))


class GetElfFilesTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()
        self.workdir = os.path.join(os.getcwd(), 'workdir')
        os.mkdir(self.workdir)

        self.ms_mock = mock.Mock()
        self.ms_mock.load.return_value = 0
        self.ms_mock.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'dynamically linked interpreter /lib64/ld-linux-x86-64.so.2, '
            'for GNU/Linux 2.6.32, BuildID[sha1]=XYZ, stripped'
        )

        patcher = mock.patch('magic.open')
        self.magic_mock = patcher.start()
        self.magic_mock.return_value = self.ms_mock
        self.addCleanup(patcher.stop)

    def test_get_elf_files(self):
        linked_elf_path = os.path.join(self.workdir, 'linked')
        open(linked_elf_path, 'w').close()

        linked_elf_path_b = linked_elf_path.encode(sys.getfilesystemencoding())

        elf_files = elf.get_elf_files(self.workdir, {'linked'})

        self.assertThat(len(elf_files), Equals(1))
        self.ms_mock.file.assert_called_once_with(linked_elf_path_b)

        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.path, Equals(linked_elf_path))
        self.assertThat(elf_file.is_executable, Equals(True))

    def test_get_elf_is_library(self):
        self.ms_mock.file.return_value = (
            'ELF 64-bit LSB shared object, x86-64, version 1 (SYSV), '
            'dynamically linked, '
            'BuildID[sha1]=62b2bc59168b25ab9b025182c1f5f43194ba167b, stripped'
        )
        linked_elf_path = os.path.join(self.workdir, 'linked')
        open(linked_elf_path, 'w').close()

        linked_elf_path_b = linked_elf_path.encode(sys.getfilesystemencoding())

        elf_files = elf.get_elf_files(self.workdir, {'linked'})

        self.assertThat(len(elf_files), Equals(1))
        self.ms_mock.file.assert_called_once_with(linked_elf_path_b)

        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.path, Equals(linked_elf_path))
        self.assertThat(elf_file.is_executable, Equals(False))

    def test_skip_object_files(self):
        open(os.path.join(self.workdir, 'object_file.o'), 'w').close()

        elf_files = elf.get_elf_files(self.workdir, {'object_file.o'})

        self.assertFalse(self.ms_mock.file.called,
                         'Expected object file to be skipped')
        self.assertThat(elf_files, Equals(set()))

    def test_no_find_dependencies_of_non_dynamically_linked(self):
        statically_linked_elf_path = os.path.join(self.workdir,
                                                  'statically-linked')
        open(statically_linked_elf_path, 'w').close()

        statically_linked_elf_path_b = statically_linked_elf_path.encode(
            sys.getfilesystemencoding())

        self.ms_mock.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'statically linked, for GNU/Linux 2.6.32, '
            'BuildID[sha1]=XYZ, stripped')

        elf_files = elf.get_elf_files(self.workdir,
                                      {'statically-linked'})

        self.ms_mock.file.assert_called_once_with(statically_linked_elf_path_b)
        self.assertThat(elf_files, Equals(set()))

    def test_non_elf_files(self):
        non_elf_path = os.path.join(self.workdir, 'non-elf')
        open(non_elf_path, 'w').close()

        non_elf_path_b = non_elf_path.encode(sys.getfilesystemencoding())

        self.ms_mock.file.return_value = 'JPEG image data, Exif standard: ...'

        elf_files = elf.get_elf_files(self.workdir, {'non-elf'})

        self.ms_mock.file.assert_called_once_with(non_elf_path_b)
        self.assertThat(elf_files, Equals(set()))

    def test_symlinks(self):
        symlinked_path = os.path.join(self.workdir, 'symlinked')
        os.symlink('/bin/dash', symlinked_path)

        elf_files = elf.get_elf_files(self.workdir, {'symlinked'})

        self.assertFalse(self.ms_mock.file.called,
                         'magic is not needed for symlinks')
        self.assertThat(elf_files, Equals(set()))

    def test_fail_to_load_magic_raises_exception(self):
        self.magic_mock.return_value.load.return_value = 1

        raised = self.assertRaises(
            RuntimeError,
            elf.get_elf_files, '.', set())

        self.assertThat(
            raised.__str__(), Equals('Cannot load magic header detection'))


class TestPatcher(unit.TestCase):

    scenarios = [
        ('snap',
         dict(snap='/snap/snapcraft/current',
              snap_name='snapcraft',
              expected_patchelf='/snap/snapcraft/current/bin/patchelf')),
        ('non-snap',
         dict(snap='',
              snap_name='',
              expected_patchelf='patchelf')),
    ]

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP', self.snap))
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP_NAME', self.snap_name))

    @mock.patch('subprocess.check_call')
    def test_patch(self, check_call_mock):
        stub_magic = ('ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
                      'dynamically linked, interpreter '
                      '/lib64/ld-linux-x86-64.so.2, for GNU/Linux 2.6.32')
        elf_file = elf.ElfFile(path='/fake-elf', magic=stub_magic)
        elf_patcher = elf.Patcher(dynamic_linker='/lib/fake-ld')
        elf_patcher.patch(elf_file=elf_file)

        check_call_mock.assert_called_once_with([
            self.expected_patchelf, '--set-interpreter', '/lib/fake-ld',
            '/fake-elf'])

    @mock.patch('subprocess.check_call')
    def test_patch_does_nothing_if_no_interpreter(self, check_call_mock):
        stub_magic = ('ELF 64-bit LSB shared object, x86-64, '
                      'version 1 (SYSV), dynamically linked')
        elf_file = elf.ElfFile(path='/fake-elf', magic=stub_magic)
        elf_patcher = elf.Patcher(dynamic_linker='/lib/fake-ld')
        elf_patcher.patch(elf_file=elf_file)

        self.assertFalse(check_call_mock.called)


class TestPatcherErrors(unit.TestCase):

    @mock.patch('subprocess.check_call',
                side_effect=subprocess.CalledProcessError(2, ['patchelf']))
    def test_patch_fails_raises_patcherror_exception(self, check_call_mock):
        stub_magic = ('ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
                      'dynamically linked, interpreter '
                      '/lib64/ld-linux-x86-64.so.2, for GNU/Linux 2.6.32')
        elf_file = elf.ElfFile(path='/fake-elf', magic=stub_magic)
        elf_patcher = elf.Patcher(dynamic_linker='/lib/fake-ld')

        self.assertRaises(errors.PatcherError,
                          elf_patcher.patch,
                          elf_file=elf_file)
