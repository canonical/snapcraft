# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import Contains, Equals
from unittest import mock

from snapcraft.internal import errors, elf, os_release
from snapcraft.tests import unit, fixture_setup


class TestElfBase(unit.TestCase):

    def setUp(self):
        super().setUp()

        self.fake_elf = fixture_setup.FakeElf(root_path=self.path)
        self.useFixture(self.fake_elf)


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


class TestGetLibraries(TestElfBase):

    def setUp(self):
        super().setUp()

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

    def test_get_libraries(self):
        elf_file = self.fake_elf['fake_elf-2.23']
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path)

        self.assertThat(libs, Equals(set(
            [self.fake_elf.root_libraries['foo.so.1'],
             '/usr/lib/bar.so.2'])))

    def test_primed_libraries_are_preferred(self):
        elf_file = self.fake_elf['fake_elf-2.23']
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path)

        self.assertThat(libs, Equals(frozenset(
            [self.fake_elf.root_libraries['foo.so.1'],
             '/usr/lib/bar.so.2'])))

    def test_non_elf_primed_sonames_matches_are_ignored(self):
        primed_foo = os.path.join(self.fake_elf.root_path, 'foo.so.1')
        with open(primed_foo, 'wb') as f:
            # A bz2 header
            f.write(b'\x42\x5a\x68')

        elf_file = self.fake_elf['fake_elf-2.23']
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path)

        self.assertThat(libs, Equals(frozenset(
            ['/lib/foo.so.1', '/usr/lib/bar.so.2'])))

    def test_get_libraries_excludes_slash_snap(self):
        elf_file = self.fake_elf['fake_elf-with-core-libs']
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path)

        self.assertThat(libs, Equals(set(
            [self.fake_elf.root_libraries['foo.so.1'],
             '/usr/lib/bar.so.2'])))

    def test_get_libraries_filtered_by_system_libraries(self):
        self.get_system_libs_mock.return_value = frozenset(['foo.so.1'])

        elf_file = self.fake_elf['fake_elf-2.23']
        libs = elf_file.load_dependencies(root_path='/',
                                          core_base_path='/snap/core/current')
        self.assertThat(libs, Equals(frozenset(['/usr/lib/bar.so.2'])))

    def test_get_libraries_ldd_failure_logs_warning(self):
        elf_file = self.fake_elf['fake_elf-bad-ldd']
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path)

        self.assertThat(libs, Equals(set()))
        self.assertThat(
            self.fake_logger.output,
            Contains("Unable to determine library dependencies for"))


class TestSystemLibsOnNewRelease(TestElfBase):

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

    def test_fail_gracefully_if_system_libs_not_found(self):
        elf_file = self.fake_elf['fake_elf-2.23']
        libs = elf_file.load_dependencies(root_path='/fake',
                                          core_base_path='/fake-core')
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


class TestGetElfFiles(TestElfBase):

    def test_get_elf_files(self):
        elf_files = elf.get_elf_files(self.fake_elf.root_path,
                                      {'fake_elf-2.23'})

        self.assertThat(len(elf_files), Equals(1))
        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.is_executable(), Equals(True))

    def test_get_elf_is_library(self):
        elf_files = elf.get_elf_files(self.fake_elf.root_path,
                                      {'fake_elf-shared-object'})

        self.assertThat(len(elf_files), Equals(1))
        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.is_executable(), Equals(False))
        self.assertThat(elf_file.is_shared_object(), Equals(True))

    def test_skip_object_files(self):
        open(os.path.join(
            self.fake_elf.root_path, 'object_file.o'), 'w').close()

        elf_files = elf.get_elf_files(self.fake_elf.root_path,
                                      {'object_file.o'})
        self.assertThat(elf_files, Equals(set()))

    def test_no_find_dependencies_statically_linked(self):
        elf_files = elf.get_elf_files(self.fake_elf.root_path,
                                      {'fake_elf-static'})
        self.assertThat(elf_files, Equals(set()))

    def test_non_elf_files(self):
        with open(os.path.join(
                self.fake_elf.root_path, 'non-elf'), 'wb') as f:
            # A bz2 header
            f.write(b'\x42\x5a\x68')

        elf_files = elf.get_elf_files(self.fake_elf.root_path, {'non-elf'})
        self.assertThat(elf_files, Equals(set()))

    def test_symlinks(self):
        symlinked_path = os.path.join(self.fake_elf.root_path, 'symlinked')
        os.symlink('/bin/dash', symlinked_path)

        elf_files = elf.get_elf_files(self.fake_elf.root_path, {'symlinked'})

        self.assertThat(elf_files, Equals(set()))


class TestGetRequiredGLIBC(TestElfBase):

    def setUp(self):
        super().setUp()

        self.elf_file = self.fake_elf['fake_elf-2.23']

    def test_get_required_glibc(self):
        self.assertThat(self.elf_file.get_required_glibc(), Equals('2.23'))

    def test_linker_version_greater_than_required_glibc(self):
        self.assertTrue(
            self.elf_file.is_linker_compatible(linker='ld-2.26.so'))

    def test_linker_version_equals_required_glibc(self):
        self.assertTrue(
            self.elf_file.is_linker_compatible(linker='ld-2.23.so'))

    def test_linker_version_less_than_required_glibc(self):
        self.assertFalse(
            self.elf_file.is_linker_compatible(linker='ld-1.2.so'))

    def test_bad_linker_raises_exception(self):
        self.assertRaises(EnvironmentError,
                          self.elf_file.is_linker_compatible,
                          linker='lib64/ld-linux-x86-64.so.2')


class TestElfFileSymbols(TestElfBase):

    def setUp(self):
        super().setUp()

    def test_symbols(self):
        elf_file = self.fake_elf['fake_elf-2.23']

        self.assertThat(len(elf_file.symbols), Equals(3))

        self.assertThat(elf_file.symbols[0].name, Equals('endgrent'))
        self.assertThat(elf_file.symbols[0].version, Equals('GLIBC_2.2.5'))
        self.assertThat(elf_file.symbols[0].section, Equals('UND'))

        self.assertThat(elf_file.symbols[1].name,
                        Equals('__ctype_toupper_loc'))
        self.assertThat(elf_file.symbols[1].version, Equals('GLIBC_2.23'))
        self.assertThat(elf_file.symbols[1].section, Equals('UND'))

        self.assertThat(elf_file.symbols[2].name, Equals('PyCodec_Register'))
        self.assertThat(elf_file.symbols[2].version, Equals(''))
        self.assertThat(elf_file.symbols[2].section, Equals('13'))

    def test_symbols_no_match(self):
        # fake_elf-shared-object has no GLIBC dependency, but two symbols
        # nonetheless
        elf_file = self.fake_elf['fake_elf-shared-object']

        self.assertThat(len(elf_file.symbols), Equals(2))


class TestPatcher(TestElfBase):

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

        patcher = mock.patch('subprocess.check_call')
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP', self.snap))
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP_NAME', self.snap_name))

    def test_patch(self):
        elf_file = self.fake_elf['fake_elf-2.23']
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker='/lib/fake-ld',
                                  root_path='/fake')
        elf_patcher.patch(elf_file=elf_file)

        self.check_call_mock.assert_called_once_with([
            self.expected_patchelf, '--set-interpreter', '/lib/fake-ld',
            elf_file.path])

    def test_patch_does_nothing_if_no_interpreter(self):
        elf_file = self.fake_elf['fake_elf-static']
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker='/lib/fake-ld',
                                  root_path='/fake')
        elf_patcher.patch(elf_file=elf_file)

        self.assertFalse(self.check_call_mock.called)


class TestPatcherErrors(TestElfBase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('subprocess.check_call')
        check_call_mock = patcher.start()
        check_call_mock.side_effect = subprocess.CalledProcessError(
            2, ['patchelf'])
        self.addCleanup(patcher.stop)

    def test_patch_fails_raises_patcherror_exception(self):
        elf_file = self.fake_elf['fake_elf-2.23']
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker='/lib/fake-ld',
                                  root_path='/fake')

        self.assertRaises(errors.PatcherError,
                          elf_patcher.patch,
                          elf_file=elf_file)
