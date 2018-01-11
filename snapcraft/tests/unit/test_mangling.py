# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import textwrap
from unittest.mock import patch

import fixtures
from testtools.matchers import FileContains, FileExists, Not

from snapcraft.internal import elf, mangling
from snapcraft.tests import unit


def _create_file(filename, contents):
    os.makedirs('test-dir', exist_ok=True)
    file_path = os.path.join('test-dir', filename)
    with open(file_path, 'w') as f:
        f.write(contents)

    return file_path


class ManglingPythonShebangTestCase(unit.TestCase):

    def test_python(self):
        file_path = _create_file('file', textwrap.dedent("""\
            #! /usr/bin/python2.7

            # Larger file
        """))
        mangling.rewrite_python_shebangs(os.path.dirname(file_path))
        self.assertThat(file_path, FileContains(textwrap.dedent("""\
            #!/usr/bin/env python2.7

            # Larger file
        """)))

    def test_python3(self):
        file_path = _create_file('file', '#!/usr/bin/python3')
        mangling.rewrite_python_shebangs(os.path.dirname(file_path))
        self.assertThat(file_path, FileContains('#!/usr/bin/env python3'))

    def test_python_args(self):
        file_path1 = _create_file('file1', '#!/usr/bin/python')
        file_path2 = _create_file('file2', textwrap.dedent("""\
            #! /usr/bin/python -E

            # Larger file
        """))
        mangling.rewrite_python_shebangs(os.path.dirname(file_path1))
        self.assertThat(file_path1, FileContains('#!/usr/bin/env python'))
        self.assertThat(file_path2, FileContains(
            textwrap.dedent("""\
                #!/bin/sh
                ''''exec python -E -- "$0" "$@" # '''

                # Larger file
            """)))

    def test_python3_args(self):
        file_path1 = _create_file('file1', '#!/usr/bin/python3')
        file_path2 = _create_file('file2', '#!/usr/bin/python3 -E')
        mangling.rewrite_python_shebangs(os.path.dirname(file_path1))
        self.assertThat(file_path1, FileContains('#!/usr/bin/env python3'))
        self.assertThat(file_path2, FileContains(
            textwrap.dedent("""\
                #!/bin/sh
                ''''exec python3 -E -- "$0" "$@" # '''""")))

    def test_python_mixed_args(self):
        file_path1 = _create_file('file1', '#!/usr/bin/python')
        # Ensure extra spaces are chopped off
        file_path2 = _create_file('file2', '#!/usr/bin/python3       -Es')
        mangling.rewrite_python_shebangs(os.path.dirname(file_path1))
        self.assertThat(file_path1, FileContains('#!/usr/bin/env python'))
        self.assertThat(file_path2, FileContains(
            textwrap.dedent("""\
                #!/bin/sh
                ''''exec python3 -Es -- "$0" "$@" # '''""")))

    def test_following_docstring_no_rewrite(self):
        file_path = _create_file('file', textwrap.dedent("""\
            #!/usr/bin/env python3.5

            '''
            This is a test
            =======================
        """))
        mangling.rewrite_python_shebangs(os.path.dirname(file_path))
        self.assertThat(file_path, FileContains(textwrap.dedent("""\
            #!/usr/bin/env python3.5

            '''
            This is a test
            =======================
        """)))


class FakeElf(fixtures.Fixture):

    def __init__(self, *, root_path):
        super().__init__()

        self.root_path = root_path

    def _setUp(self):
        super()._setUp()

        readelf_path = os.path.abspath(os.path.join(
            __file__, '..', '..', 'bin', 'readelf'))
        current_path = os.environ.get('PATH')
        new_path = '{}:{}'.format(readelf_path, current_path)
        self.useFixture(fixtures.EnvironmentVariable('PATH', new_path))

        stub_magic = ('ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
                      'dynamically linked, interpreter '
                      '/lib64/ld-linux-x86-64.so.2, for GNU/Linux 2.6.32')

        self.elf_files = [
            elf.ElfFile(path=os.path.join(self.root_path, 'fake_elf-2.26'),
                        magic=stub_magic),
            elf.ElfFile(path=os.path.join(self.root_path, 'fake_elf-2.23'),
                        magic=stub_magic),
            elf.ElfFile(path=os.path.join(self.root_path, 'fake_elf-1.1'),
                        magic=stub_magic),
        ]

        for elf_file in self.elf_files:
            open(elf_file.path, 'w').close()


# This is just a subset
_LIBC6_LIBRARIES = [
    'ld-2.26.so',
    'ld-linux-x86-64.so.2',
    'libBrokenLocale-2.26.so',
    'libBrokenLocale.so.1',
    'libSegFault.so',
    'libanl-2.26.so',
]


class HandleGlibcTestCase(unit.TestCase):

    def _setup_libc6(self):
        lib_path = os.path.join(self.path, 'lib')
        libraries = {os.path.join(lib_path, l) for l in _LIBC6_LIBRARIES}

        os.mkdir(lib_path)
        for library in libraries:
            open(library, 'w').close()

        return libraries

    def setUp(self):
        super().setUp()

        patcher = patch('snapcraft.internal.elf.ElfFile.load_dependencies')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('snapcraft.internal.elf.Patcher.patch')
        self.patch_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('snapcraft.internal.repo.Repo.get_package_libraries')
        self.get_packages_mock = patcher.start()
        self.get_packages_mock.return_value = self._setup_libc6()
        self.addCleanup(patcher.stop)

        self.fake_elf = FakeElf(root_path=self.path)
        self.useFixture(self.fake_elf)

    def test_glibc_mangling(self):
        mangling.handle_glibc_mismatch(
            elf_files=self.fake_elf.elf_files,
            root_path=self.path,
            core_base_path='/snap/core/current',
            snap_base_path='/snap/snap-name/current')

        self.get_packages_mock.assert_called_once_with('libc6')
        self.assertThat(os.path.join(self.path, 'snap', 'libc6', 'ld-2.26.so'),
                        FileExists())
        # Only fake_elf1 requires a newer libc6
        self.patch_mock.assert_called_once_with(
            elf_file=self.fake_elf.elf_files[0])

    def test_nothing_to_patch(self):
        mangling.handle_glibc_mismatch(
            # Pass part of the slice without fake_elf-2.26
            elf_files=self.fake_elf.elf_files[1:],
            root_path=self.path,
            core_base_path='/snap/core/current',
            snap_base_path='/snap/snap-name/current')

        self.get_packages_mock.assert_not_called()
        self.assertThat(os.path.join(self.path, 'snap', 'libc6', 'ld-2.26.so'),
                        Not(FileExists()))
        self.patch_mock.assert_not_called()

    def test_bad_dynamic_linker_in_libc6_package(self):
        self.get_packages_mock.return_value = {'/usr/lib/dyn-linker-2.25.so'}

        self.assertRaises(RuntimeError,
                          mangling.handle_glibc_mismatch,
                          elf_files=self.fake_elf.elf_files,
                          root_path=self.path,
                          core_base_path='/snap/core/current',
                          snap_base_path='/snap/snap-name/current')
