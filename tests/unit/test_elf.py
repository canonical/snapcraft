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

import logging
import os
import sys
import tempfile
from unittest import mock

import fixtures
import pytest
from testtools.matchers import Contains, EndsWith, Equals, NotEquals, StartsWith

from snapcraft import ProjectOptions
from snapcraft.internal import elf, errors
from tests import fixture_setup, unit


class TestElfBase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_elf = fixture_setup.FakeElf(root_path=self.path)
        self.useFixture(self.fake_elf)
        self.content_dirs = []
        self.arch_triplet = ProjectOptions().arch_triplet


class TestLdLibraryPathParser(unit.TestCase):
    def _write_conf_file(self, contents):
        tmp = tempfile.NamedTemporaryFile(delete=False, mode="w")
        self.addCleanup(os.remove, tmp.name)

        tmp.write(contents)
        tmp.close()

        return tmp.name

    def test_extract_ld_library_paths(self):
        file_path = self._write_conf_file(
            """# This is a comment
/foo/bar
/colon:/separated,/comma\t/tab /space # This is another comment
/baz"""
        )

        self.assertThat(
            elf._extract_ld_library_paths(file_path),
            Equals(
                ["/foo/bar", "/colon", "/separated", "/comma", "/tab", "/space", "/baz"]
            ),
        )


class TestElfFileSmoketest(unit.TestCase):
    def test_bin_echo(self):
        # Try parsing a file without the pyelftools logic mocked out
        elf_file = elf.ElfFile(path=sys.executable)

        self.assertThat(elf_file.path, Equals(sys.executable))

        # The arch attribute will be a tuple of three strings
        self.assertTrue(isinstance(elf_file.arch, tuple))
        self.assertThat(len(elf_file.arch), Equals(3))
        self.assertThat(elf_file.arch[0], StartsWith("ELFCLASS"))
        self.assertThat(elf_file.arch[1], StartsWith("ELFDATA"))
        self.assertThat(elf_file.arch[2], StartsWith("EM_"))

        # We expect Python to be a dynamic linked executable with an
        # ELF interpreter.
        self.assertTrue(isinstance(elf_file.interp, str))
        self.assertThat(elf_file.interp, NotEquals(""))

        # Python is not a shared library, so has no soname or defined versions
        self.assertThat(elf_file.soname, Equals(""))
        self.assertThat(elf_file.versions, Equals(set()))

        # We expect that Python will be linked to libc
        for lib in elf_file.needed.values():
            if lib.name.startswith("libc.so"):
                break
        else:
            self.fail("Expected to find libc in needed library list")

        self.assertTrue(isinstance(lib.name, str))
        for version in lib.versions:
            self.assertTrue(
                isinstance(version, str), "expected {!r} to be a string".format(version)
            )

        # GCC adds a build ID to executables
        self.assertThat(elf_file.build_id, NotEquals(""))

        # If the Python interpreter is distro packaged, it probably
        # doesn't have debug info, but we don't know for sure.
        # Instead just check that it is a boolean.
        self.assertTrue(isinstance(elf_file.has_debug_info, bool))

        # Ensure type is detered as executable.
        self.assertThat(elf_file.elf_type, Equals("ET_EXEC"))


class TestInvalidElf(unit.TestCase):
    def test_invalid_elf_file(self):
        invalid_elf = os.path.join(self.path, "invalid-elf")
        open(invalid_elf, "wb").write(b"\x7fELF\x00")

        elf_files = elf.get_elf_files(self.path, ["invalid-elf"])
        self.assertThat(elf_files, Equals(set()))


class TestMissingLibraries(TestElfBase):
    def test_get_libraries_missing_libs(self):
        elf_file = self.fake_elf["fake_elf-with-missing-libs"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
        )

        self.assertThat(
            libs,
            Equals(set([self.fake_elf.root_libraries["foo.so.1"], "missing.so.2"])),
        )


class TestGetLibraries(TestElfBase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixtures.MockPatch("os.path.exists", return_value=True))

    def test_get_libraries(self):
        elf_file = self.fake_elf["fake_elf-2.23"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
        )

        self.assertThat(
            libs,
            Equals(
                set([self.fake_elf.root_libraries["foo.so.1"], "/usr/lib/bar.so.2"])
            ),
        )

    def test_get_libraries_with_soname_cache(self):
        elf_file = self.fake_elf["fake_elf-2.23"]

        arch = ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")
        soname_cache = elf.SonameCache()
        soname_cache[arch, "bar.so.2"] = "/lib/bar.so.2"

        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
            soname_cache=soname_cache,
        )

        # With no cache this would have returned '/usr/lib/bar.so.2'
        self.assertThat(
            libs,
            Equals(set([self.fake_elf.root_libraries["foo.so.1"], "/lib/bar.so.2"])),
        )

    def test_primed_libraries_are_preferred(self):
        elf_file = self.fake_elf["fake_elf-2.23"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
        )

        self.assertThat(
            libs,
            Equals(
                frozenset(
                    [self.fake_elf.root_libraries["foo.so.1"], "/usr/lib/bar.so.2"]
                )
            ),
        )

    def test_non_elf_primed_sonames_matches_are_ignored(self):
        primed_foo = os.path.join(self.fake_elf.root_path, "foo.so.1")
        with open(primed_foo, "wb") as f:
            # A bz2 header
            f.write(b"\x42\x5a\x68")

        elf_file = self.fake_elf["fake_elf-2.23"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
        )

        self.assertThat(libs, Equals(frozenset(["/lib/foo.so.1", "/usr/lib/bar.so.2"])))

    def test_get_libraries_excludes_slash_snap(self):
        elf_file = self.fake_elf["fake_elf-with-core-libs"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
        )

        self.assertThat(
            libs,
            Equals(
                set([self.fake_elf.root_libraries["foo.so.1"], "/usr/lib/bar.so.2"])
            ),
        )

    def test_get_libraries_ldd_failure_logs_warning(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

        elf_file = self.fake_elf["fake_elf-bad-ldd"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
            arch_triplet=self.arch_triplet,
            content_dirs=self.content_dirs,
        )

        self.assertThat(libs, Equals(set()))
        self.assertThat(
            self.fake_logger.output,
            Contains("Unable to determine library dependencies for"),
        )

    def test_existing_host_library_searched_for(self):
        elf_file = self.fake_elf["fake_elf-with-host-libraries"]

        class MooLibrary(elf.Library):
            """A Library implementation that always returns valid for moo."""

            def _is_valid_elf(self, resolved_path: str) -> bool:
                #  This path is defined in ldd for fake_elf-with-host-libraries.
                if resolved_path == "/usr/lib/moo.so.2":
                    return True
                else:
                    return super()._is_valid_elf(resolved_path)

        with mock.patch("snapcraft.internal.elf.Library", side_effect=MooLibrary):
            libs = elf_file.load_dependencies(
                root_path=self.fake_elf.root_path,
                core_base_path=self.fake_elf.core_base_path,
                arch_triplet=self.arch_triplet,
                content_dirs=self.content_dirs,
            )

        self.assertThat(libs, Equals({self.fake_elf.root_libraries["moo.so.2"]}))


class TestLibrary(TestElfBase):
    def test_is_valid_elf_ignores_corrupt_files(self):
        soname = "libssl.so.1.0.0"
        soname_path = os.path.join(self.path, soname)
        library = elf.Library(
            soname=soname,
            soname_path=soname_path,
            search_paths=[self.path],
            core_base_path="/snap/core/current",
            arch=("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64"),
            soname_cache=elf.SonameCache(),
        )

        self.assertThat(library._is_valid_elf(soname_path), Equals(True))

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.elf.ElfFile",
                side_effect=errors.CorruptedElfFileError(
                    path=soname_path, error=RuntimeError()
                ),
            )
        )

        self.assertThat(library._is_valid_elf(soname_path), Equals(False))


class TestGetElfFiles(TestElfBase):
    def test_get_elf_files(self):
        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"fake_elf-2.23"})

        self.assertThat(len(elf_files), Equals(1))
        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.interp, Equals("/lib64/ld-linux-x86-64.so.2"))

    def test_skip_object_files(self):
        open(os.path.join(self.fake_elf.root_path, "object_file.o"), "w").close()

        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"object_file.o"})
        self.assertThat(elf_files, Equals(set()))

    def test_no_find_dependencies_statically_linked(self):
        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"fake_elf-static"})
        self.assertThat(elf_files, Equals(set()))

    def test_elf_with_execstack(self):
        elf_files = elf.get_elf_files(
            self.fake_elf.root_path, {"fake_elf-with-execstack"}
        )
        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.execstack_set, Equals(True))

    def test_elf_without_execstack(self):
        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"fake_elf-2.23"})
        elf_file = set(elf_files).pop()
        self.assertThat(elf_file.execstack_set, Equals(False))

    def test_non_elf_files(self):
        with open(os.path.join(self.fake_elf.root_path, "non-elf"), "wb") as f:
            # A bz2 header
            f.write(b"\x42\x5a\x68")

        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"non-elf"})
        self.assertThat(elf_files, Equals(set()))

    def test_symlinks(self):
        symlinked_path = os.path.join(self.fake_elf.root_path, "symlinked")
        os.symlink("/bin/dash", symlinked_path)

        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"symlinked"})

        self.assertThat(elf_files, Equals(set()))

    def test_device_files(self):
        elf_files = elf.get_elf_files("/dev", {"null"})
        self.assertThat(elf_files, Equals(set()))

    def test_fifo(self):
        fifo_path = os.path.join(self.fake_elf.root_path, "fifo")
        os.mkfifo(fifo_path)

        elf_files = elf.get_elf_files(self.fake_elf.root_path, {"fifo"})
        self.assertThat(elf_files, Equals(set()))


class TestGetRequiredGLIBC(TestElfBase):
    def setUp(self):
        super().setUp()

        self.elf_file = self.fake_elf["fake_elf-2.23"]

    def test_get_required_glibc(self):
        self.assertThat(self.elf_file.get_required_glibc(), Equals("2.23"))

    def test_linker_version_greater_than_required_glibc(self):
        self.assertTrue(self.elf_file.is_linker_compatible(linker_version="2.26"))

    def test_linker_version_equals_required_glibc(self):
        self.assertTrue(self.elf_file.is_linker_compatible(linker_version="2.23"))

    def test_linker_version_less_than_required_glibc(self):
        self.assertFalse(self.elf_file.is_linker_compatible(linker_version="1.2"))


class TestElfFileAttrs(TestElfBase):
    def setUp(self):
        super().setUp()

    def test_executable(self):
        elf_file = self.fake_elf["fake_elf-2.23"]

        self.assertThat(elf_file.interp, Equals("/lib64/ld-linux-x86-64.so.2"))
        self.assertThat(elf_file.soname, Equals(""))
        self.assertThat(sorted(elf_file.needed.keys()), Equals(["libc.so.6"]))

        glibc = elf_file.needed["libc.so.6"]
        self.assertThat(glibc.name, Equals("libc.so.6"))
        self.assertThat(glibc.versions, Equals({"GLIBC_2.2.5", "GLIBC_2.23"}))

    def test_shared_object(self):
        # fake_elf-shared-object has no GLIBC dependency, but two symbols
        # nonetheless
        elf_file = self.fake_elf["fake_elf-shared-object"]

        self.assertThat(elf_file.interp, Equals(""))
        self.assertThat(elf_file.soname, Equals("libfake_elf.so.0"))
        self.assertThat(sorted(elf_file.needed.keys()), Equals(["libssl.so.1.0.0"]))

        openssl = elf_file.needed["libssl.so.1.0.0"]
        self.assertThat(openssl.name, Equals("libssl.so.1.0.0"))
        self.assertThat(openssl.versions, Equals({"OPENSSL_1.0.0"}))

    def test_libssl(self):
        # libssl.so.1.0.0 defines some symbol versions it provides
        elf_file = self.fake_elf["libssl.so.1.0.0"]

        self.assertThat(elf_file.interp, Equals(""))
        self.assertThat(elf_file.soname, Equals("libssl.so.1.0.0"))
        self.assertThat(elf_file.versions, Equals({"libssl.so.1.0.0", "OPENSSL_1.0.0"}))


class TestPatcher(TestElfBase):
    def test_patch(self):
        elf_file = self.fake_elf["fake_elf-2.23"]
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")
        elf_patcher.patch(elf_file=elf_file)

    def test_patch_does_nothing_if_no_interpreter(self):
        elf_file = self.fake_elf["fake_elf-static"]
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")
        elf_patcher.patch(elf_file=elf_file)


class TestPatcherErrors(TestElfBase):
    def test_patch_fails_raises_patcherror_exception(self):
        elf_file = self.fake_elf["fake_elf-bad-patchelf"]
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")

        self.assertRaises(
            errors.PatcherGenericError, elf_patcher.patch, elf_file=elf_file
        )


class TestSonameCache(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.arch = ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")
        self.soname_cache = elf.SonameCache()

    def test_add_and_retrieve_soname_path(self):
        self.soname_cache[self.arch, "soname.so"] = "/fake/path/soname.so"
        self.assertThat(
            self.soname_cache[self.arch, "soname.so"], Equals("/fake/path/soname.so")
        )

    def test_add_and_verify_soname_path(self):
        self.soname_cache[self.arch, "soname.so"] = "/fake/path/soname.so"
        self.assertTrue((self.arch, "soname.so") in self.soname_cache)

    def test_reset_except_root(self):
        self.soname_cache[self.arch, "soname.so"] = "/fake/path/soname.so"
        self.soname_cache[self.arch, "soname2.so"] = "/keep/me/soname2.so"
        self.soname_cache[self.arch, "notfound.so"] = None

        self.assertTrue((self.arch, "soname.so") in self.soname_cache)
        self.assertTrue((self.arch, "soname2.so") in self.soname_cache)
        self.assertTrue((self.arch, "notfound.so") in self.soname_cache)

        self.soname_cache.reset_except_root("/keep/me")

        self.assertFalse((self.arch, "soname.so") in self.soname_cache)
        self.assertFalse((self.arch, "notfound.so") in self.soname_cache)
        self.assertTrue((self.arch, "soname2.so") in self.soname_cache)


class TestSonameCacheErrors:

    scenarios = (
        ("invalid string key", dict(key="soname.so", partial_message="The key for")),
        (
            "invalid first argument tuple",
            dict(
                key=(("ELFCLASS64", "ELFDATA2LSB"), "soname.so"),
                partial_message="The first element",
            ),
        ),
        (
            "invalid second argument type",
            dict(
                key=(("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64"), 1),
                partial_message="The second element",
            ),
        ),
    )

    def test_error(self, key, partial_message):
        soname_cache = elf.SonameCache()
        with pytest.raises(EnvironmentError) as error:
            soname_cache.__setitem__(key, "/soname.so")
            assert str(error).startswith(partial_message)


# This is just a subset
_LIBC6_LIBRARIES = [
    "ld-2.26.so",
    "ld-linux-x86-64.so.2",
    "libBrokenLocale-2.26.so",
    "libBrokenLocale.so.1",
    "libSegFault.so",
    "libanl-2.26.so",
]


class HandleGlibcTestCase(unit.TestCase):
    def _setup_libc6(self):
        lib_path = os.path.join(self.path, "lib")
        libraries = {os.path.join(lib_path, l) for l in _LIBC6_LIBRARIES}

        os.mkdir(lib_path)
        for library in libraries:
            open(library, "w").close()

        return libraries

    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.internal.repo.Repo.get_package_libraries")
        self.get_packages_mock = patcher.start()
        self.get_packages_mock.return_value = self._setup_libc6()
        self.addCleanup(patcher.stop)

    def test_glibc_mangling(self):
        dynamic_linker = elf.find_linker(
            root_path=self.path, snap_base_path="/snap/snap-name/current"
        )

        self.get_packages_mock.assert_called_once_with("libc6")

        self.assertThat(dynamic_linker, EndsWith("ld-2.26.so"))

    def test_bad_dynamic_linker_in_libc6_package(self):
        self.get_packages_mock.return_value = {"/usr/lib/dyn-linker-2.25.so"}
        self.assertRaises(
            RuntimeError,
            elf.find_linker,
            root_path=self.path,
            snap_base_path="/snap/snap-name/current",
        )


class TestLddParsing:
    scenarios = [
        (
            "ubuntu 20.04 basic",
            dict(
                ldd_output="""
\tlinux-vdso.so.1 (0x00007ffcae3e6000)
\tlibc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007f33eebeb000)
\t/lib64/ld-linux-x86-64.so.2 (0x00007f33eedf2000)
""",
                expected={"libc.so.6": "/lib/x86_64-linux-gnu/libc.so.6"},
            ),
        ),
        (
            "ubuntu 18.04 lspci w/o libpci",
            dict(
                ldd_output="""
\tlinux-vdso.so.1 (0x00007fffeddd1000)
\tlibpci.so.3 => not found
\tlibkmod.so.2 => /lib/x86_64-linux-gnu/libkmod.so.2 (0x00007fe500619000)
\tlibc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fe500228000)
\t/lib64/ld-linux-x86-64.so.2 (0x00007fe500a44000)
""",
                expected={
                    "libc.so.6": "/lib/x86_64-linux-gnu/libc.so.6",
                    "libkmod.so.2": "/lib/x86_64-linux-gnu/libkmod.so.2",
                    "libpci.so.3": "libpci.so.3",
                },
            ),
        ),
        (
            "ubuntu 16.04 basic",
            dict(
                ldd_output="""
\tlinux-vdso.so.1 =>  (0x00007ffd71d64000)
\tlibc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fda33a16000)
\t/lib64/ld-linux-x86-64.so.2 (0x00007fda33de0000)
    """,
                expected={"libc.so.6": "/lib/x86_64-linux-gnu/libc.so.6"},
            ),
        ),
        (
            "ubuntu 16.04 lspci w/o libpci",
            dict(
                ldd_output="""
\tlinux-vdso.so.1 =>  (0x00007fff305b3000)
\tlibpci.so.3 => not found
\tlibkmod.so.2 => /lib/x86_64-linux-gnu/libkmod.so.2 (0x00007faef225c000)
\tlibc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007faef1e92000)
\t/lib64/ld-linux-x86-64.so.2 (0x00007faef2473000)
    """,
                expected={
                    "libc.so.6": "/lib/x86_64-linux-gnu/libc.so.6",
                    "libkmod.so.2": "/lib/x86_64-linux-gnu/libkmod.so.2",
                    "libpci.so.3": "libpci.so.3",
                },
            ),
        ),
    ]

    def test_ldd(self, ldd_output, expected, monkeypatch, fake_process):
        monkeypatch.setattr(os.path, "exists", lambda f: True)
        monkeypatch.setattr(os.path, "abspath", lambda p: p)

        fake_process.register_subprocess(
            ["/usr/bin/ldd", "/bin/foo"], stdout=ldd_output.encode()
        )

        libraries = elf._determine_libraries(path="/bin/foo", ld_library_paths=[])

        assert libraries == expected

    def test_ldd_with_preload(
        self, ldd_output, expected, monkeypatch, fake_process, tmp_path
    ):
        libc_path = tmp_path / "libc.so.6"
        libc_path.write_text("")

        monkeypatch.setattr(os.path, "exists", lambda f: True)
        monkeypatch.setattr(os.path, "abspath", lambda p: p)
        monkeypatch.setattr(elf, "_get_host_libc_path", lambda: libc_path)

        fake_process.register_subprocess(["/usr/bin/ldd", "/bin/foo"], returncode=1)
        fake_process.register_subprocess(
            ["/usr/bin/ldd", "/bin/foo"], stdout=ldd_output.encode()
        )

        libraries = elf._determine_libraries(path="/bin/foo", ld_library_paths=[])

        assert libraries == expected

    def test_ld_trace(self, ldd_output, expected, monkeypatch, fake_process):
        monkeypatch.setattr(os.path, "exists", lambda f: True)
        monkeypatch.setattr(os.path, "abspath", lambda p: p)

        fake_process.register_subprocess(["/usr/bin/ldd", "/bin/foo"], returncode=1)
        fake_process.register_subprocess(["/usr/bin/ldd", "/bin/foo"], returncode=1)
        fake_process.register_subprocess(["/bin/foo"], stdout=ldd_output.encode())

        libraries = elf._determine_libraries(path="/bin/foo", ld_library_paths=[])

        assert libraries == expected


def test_ldd_with_preload_no_libc(monkeypatch, fake_process, tmp_path):
    monkeypatch.setattr(os.path, "exists", lambda f: True)
    monkeypatch.setattr(os.path, "abspath", lambda p: p)
    monkeypatch.setattr(elf, "_get_host_libc_path", lambda: tmp_path / "does-not-exist")

    fake_process.register_subprocess(["/usr/bin/ldd", "/bin/foo"], returncode=1)
    fake_process.register_subprocess(
        ["/bin/foo"], stdout="\tlibx.so.0 => /lib/libx.so (0x0123)".encode()
    )

    libraries = elf._determine_libraries(path="/bin/foo", ld_library_paths=[])

    assert libraries == {"libx.so.0": "/lib/libx.so"}


def test_ld_trace_os_error_for_wrong_arch(monkeypatch, fake_process, tmp_path):
    monkeypatch.setattr(os.path, "exists", lambda f: True)
    monkeypatch.setattr(os.path, "abspath", lambda p: p)
    monkeypatch.setattr(elf, "_get_host_libc_path", lambda: tmp_path / "does-not-exist")

    fake_process.register_subprocess(["/usr/bin/ldd", "/bin/foo"], returncode=1)

    def raise_os_error(*args, **kwargs):
        raise OSError(8, "Exec format error:", "does-not-exist")

    fake_process.register_subprocess(["/bin/foo"], callback=raise_os_error)

    libraries = elf._determine_libraries(path="/bin/foo", ld_library_paths=[])

    assert libraries == {}
