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
import sys

from testtools.matchers import Contains, EndsWith, Equals, NotEquals, StartsWith
from unittest import mock

from snapcraft.internal import errors, elf
from tests import unit, fixture_setup
from tests.fixture_setup.os_release import FakeOsRelease


class TestElfBase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_elf = fixture_setup.FakeElf(root_path=self.path)
        self.useFixture(self.fake_elf)


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

        # Python is not a shared library, so has no soname
        self.assertThat(elf_file.soname, Equals(""))

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


class TestGetLibraries(TestElfBase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.internal.elf._get_system_libs")
        self.get_system_libs_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.get_system_libs_mock.return_value = frozenset()

        patcher = mock.patch("os.path.exists")
        self.path_exists_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.path_exists_mock.return_value = True

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_get_libraries(self):
        elf_file = self.fake_elf["fake_elf-2.23"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
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
        )

        self.assertThat(libs, Equals(frozenset(["/lib/foo.so.1", "/usr/lib/bar.so.2"])))

    def test_get_libraries_excludes_slash_snap(self):
        elf_file = self.fake_elf["fake_elf-with-core-libs"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
        )

        self.assertThat(
            libs,
            Equals(
                set([self.fake_elf.root_libraries["foo.so.1"], "/usr/lib/bar.so.2"])
            ),
        )

    def test_get_libraries_filtered_by_system_libraries(self):
        self.get_system_libs_mock.return_value = frozenset(["foo.so.1"])

        elf_file = self.fake_elf["fake_elf-2.23"]
        libs = elf_file.load_dependencies(
            root_path="/", core_base_path="/snap/core/current"
        )
        self.assertThat(libs, Equals(frozenset(["/usr/lib/bar.so.2"])))

    def test_get_libraries_ldd_failure_logs_warning(self):
        elf_file = self.fake_elf["fake_elf-bad-ldd"]
        libs = elf_file.load_dependencies(
            root_path=self.fake_elf.root_path,
            core_base_path=self.fake_elf.core_base_path,
        )

        self.assertThat(libs, Equals(set()))
        self.assertThat(
            self.fake_logger.output,
            Contains("Unable to determine library dependencies for"),
        )


class TestSystemLibsOnNewRelease(TestElfBase):
    def setUp(self):
        super().setUp()
        self.useFixture(FakeOsRelease())

    def test_fail_gracefully_if_system_libs_not_found(self):
        elf_file = self.fake_elf["fake_elf-2.23"]
        libs = elf_file.load_dependencies(
            root_path="/fake", core_base_path="/fake-core"
        )
        self.assertThat(libs, Equals(frozenset()))


class TestSystemLibsOnReleasesWithNoVersionId(unit.TestCase):
    def setUp(self):
        super().setUp()

        elf._libraries = None
        self.useFixture(FakeOsRelease(version_id=None))

    @mock.patch(
        "snapcraft.internal.elf.repo.Repo.get_package_libraries",
        return_value=["/usr/lib/libc.so.6", "/lib/libpthreads.so.6"],
    )
    def test_fail_gracefully_if_no_version_id_found(self, mock_package_libs):
        self.assertThat(
            elf._get_system_libs(), Equals(frozenset(["libc.so.6", "libpthreads.so.6"]))
        )


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

    def test_patch_fails_with_old_version(self):
        self.fake_elf = fixture_setup.FakeElf(
            root_path=self.path, patchelf_version="0.8"
        )
        self.useFixture(self.fake_elf)

        elf_file = self.fake_elf["fake_elf-bad-patchelf"]
        # The base_path does not matter here as there are not files to
        # be crawled for.
        elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")

        with mock.patch(
            "subprocess.check_call", wraps=subprocess.check_call
        ) as mock_check_call:
            self.assertRaises(
                errors.PatcherNewerPatchelfError, elf_patcher.patch, elf_file=elf_file
            )

            # Test that .note.go.buildid is stripped off
            mock_check_call.assert_has_calls(
                [
                    mock.call(
                        ["patchelf", "--set-interpreter", "/lib/fake-ld", mock.ANY]
                    ),
                    mock.call(
                        ["strip", "--remove-section", ".note.go.buildid", mock.ANY]
                    ),
                    mock.call(
                        ["patchelf", "--set-interpreter", "/lib/fake-ld", mock.ANY]
                    ),
                ]
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


class TestSonameCacheErrors(unit.TestCase):

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

    def test_error(self):
        soname_cache = elf.SonameCache()
        raised = self.assertRaises(
            EnvironmentError, soname_cache.__setitem__, self.key, "/soname.so"
        )

        self.assertThat(str(raised), StartsWith(self.partial_message))


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
