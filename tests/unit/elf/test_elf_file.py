# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2022 Canonical Ltd.
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

from pathlib import Path

import pytest

from snapcraft import elf
from snapcraft.elf import elf_utils, errors
from snapcraft.elf._elf_file import _Library


class TestElfFileSmoketest:
    """Basic ElfFile functionality."""

    def test_bin_echo(self):
        # Try parsing a file without the pyelftools logic mocked out
        elf_file = elf.ElfFile(path=Path("/bin/ls"))

        assert elf_file.path == Path("/bin/ls")

        # The arch attribute will be a tuple of three strings
        assert isinstance(elf_file.arch_tuple, tuple)
        assert len(elf_file.arch_tuple) == 3
        assert elf_file.arch_tuple[0].startswith("ELFCLASS")
        assert elf_file.arch_tuple[1].startswith("ELFDATA")
        assert elf_file.arch_tuple[2].startswith("EM_")

        # We expect /bin/ls to be a dynamic linked executable with an ELF interpreter
        assert isinstance(elf_file.interp, str)
        assert elf_file.interp != ""

        # /bin/ls is not a shared library, so has no soname or defined versions
        assert elf_file.soname == ""
        assert elf_file.versions == set()

        # We expect that /bin/ls will be linked to libc
        lib = None
        for lib in elf_file.needed.values():
            if lib.name.startswith("libc.so"):
                break
        else:
            pytest.fail("Expected to find libc in needed library list")

        assert isinstance(lib.name, str) is True
        for version in lib.versions:
            assert isinstance(version, str)

        # GCC adds a build ID to executables
        assert elf_file.build_id != ""

        # If /bin/ls distro packaged, it probably doesn't have debug info, but we don't
        # know for sure. Instead just check that it is a boolean.
        assert isinstance(elf_file.has_debug_info, bool)

        # Ensure type is detered as executable.
        assert elf_file.elf_type == "ET_DYN"

    def test_invalid_elf_file(self, new_dir):
        Path("invalid-elf").write_bytes(b"\x7fELF\x00")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, ["invalid-elf"])
        assert elf_files == []


class TestGetLibraries:
    """ELF file libraries."""

    @pytest.fixture(autouse=True)
    def setup_fixture(self, mocker, fake_tools):
        mocker.patch("os.path.exists", return_value=True)

    def test_get_libraries(self, new_dir, fake_elf, fake_libs):
        elf_file = fake_elf("fake_elf-2.23")
        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        # bar.so.2 comes from fake ldd result
        assert libs == set([str(fake_libs["foo.so.1"]), "/usr/lib/bar.so.2"])

    def test_get_libraries_missing_libs(self, new_dir, fake_elf, fake_libs):
        elf_file = fake_elf("fake_elf-with-missing-libs")
        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        assert libs == {str(fake_libs["foo.so.1"]), "missing.so.2"}

    def test_get_libraries_with_soname_cache(self, new_dir, fake_elf, fake_libs):
        elf_file = fake_elf("fake_elf-2.23")

        arch = ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")
        soname_cache = elf.SonameCache()
        soname_cache[arch, "bar.so.2"] = Path("/lib/bar.so.2")

        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
            soname_cache=soname_cache,
        )

        # With no cache this would have returned '/usr/lib/bar.so.2'
        assert libs == {str(fake_libs["foo.so.1"]), "/lib/bar.so.2"}

    def test_primed_libraries_are_preferred(self, new_dir, fake_elf, fake_libs):
        elf_file = fake_elf("fake_elf-2.23")
        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        assert libs == frozenset([str(fake_libs["foo.so.1"]), "/usr/lib/bar.so.2"])

    def test_non_elf_primed_sonames_matches_are_ignored(self, new_dir, fake_elf):
        primed_foo = new_dir / "foo.so.1"
        primed_foo.write_bytes(b"\x42\x5a\x68")  # a bz2 header

        elf_file = fake_elf("fake_elf-2.23")
        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        assert libs == frozenset(["/lib/foo.so.1", "/usr/lib/bar.so.2"])

    def test_get_libraries_excludes_slash_snap(self, new_dir, fake_elf, fake_libs):
        elf_file = fake_elf("fake_elf-with-core-libs")
        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        assert libs == {str(fake_libs["foo.so.1"]), "/usr/lib/bar.so.2"}

    def test_get_libraries_ldd_failure_logs_warning(self, emitter, new_dir, fake_elf):
        elf_file = fake_elf("fake_elf-bad-ldd")
        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        assert libs == set()
        emitter.assert_progress(
            "Unable to determine library dependencies for 'fake_elf-bad-ldd'",
            permanent=True,
        )

    def test_existing_host_library_searched_for(
        self, mocker, new_dir, fake_elf, fake_libs
    ):
        elf_file = fake_elf("fake_elf-with-host-libraries")

        class _MooLibrary(_Library):
            """A Library implementation that always returns valid for moo."""

            def _is_valid_elf(self, resolved_path: Path) -> bool:
                #  This path is defined in ldd for fake_elf-with-host-libraries.
                if resolved_path == Path("/usr/lib/moo.so.2"):
                    return True

                return super()._is_valid_elf(resolved_path)

        mocker.patch("snapcraft.elf._elf_file._Library", side_effect=_MooLibrary)

        libs = elf_file.load_dependencies(
            root_path=new_dir,
            base_path=new_dir / "core",
            arch_triplet="x86_64-linux-gnu",
            content_dirs=[],
        )

        assert libs == {str(fake_libs["moo.so.2"])}


class TestLibrary:
    """Verify the _Library class."""

    def test_is_valid_elf_ignores_corrupt_files(self, mocker, new_dir, fake_elf):
        soname = "libssl.so.1.0.0"
        soname_path = new_dir / soname
        fake_elf(soname)

        library = _Library(
            soname=soname,
            soname_path=soname_path,
            search_paths=[new_dir],
            base_path=Path("/snap/core20/current"),
            arch_tuple=("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64"),
            soname_cache=elf.SonameCache(),
        )

        assert library._is_valid_elf(soname_path) is True

        mocker.patch(
            "snapcraft.elf._elf_file.ElfFile",
            side_effect=errors.CorruptedElfFile(path=soname_path, error=RuntimeError()),
        )

        assert library._is_valid_elf(soname_path) is False


class TestGetRequiredGLIBC:
    """ELF file glibc required versions."""

    def test_get_required_glibc(self, fake_elf):
        elf_file = fake_elf("fake_elf-2.23")
        assert elf_file.get_required_glibc() == "2.23"

    def test_linker_version_greater_than_required_glibc(self, fake_elf):
        elf_file = fake_elf("fake_elf-2.23")
        assert elf_file.is_linker_compatible(linker_version="2.26") is True

    def test_linker_version_equals_required_glibc(self, fake_elf):
        elf_file = fake_elf("fake_elf-2.23")
        assert elf_file.is_linker_compatible(linker_version="2.23") is True

    def test_linker_version_less_than_required_glibc(self, fake_elf):
        elf_file = fake_elf("fake_elf-2.23")
        assert elf_file.is_linker_compatible(linker_version="1.2") is False


class TestElfFileAttrs:
    """ELF file interpreter, soname, and other attributes."""

    def test_executable(self, fake_elf):
        elf_file = fake_elf("fake_elf-2.23")

        assert elf_file.interp == "/lib64/ld-linux-x86-64.so.2"
        assert elf_file.soname == ""
        assert list(elf_file.needed.keys()) == ["libc.so.6"]

        glibc = elf_file.needed["libc.so.6"]
        assert glibc.name == "libc.so.6"
        assert glibc.versions == {"GLIBC_2.2.5", "GLIBC_2.23"}

    def test_shared_object(self, fake_elf):
        # fake_elf-shared-object has no GLIBC dependency, but two symbols
        # nonetheless
        elf_file = fake_elf("fake_elf-shared-object")

        assert elf_file.interp == ""
        assert elf_file.soname == "libfake_elf.so.0"
        assert list(elf_file.needed.keys()) == ["libssl.so.1.0.0"]

        openssl = elf_file.needed["libssl.so.1.0.0"]
        assert openssl.name == "libssl.so.1.0.0"
        assert openssl.versions == {"OPENSSL_1.0.0"}

    def test_libssl(self, fake_elf):
        # libssl.so.1.0.0 defines some symbol versions it provides
        elf_file = fake_elf("libssl.so.1.0.0")

        assert elf_file.interp == ""
        assert elf_file.soname == "libssl.so.1.0.0"
        assert elf_file.versions == {"libssl.so.1.0.0", "OPENSSL_1.0.0"}


class TestSonameCache:
    """SonameCache functionality."""

    @pytest.fixture
    def arch(self):
        yield ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")

    def test_add_and_retrieve_soname_path(self, arch):
        soname_cache = elf.SonameCache()
        soname_cache[arch, "soname.so"] = Path("/fake/path/soname.so")
        assert soname_cache[arch, "soname.so"] == Path("/fake/path/soname.so")

    def test_add_and_verify_soname_path(self, arch):
        soname_cache = elf.SonameCache()
        soname_cache[arch, "soname.so"] = Path("/fake/path/soname.so")
        assert (arch, "soname.so") in soname_cache

    def test_reset_except_root(self, arch):
        soname_cache = elf.SonameCache()
        soname_cache[arch, "soname.so"] = Path("/fake/path/soname.so")
        soname_cache[arch, "soname2.so"] = Path("/keep/me/soname2.so")
        soname_cache[arch, "notfound.so"] = None  # type: ignore

        assert (arch, "soname.so") in soname_cache
        assert (arch, "soname2.so") in soname_cache
        assert (arch, "notfound.so") in soname_cache

        soname_cache.reset_except_root("/keep/me")

        assert (arch, "soname.so") not in soname_cache
        assert (arch, "notfound.so") not in soname_cache
        assert (arch, "soname2.so") in soname_cache

    @pytest.mark.parametrize(
        "key,partial_message",
        [
            # Invalid string key
            ("soname.so", "The key for"),
            # Invalid first argument tuple
            ((("ELFCLASS64", "ELFDATA2LSB"), "soname.so"), "The first element"),
            # Invalid second argument type
            ((("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64"), 1), "The second element"),
        ],
    )
    def test_error(self, key, partial_message):
        soname_cache = elf.SonameCache()
        with pytest.raises(EnvironmentError) as raised:
            soname_cache[key] = Path("/soname.so")
            assert str(raised.value).startswith(partial_message)
