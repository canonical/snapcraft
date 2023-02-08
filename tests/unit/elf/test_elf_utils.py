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

import os
from pathlib import Path

import pytest

from snapcraft.elf import elf_utils
from snapcraft.errors import SnapcraftError


@pytest.fixture(autouse=True, scope="function")
def _setup_method():
    elf_utils.get_elf_files.cache_clear()


class TestGetElfFiles:
    """get_elf_files functionality."""

    def test_get_elf_files(self, new_dir, fake_elf):
        fake_elf("fake_elf-2.23")
        fake_elf("object_file.o")
        fake_elf("fake_elf-static")
        elf_files = elf_utils.get_elf_files(new_dir)
        assert len(elf_files) == 1

        elf_file = elf_files.pop()
        assert elf_file.interp == "/lib64/ld-linux-x86-64.so.2"

    def test_get_elf_files_from_list_from_list(self, new_dir, fake_elf):
        fake_elf("fake_elf-2.23")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, ["fake_elf-2.23"])
        assert len(elf_files) == 1

        elf_file = elf_files.pop()
        assert elf_file.interp == "/lib64/ld-linux-x86-64.so.2"

    def test_skip_object_files(self, new_dir, fake_elf):
        fake_elf("object_file.o")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, ["object_file.o"])
        assert elf_files == []

    def test_no_find_dependencies_statically_linked(self, new_dir, fake_elf):
        fake_elf("fake_elf-static")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, ["fake_elf-static"])
        assert elf_files == []

    def test_elf_with_execstack(self, new_dir, fake_elf):
        fake_elf("fake_elf-with-execstack")
        elf_files = elf_utils.get_elf_files_from_list(
            new_dir, {"fake_elf-with-execstack"}
        )
        elf_file = elf_files.pop()
        assert elf_file.execstack_set is True

    def test_elf_without_execstack(self, new_dir, fake_elf):
        fake_elf("fake_elf-2.23")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, {"fake_elf-2.23"})
        elf_file = elf_files.pop()
        assert elf_file.execstack_set is False

    def test_non_elf_files(self, new_dir):
        # A bz2 header
        Path("non-elf").write_bytes(b"\x42\x5a\x68")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, {"non-elf"})
        assert elf_files == []

    def test_symlinks(self, new_dir):
        symlinked_path = Path(new_dir, "symlinked")
        symlinked_path.symlink_to("/bin/dash")
        elf_files = elf_utils.get_elf_files_from_list(new_dir, {"symlinked"})
        assert elf_files == []

    def test_device_files(self):
        elf_files = elf_utils.get_elf_files_from_list(Path("/dev"), {"null"})
        assert elf_files == []

    def test_fifo(self, new_dir):
        fifo_path = os.path.join(new_dir, "fifo")
        os.mkfifo(fifo_path)
        elf_files = elf_utils.get_elf_files_from_list(new_dir, {"fifo"})
        assert elf_files == []


class TestGetDynamicLinker:
    """find_linker functionality."""

    @pytest.mark.parametrize(
        "arch,linker",
        [
            ("x86_64", "lib64/ld-linux-x86-64.so.2"),
            ("aarch64", "lib/ld-linux-aarch64.so.1"),
            ("armv7l", "lib/ld-linux-armhf.so.3"),
            ("riscv64", "lib/ld-linux-riscv64-lp64d.so.1"),
            ("ppc64le", "lib64/ld64.so.2"),
            ("s390x", "lib/ld64.so.1"),
        ],
    )
    def test_get_dynamic_linker(self, mocker, new_dir, arch, linker):
        mocker.patch("platform.machine", return_value=arch)

        lpath = Path(linker)
        lpath.parent.mkdir(parents=True)
        lpath.touch()

        dynamic_linker = elf_utils.get_dynamic_linker(
            root_path=new_dir, snap_path=Path("/snap/foo/current")
        )
        assert dynamic_linker == f"/snap/foo/current/{linker}"

    def test_get_dynamic_linker_undefined(self, mocker):
        mocker.patch("platform.machine", return_value="z80")

        with pytest.raises(RuntimeError) as err:
            elf_utils.get_dynamic_linker(
                root_path=Path("prime"), snap_path=Path("/snap/foo/current")
            )

        assert str(err.value) == "Dynamic linker not defined for arch 'z80'"

    def test_get_dynamic_linker_not_found(self, mocker):
        mocker.patch("platform.machine", return_value="x86_64")

        with pytest.raises(SnapcraftError) as err:
            elf_utils.get_dynamic_linker(
                root_path=Path("prime"), snap_path=Path("/snap/foo/current")
            )

        assert str(err.value) == (
            "Dynamic linker 'prime/lib64/ld-linux-x86-64.so.2' not found."
        )


class TestArchConfig:
    """Test architecture config functionality."""

    @pytest.mark.parametrize(
        "machine, expected_arch_triplet",
        [
            ("aarch64", "aarch64-linux-gnu"),
            ("armv7l", "arm-linux-gnueabihf"),
            ("ppc64le", "powerpc64le-linux-gnu"),
            ("riscv64", "riscv64-linux-gnu"),
            ("s390x", "s390x-linux-gnu"),
            ("x86_64", "x86_64-linux-gnu"),
        ],
    )
    def test_get_arch_triplet(self, mocker, machine, expected_arch_triplet):
        """Verify `get_arch_triplet()` gets the host's architecture triplet."""
        mocker.patch("snapcraft.elf.elf_utils.platform.machine", return_value=machine)
        arch_triplet = elf_utils.get_arch_triplet()

        assert arch_triplet == expected_arch_triplet

    def test_get_arch_triplet_error(self, mocker):
        """Verify `get_arch_triplet()` raises an error for invalid machines."""
        mocker.patch("snapcraft.elf.elf_utils.platform.machine", return_value="4004")
        with pytest.raises(RuntimeError) as raised:
            elf_utils.get_arch_triplet()

        assert str(raised.value) == "Arch triplet not defined for arch '4004'"

    def test_get_all_arch_triplets(self):
        """Verify `get_all_arch_triplets()` gets a list of all architecture triplets."""
        arch_triplets = elf_utils.get_all_arch_triplets()

        assert arch_triplets == [
            "aarch64-linux-gnu",
            "arm-linux-gnueabihf",
            "powerpc64le-linux-gnu",
            "riscv64-linux-gnu",
            "s390x-linux-gnu",
            "x86_64-linux-gnu",
        ]
