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
import shutil
from pathlib import Path
from unittest.mock import ANY, call

import pytest

from snapcraft import elf
from snapcraft.elf import errors

PATCHELF_PATH = "/path/to/patchelf"


@pytest.mark.usefixtures("fake_tools")
def test_patcher(fake_elf):
    elf_file = fake_elf("fake_elf-2.23")
    # The base_path does not matter here as there are not files to
    # be crawled for.
    elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path=Path("/fake"))
    elf_patcher.patch(elf_file=elf_file)


@pytest.mark.usefixtures("fake_tools")
def test_patcher_does_nothing_if_no_interpreter(fake_elf):
    elf_file = fake_elf("fake_elf-static")
    # The base_path does not matter here as there are not files to
    # be crawled for.
    elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path=Path("/fake"))
    elf_patcher.patch(elf_file=elf_file)


@pytest.mark.usefixtures("fake_tools")
def test_patcher_fails_raises_patcherror_exception(fake_elf):
    elf_file = fake_elf("fake_elf-bad-patchelf")
    # The base_path does not matter here as there are not files to
    # be crawled for.
    elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path=Path("/fake"))

    with pytest.raises(errors.PatcherError) as raised:
        elf_patcher.patch(elf_file=elf_file)
    assert raised.value.path == Path("fake_elf-bad-patchelf")
    assert raised.value.code == 1


@pytest.fixture
def elf_file(new_dir):
    base_path = new_dir / "core"
    base_path.mkdir()

    elf_path = Path(new_dir / "elf")
    shutil.copy("/bin/true", elf_path)
    elf_file = elf.ElfFile(path=elf_path)

    # Use base path as / so that the elf file is inside the base
    elf_file.load_dependencies(
        root_path=Path("/snap/foo/current"),
        base_path=Path("/"),
        content_dirs=[],
        arch_triplet="x86_64-linux-gnu",
    )
    yield elf_file


@pytest.fixture
def patcher():
    yield elf.Patcher(
        dynamic_linker="/my/dynamic/linker",
        root_path=Path("/snap/foo/current"),
        preferred_patchelf=PATCHELF_PATH,
    )


def test_patcher_patch_rpath(mocker, patcher, elf_file):
    run_mock = mocker.patch("subprocess.check_call")
    mocker.patch("subprocess.check_output", return_value=b"\n")
    expected_proposed_rpath = list(elf_file.dependencies)[0].path.parent
    assert patcher.get_current_rpath(elf_file) == []

    patcher.patch(elf_file=elf_file)
    assert run_mock.mock_calls == [
        call(
            [
                PATCHELF_PATH,
                "--set-interpreter",
                "/my/dynamic/linker",
                "--force-rpath",
                "--set-rpath",
                str(expected_proposed_rpath),
                ANY,
            ]
        )
    ]


def test_patcher_patch_existing_rpath_origin(mocker, patcher, elf_file):
    run_mock = mocker.patch("subprocess.check_call")
    mocker.patch(
        "snapcraft.elf._patcher.Patcher.get_current_rpath",
        return_value=["$ORIGIN/current/rpath"],
    )

    expected_proposed_rpath = list(elf_file.dependencies)[0].path.parent
    assert patcher.get_current_rpath(elf_file) == ["$ORIGIN/current/rpath"]

    patcher.patch(elf_file=elf_file)
    assert run_mock.mock_calls == [
        call(
            [
                PATCHELF_PATH,
                "--set-interpreter",
                "/my/dynamic/linker",
                "--force-rpath",
                "--set-rpath",
                f"$ORIGIN/current/rpath:{str(expected_proposed_rpath)}",
                ANY,
            ]
        )
    ]


def test_patcher_patch_existing_rpath_not_origin(mocker, patcher, elf_file):
    run_mock = mocker.patch("subprocess.check_call")
    mocker.patch(
        "snapcraft.elf._patcher.Patcher.get_current_rpath",
        return_value=["/current/rpath"],
    )

    expected_proposed_rpath = list(elf_file.dependencies)[0].path.parent
    assert patcher.get_current_rpath(elf_file) == ["/current/rpath"]

    patcher.patch(elf_file=elf_file)
    assert run_mock.mock_calls == [
        call(
            [
                PATCHELF_PATH,
                "--set-interpreter",
                "/my/dynamic/linker",
                "--force-rpath",
                "--set-rpath",
                str(expected_proposed_rpath),
                ANY,
            ]
        )
    ]


def test_patcher_patch_rpath_same_interpreter(mocker, patcher, elf_file):
    run_mock = mocker.patch("subprocess.check_call")
    mocker.patch("subprocess.check_output", return_value=b"\n")
    patcher._dynamic_linker = elf_file.interp

    expected_proposed_rpath = list(elf_file.dependencies)[0].path.parent
    assert patcher.get_current_rpath(elf_file) == []

    patcher.patch(elf_file=elf_file)
    assert run_mock.mock_calls == [
        call(
            [
                PATCHELF_PATH,
                "--force-rpath",
                "--set-rpath",
                str(expected_proposed_rpath),
                ANY,
            ]
        )
    ]


def test_patcher_patch_rpath_already_set(mocker, patcher, elf_file):
    run_mock = mocker.patch("subprocess.check_call")

    expected_proposed_rpath = list(elf_file.dependencies)[0].path.parent
    mocker.patch(
        "snapcraft.elf._patcher.Patcher.get_current_rpath",
        return_value=["$ORIGIN/current/rpath", str(expected_proposed_rpath)],
    )

    patcher.patch(elf_file=elf_file)
    assert run_mock.mock_calls == [
        call(
            [
                PATCHELF_PATH,
                "--set-interpreter",
                "/my/dynamic/linker",
                ANY,
            ]
        )
    ]
