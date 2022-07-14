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
from snapcraft.elf import errors


def test_patcher(fake_elf, fake_tools):
    elf_file = fake_elf("fake_elf-2.23")
    # The base_path does not matter here as there are not files to
    # be crawled for.
    elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")
    elf_patcher.patch(elf_file=elf_file)


def test_patcher_does_nothing_if_no_interpreter(fake_elf, fake_tools):
    elf_file = fake_elf("fake_elf-static")
    # The base_path does not matter here as there are not files to
    # be crawled for.
    elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")
    elf_patcher.patch(elf_file=elf_file)


def test_patcher_fails_raises_patcherror_exception(fake_elf, fake_tools):
    elf_file = fake_elf("fake_elf-bad-patchelf")
    # The base_path does not matter here as there are not files to
    # be crawled for.
    elf_patcher = elf.Patcher(dynamic_linker="/lib/fake-ld", root_path="/fake")

    with pytest.raises(errors.PatcherError) as raised:
        elf_patcher.patch(elf_file=elf_file)
    assert raised.value.path == Path("fake_elf-bad-patchelf")
    assert raised.value.code == 1
