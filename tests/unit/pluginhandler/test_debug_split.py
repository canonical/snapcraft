# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
from unittest import mock

import fixtures
from testtools.matchers import Equals

from snapcraft import ProjectOptions
from snapcraft.internal.pluginhandler._debug_split import (
    DebugSplitter,
    split_debug_info,
)
from tests import unit


original_stat = os.stat


def fake_stat(path):
    basename = os.path.basename(path)
    if basename == "fake-elf-executable":
        return mock.Mock(st_mode=0o755)
    elif basename.startswith("fake-"):
        return mock.Mock(st_mode=0o644)
    else:
        return original_stat(path)


class FakeElfFile:
    def __init__(self, path: str) -> None:
        basename = os.path.basename(path)

        self.path = path
        self.build_id = f"fbid-{basename}"
        self.elf_type = "ET_EXEC"
        self.has_debug_info = True

        if basename == "fake-elf-shared-object":
            self.elf_type = "ET_DYN"
        elif basename == "fake-elf-core":
            self.elf_type = "ET_CORE"
        elif basename == "fake-elf-no-debug-info":
            self.has_debug_info = False
        elif basename == "fake-elf-no-build-id":
            self.build_id = ""

    @classmethod
    def is_elf(cls, path: str) -> bool:
        basename = os.path.basename(path)
        if basename.startswith("fake-elf"):
            return True
        return False


class SplitDebugTests(unit.TestCase):
    def setUp(self):
        super().setUp()

        # Shared properties.
        self.arch_triplet = ProjectOptions().arch_triplet
        self.debug_dir = Path(self.path)
        self.strip_cmd = f"{self.arch_triplet}-strip"
        self.objcopy_cmd = f"{self.arch_triplet}-objcopy"

        # Mock _debug_split._run().
        self.fake_run = fixtures.MockPatch(
            "snapcraft.internal.pluginhandler._debug_split._run"
        )
        self.useFixture(self.fake_run)

        # Mock os.stat().
        self.useFixture(fixtures.MockPatch("os.stat", side_effect=fake_stat))

        # Mock ElfFile.
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.pluginhandler._debug_split.ElfFile", new=FakeElfFile
            )
        )

    def test_no_debug_info(self):
        elf = FakeElfFile("fake-elf-no-debug-info")

        splitter = DebugSplitter(
            arch_triplet=self.arch_triplet, debug_dir=self.debug_dir
        )
        split_path = splitter.split(elf)

        self.assertThat(split_path, Equals(None))

    def test_no_build_id(self):
        elf = FakeElfFile("fake-elf-no-build-id")

        splitter = DebugSplitter(
            arch_triplet=self.arch_triplet, debug_dir=self.debug_dir
        )
        split_path = splitter.split(elf)

        self.assertThat(split_path, Equals(None))

    def test_ignored_elf_type(self):
        elf = FakeElfFile("fake-elf-core")

        splitter = DebugSplitter(
            arch_triplet=self.arch_triplet, debug_dir=self.debug_dir
        )
        split_path = splitter.split(elf)

        self.assertThat(split_path, Equals(None))

    def test_executable(self):
        elf = FakeElfFile("fake-elf-executable")

        expected_dst = Path(self.debug_dir, "fb/id-fake-elf-executable")

        splitter = DebugSplitter(
            arch_triplet=self.arch_triplet, debug_dir=self.debug_dir
        )
        split_path = splitter.split(elf)

        self.assertThat(split_path, Equals(expected_dst))

        self.fake_run.mock.assert_has_calls(
            [
                mock.call(
                    [
                        self.objcopy_cmd,
                        "--only-keep-debug",
                        "--compress-debug-sections",
                        elf.path,
                        expected_dst.as_posix(),
                    ]
                ),
                mock.call(
                    [
                        self.strip_cmd,
                        "--remove-section=.comment",
                        "--remove-section=.note",
                        elf.path,
                    ]
                ),
                mock.call(
                    [
                        self.objcopy_cmd,
                        "--add-gnu-debuglink",
                        expected_dst.as_posix(),
                        elf.path,
                    ]
                ),
            ]
        )

    def test_shared_object(self):
        elf = FakeElfFile("fake-elf-shared-object")

        expected_dst = Path(self.debug_dir, "fb/id-fake-elf-shared-object")

        splitter = DebugSplitter(
            arch_triplet=self.arch_triplet, debug_dir=self.debug_dir
        )
        split_path = splitter.split(elf)

        self.assertThat(split_path, Equals(expected_dst))

        self.fake_run.mock.assert_has_calls(
            [
                mock.call(
                    [
                        self.objcopy_cmd,
                        "--only-keep-debug",
                        "--compress-debug-sections",
                        elf.path,
                        expected_dst.as_posix(),
                    ]
                ),
                mock.call(
                    [
                        self.strip_cmd,
                        "--remove-section=.comment",
                        "--remove-section=.note",
                        "--strip-unneeded",
                        elf.path,
                    ]
                ),
                mock.call(
                    [
                        self.objcopy_cmd,
                        "--add-gnu-debuglink",
                        expected_dst.as_posix(),
                        elf.path,
                    ]
                ),
            ]
        )

    def test_split_debug_info(self):
        file_paths = set(
            [
                "fake-elf-executable",
                "fake-elf-shared-object",
                "fake-elf-core",
                "fake-elf-no-debug-info",
                "fake-elf-no-build-id",
                "fake-data",
            ]
        )

        debug_files, debug_dirs = split_debug_info(
            debug_dir=self.debug_dir,
            arch_triplet=self.arch_triplet,
            prime_dir=self.path,
            file_paths=file_paths,
        )

        self.assertThat(
            debug_files,
            Equals({"fb/id-fake-elf-executable", "fb/id-fake-elf-shared-object"}),
        )
        self.assertThat(debug_dirs, Equals({"fb"}))
