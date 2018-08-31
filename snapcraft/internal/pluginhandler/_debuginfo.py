# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import subprocess
from typing import List

from snapcraft import file_utils
from snapcraft.internal import elf, errors

logger = logging.getLogger(__name__)


class DebugInfoCollector:
    """DebugInfoCollector detaches and strips debug info from ELF files."""

    def __init__(self, *, debug_dir: str) -> None:
        self._debug_dir = debug_dir

        self._objcopy_cmd = file_utils.get_tool_path("objcopy")
        self._strip_cmd = file_utils.get_tool_path("strip")

    def _debug_info_file(self, *, elf_file: elf.ElfFile) -> str:
        build_id = elf_file.build_id
        assert build_id != ""
        return os.path.join(
            self._debug_dir, ".build-id", build_id[:2], "{}.debug".format(build_id[2:])
        )

    def separate(self, *, elf_file: elf.ElfFile) -> None:
        assert elf_file.has_debug_info
        debug_file = self._debug_info_file(elf_file=elf_file)
        os.makedirs(os.path.dirname(debug_file), exist_ok=True)

        self._run(
            [
                self._objcopy_cmd,
                "--only-keep-debug",
                "--compress-debug-sections",
                elf_file.path,
                debug_file,
            ]
        )
        # FIXME: consider being a bit more aggressive in stripping executable.
        self._run([self._strip_cmd, "-g", elf_file.path])
        self._run([self._objcopy_cmd, "--add-gnu-debuglink", debug_file, elf_file.path])

    def _run(self, args: List[str]) -> None:
        try:
            subprocess.check_call(args)
        except subprocess.CalledProcessError as call_error:
            raise errors.SnapcraftCommandError(
                command=" ".join(args), call_error=call_error
            ) from call_error

    def separate_tree(self, *, base_dir: str) -> None:
        for directory, _, files in os.walk(base_dir):
            for file_name in files:
                if file_name.endswith(".o"):
                    continue
                file_path = os.path.join(directory, file_name)
                if not elf.ElfFile.is_elf(file_path):
                    continue

                elf_file = elf.ElfFile(path=file_path)
                if elf_file.has_debug_info:
                    if elf_file.build_id != "":
                        self.separate(elf_file=elf_file)
                    else:
                        # FIXME: consider how to handle files without
                        # build-id, given we don't know the install
                        # location ahead of time.
                        logger.warn("No build ID for {}".format(file_path))
                else:
                    # FIXME: this will include files from staged
                    # packages, which we should be able to locate
                    # debug info for.
                    logger.warn("No debug info found for {}".format(file_path))
