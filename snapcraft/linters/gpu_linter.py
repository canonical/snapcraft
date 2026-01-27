# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""GPU linter implementation."""

import fnmatch
from pathlib import Path

from overrides import overrides

from snapcraft.elf import elf_utils

from .base import Linter, LinterIssue, LinterResult

_HELP_URLS = {
    "core22": "https://canonical-ubuntu-frame-documentation.readthedocs-hosted.com/how-to/use-snap-graphics-on-base-core22/",
    "core24": "https://canonical-ubuntu-frame-documentation.readthedocs-hosted.com/how-to/use-snap-graphics-on-base-core24/",
}

_CORE22_PATTERNS = {
    "dri/*_dri.so",
    "dri/*_drv_video.so",
    "libEGL.so.1",
    "libEGL_*",
    "libGL.so.1",
    "libGLESv2.so.2",
    "libGLX_*",
    "libGL_*",
    "libdrm.so.2",
    "libdrm_*",
    "libgbm.so.1",
    "libva*so*",
    "libvulkan.so.1",
    "libvulkan_*",
    "vdpau/libvdpau_*",
}

_CORE24_PATTERNS = _CORE22_PATTERNS | {
    "gbm/*_gbm.so",
    "libGLESv1_CM.so.1",
    "libvdpau*so*",
}

_BASE_PATTERNS = {
    "core22": _CORE22_PATTERNS,
    "core24": _CORE24_PATTERNS,
}


class GpuLinter(Linter):
    """Linter for GPU libraries that should be provided by a content interface."""

    @staticmethod
    def get_categories() -> list[str]:
        return ["gpu"]

    @overrides
    def run(self) -> list[LinterIssue]:
        if self._snap_metadata.type not in ("app", None):
            return []

        # GPU content interfaces only work with strict confinement
        if self._snap_metadata.confinement == "classic":
            return []

        current_path = Path()
        issues: list[LinterIssue] = []
        base = self._snap_metadata.base or "core24"

        elf_files = elf_utils.get_elf_files(current_path)

        for elf_file in elf_files:
            if self._is_file_ignored(elf_file):
                continue

            # Check if the file path matches any GPU library patterns
            file_path_str = str(elf_file.path)
            for pattern in _BASE_PATTERNS[base]:
                if fnmatch.fnmatch(file_path_str, f"*/{pattern}"):
                    issues.append(
                        LinterIssue(
                            name=self._name,
                            result=LinterResult.WARNING,
                            filename=str(elf_file.path),
                            text="GPU support library should be provided by a content interface.",
                            url=_HELP_URLS[base],
                        )
                    )
                    break  # Only report once per file

        return issues
