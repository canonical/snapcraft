# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Constants used in snapcraft."""

import enum


class SnapArch(str, enum.Enum):
    """An architecture for a snap."""

    amd64 = "amd64"
    arm64 = "arm64"
    armhf = "armhf"
    ppc64el = "ppc64el"
    riscv64 = "riscv64"
    s390x = "s390x"

    def __str__(self) -> str:
        """Stringify the value."""
        return str(self.value)


SUPPORTED_ARCHS = frozenset(arch.value for arch in SnapArch)

# todo: 'devel' is not a valid base (#4879)
BASES = frozenset({"core", "core18", "core20", "core22", "core24", "devel"})
"""All bases recognized by snapcraft."""

ESM_BASES = frozenset({"core", "core18"})
"""Bases no longer supported by the current version of snapcraft."""

LEGACY_BASES = frozenset({"core20"})
"""Bases handled by the legacy snapcraft codebase."""

CURRENT_BASES = frozenset(BASES - ESM_BASES - LEGACY_BASES)
"""Bases handled by the current snapcraft codebase."""

BUILD_BASES = CURRENT_BASES
"""Build bases supported by snapcraft."""

KERNEL_BUILD_BASES = frozenset(
    {f"ubuntu@{series}" for series in ["22.04", "24.04", "24.10"]}
)
"""Build bases supported for kernel snaps."""
