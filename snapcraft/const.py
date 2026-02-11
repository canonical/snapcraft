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

DEPRECATED_COMMAND_WARNING = (
    "The '{old}' command was renamed to '{new}'. Use '{new}' instead. "
    "The old name will be removed in a future release."
)


class StrEnum(str, enum.Enum):
    def __repr__(self) -> str:
        return repr(self.value)

    def __str__(self) -> str:
        return str(self.value)


class ProjectType(StrEnum):
    """The type of snap project."""

    APP = "app"
    """An application snap (the default). Most snaps are this type."""
    BASE = "base"
    """A base snap, e.g. core26"""
    GADGET = "gadget"
    """A gadget snap."""
    KERNEL = "kernel"
    """A snap containing a kernel and initrd for Ubuntu Core."""
    SNAPD = "snapd"
    """A snap containing snapd itself."""


class StableBase(StrEnum):
    """Bases whose build-base must equal the base (or be absent)."""

    CORE22 = "core22"
    CORE24 = "core24"


class UnstableBase(StrEnum):
    """Bases that require "devel" as their build-base."""

    CORE26 = "core26"
    DEVEL = "devel"


class BuildBase(StrEnum):
    """Values that are valid for the 'build-base' key for most snaps.

    These correspond to the bases on which we can build for most snap types.
    """

    CORE22 = "core22"
    CORE24 = "core24"
    DEVEL = "devel"


class SnapArch(StrEnum):
    """An architecture for a snap."""

    amd64 = "amd64"
    arm64 = "arm64"
    armhf = "armhf"
    ppc64el = "ppc64el"
    riscv64 = "riscv64"
    s390x = "s390x"


SUPPORTED_ARCHS = frozenset(arch.value for arch in SnapArch)


class OutputFormat(str, enum.Enum):
    """Output formats for snapcraft commands."""

    json = "json"
    table = "table"

    def __str__(self) -> str:
        """Stringify the value."""
        return str(self.value)


OUTPUT_FORMATS = frozenset(output_format.value for output_format in OutputFormat)
"""Supported output formats for commands."""

CURRENT_BASES = frozenset(b.value for b in (*StableBase, *UnstableBase))
"""Bases handled by the current snapcraft codebase."""

ESM_BASES = frozenset({"core", "core18", "core20"})
"""Bases no longer supported by the current version of snapcraft."""

BASES = CURRENT_BASES | ESM_BASES
"""All bases recognized by snapcraft."""


SNAPCRAFT_ENVIRONMENT_VARIABLES = frozenset(
    {
        "SNAPCRAFT_BUILD_INFO",
        "SNAPCRAFT_IMAGE_INFO",
        "SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS",
    }
)
"""Snapcraft-specific environment variables."""
