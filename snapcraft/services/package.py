# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Snapcraft Package service."""

from __future__ import annotations

import pathlib

from craft_application import PackageService, models
from overrides import override


class Package(PackageService):
    """Package service subclass for Snapcraft."""

    @override
    def pack(self, prime_dir: pathlib.Path, dest: pathlib.Path) -> list[pathlib.Path]:
        """Create one or more packages as appropriate.

        :param prime_dir: Path to the directory to pack.
        :param dest: Directory into which to write the package(s).

        :returns: A list of paths to created packages.
        """
        # TODO
        raise NotImplementedError(
            "Packing using the package service not yet implemented."
        )

    @override
    def write_metadata(self, path: pathlib.Path) -> None:
        """Write the project metadata to metadata.yaml in the given directory.

        :param path: The path to the prime directory.
        """
        # TODO
        raise NotImplementedError("Writing metadata not yet implemented.")

    @property
    def metadata(self) -> models.BaseMetadata:
        """Get the metadata model for this project."""
        # TODO: get metadata from project
        return models.BaseMetadata()
