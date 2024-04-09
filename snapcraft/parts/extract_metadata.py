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
"""Utilities to extract project metadata from lifecycle directories."""
from __future__ import annotations

import pathlib
from typing import Sequence

from craft_cli import emit
from craft_parts import Part, ProjectDirs

from snapcraft import meta


def extract_lifecycle_metadata(
    adopt_info: str | None,
    parse_info: dict[str, list[str]],
    work_dir: pathlib.Path,
    partitions: Sequence[str] | None,
) -> list[meta.ExtractedMetadata]:
    """Obtain metadata information.

    :param adopt_info: the Project's ``adopt-info``
    :param parse_info: the ``parse-info`` information from the Project, organised
      as a dict of "part-name" to "list of files providing metadata".
    :param work_dir: the lifecycle's working directory.
    """
    if adopt_info is None or adopt_info not in parse_info:
        return []

    dirs = ProjectDirs(work_dir=work_dir, partitions=partitions)
    part = Part(adopt_info, {}, project_dirs=dirs, partitions=partitions)
    locations = (
        part.part_src_dir,
        part.part_build_dir,
        part.part_install_dir,
    )
    metadata_list: list[meta.ExtractedMetadata] = []

    for metadata_file in parse_info[adopt_info]:
        emit.trace(f"extract metadata: parse info from {metadata_file}")

        for location in locations:
            if pathlib.Path(location, metadata_file.lstrip("/")).is_file():
                metadata = meta.extract_metadata(metadata_file, workdir=str(location))
                if metadata:
                    metadata_list.append(metadata)
                    break

                emit.progress(
                    f"No metadata extracted from {metadata_file}", permanent=True
                )

    return metadata_list
