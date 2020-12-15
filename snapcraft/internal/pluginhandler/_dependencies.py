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
from typing import Sequence, Set

from snapcraft.internal import repo

_MSG_EXTEND_STAGE_PACKAGES = (
    "The {part_name!r} part is missing libraries that are not "
    "included in the snap or base. They can be satisfied by adding the following "
    "entries to the existing stage-packages for this "
    "part:\n{stage_packages}"
)
_MSG_ADD_STAGE_PACKAGES = (
    "The {part_name!r} part is missing libraries that are not "
    "included in the snap or base. They can be satisfied by adding the following "
    "entry for this part\nstage-packages:\n{stage_packages}"
)
_MSG_UNHANDLED_DEPENDENCIES = (
    "This part is missing libraries that cannot be satisfied with "
    "any available stage-packages known to snapcraft:\n{unhandled_list}\n"
    "These dependencies can be satisfied via additional parts or content sharing. "
    "Consider validating configured filesets if this dependency was built."
)


def _get_formatted_list(items: Set[str]) -> str:
    return "".join(["- {}\n".format(s) for s in sorted(items)]).strip()


class MissingDependencyResolver:
    """A missing DT_NEEDED resolver."""

    def __init__(self, elf_files: Sequence[str]) -> None:
        self._stage_packages_dependencies = set()  # type: Set[str]
        self._unhandled_dependencies = set()  # type: Set[str]
        self._process(elf_files)

    def _process(self, elf_files: Sequence[str]) -> None:
        for elf_file in elf_files:
            try:
                stage_package = repo.Repo.get_package_for_file(
                    file_path=os.path.join(os.path.sep, elf_file)
                )
                self._stage_packages_dependencies.add(stage_package)
            except repo.errors.FileProviderNotFound:
                self._unhandled_dependencies.add(elf_file)

    def print_resolutions(
        self, *, part_name: str, stage_packages_exist: bool, echoer
    ) -> None:
        if not self._stage_packages_dependencies and not self._unhandled_dependencies:
            return

        if self._stage_packages_dependencies:
            stage_packages_list = _get_formatted_list(self._stage_packages_dependencies)
            if stage_packages_exist:
                echoer.warning(
                    _MSG_EXTEND_STAGE_PACKAGES.format(
                        part_name=part_name, stage_packages=stage_packages_list
                    )
                )
            else:
                echoer.warning(
                    _MSG_ADD_STAGE_PACKAGES.format(
                        part_name=part_name, stage_packages=stage_packages_list
                    )
                )

        if self._unhandled_dependencies:
            unhandled_list = _get_formatted_list(self._unhandled_dependencies)
            echoer.warning(
                _MSG_UNHANDLED_DEPENDENCIES.format(
                    part_name=part_name, unhandled_list=unhandled_list
                )
            )
