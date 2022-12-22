# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

from typing import Any, Dict, Set

from craft_grammar import GrammarProcessor

from snapcraft_legacy.internal import repo


class GlobalGrammarProcessor:
    """Process global properties that support grammar.

    Build packages example:
    >>> import snapcraft_legacy
    >>> from snapcraft_legacy import repo
    >>> processor = GlobalGrammarProcessor(
    ...    properties={'build-packages': [{'try': ['hello']}]},
    ...    project=snapcraft_legacy.project.Project())
    >>> processor.get_build_packages()
    {'hello'}
    """

    def __init__(
        self, *, properties: Dict[str, Any], arch: str, target_arch: str
    ) -> None:
        self._arch = arch
        self._target_arch = target_arch

        self._build_package_grammar = properties.get("build-packages", [])
        self.__build_packages = set()  # type: Set[str]

    def get_build_packages(self) -> Set[str]:
        if not self.__build_packages:
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=repo.Repo.build_package_is_valid,
            )
            self.__build_packages = set(
                processor.process(grammar=self._build_package_grammar)
            )

        return self.__build_packages
