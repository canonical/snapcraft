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

from snapcraft import project
from snapcraft.internal import repo
from snapcraft.internal.project_loader import grammar

from ._package_transformer import package_transformer


class GlobalGrammarProcessor:
    """Process global properties that support grammar.

    Build packages example:
    >>> import snapcraft
    >>> from snapcraft import repo
    >>> processor = GlobalGrammarProcessor(
    ...    properties={'build-packages': [{'try': ['hello']}]},
    ...    project=snapcraft.project.Project())
    >>> processor.get_build_packages()
    {'hello'}
    """

    def __init__(self, *, properties: Dict[str, Any], project: project.Project) -> None:
        self._project = project

        self._build_package_grammar = properties.get("build-packages", [])
        self.__build_packages = set()  # type: Set[str]

    def get_build_packages(self) -> Set[str]:
        if not self.__build_packages:
            processor = grammar.GrammarProcessor(
                self._build_package_grammar,
                self._project,
                repo.Repo.build_package_is_valid,
                transformer=package_transformer,
            )
            self.__build_packages = processor.process()

        return self.__build_packages
