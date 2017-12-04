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

from typing import Set

import snapcraft
from snapcraft.internal import repo
from snapcraft.internal.project_loader import grammar


class GlobalGrammarProcessor:
    """Process global properties that support grammar.

    Build packages example:
    >>> import snapcraft
    >>> from snapcraft import repo
    >>> processor = GlobalGrammarProcessor(
    ...    properties={'build-packages': [{'try': ['hello']}]},
    ...    project_options=snapcraft.ProjectOptions())
    >>> processor.get_build_packages()
    {'hello'}

    Architectures example:
    >>> import snapcraft
    >>> from snapcraft import repo
    >>> processor = GlobalGrammarProcessor(
    ...    properties={'architectures': [{'try': ['armhf']}]},
    ...    project_options=snapcraft.ProjectOptions())
    >>> processor.get_architectures()
    {'armhf'}
    """

    def __init__(self, *, properties, project_options) -> None:
        self._project_options = project_options

        self._build_package_grammar = properties.get('build-packages', [])
        self.__build_packages = set()  # type: Set[str]

        self._architectures_grammar = properties.get('architectures', [])
        self.__architectures = set()  # type: Set[str]

    def get_build_packages(self) -> Set[str]:
        if not self.__build_packages:
            self.__build_packages = grammar.process_grammar(
                self._build_package_grammar, self._project_options,
                repo.Repo.build_package_is_valid)

        return self.__build_packages

    def get_architectures(self) -> Set[str]:
        if not self.__architectures:
            self.__architectures = grammar.process_grammar(
                self._architectures_grammar, self._project_options,
                snapcraft._options.deb_arch_is_supported)

        return self.__architectures
