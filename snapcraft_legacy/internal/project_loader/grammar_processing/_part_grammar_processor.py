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

from typing import Any, Dict, List, Set

from craft_grammar import Grammar, GrammarProcessor

from snapcraft_legacy.internal import repo
from snapcraft_legacy.plugins.v2 import PluginV2

from ._package_transformer import package_transformer


class PartGrammarProcessor:
    """Process part properties that support grammar.

    Stage packages example:
    >>> from unittest import mock
    >>> import snapcraft_legacy
    >>> # Pretend that all packages are valid
    >>> repo = mock.Mock()
    >>> repo.is_valid.return_value = True
    >>> plugin = mock.Mock()
    >>> plugin.stage_packages = [{'try': ['foo']}]
    >>> processor = PartGrammarProcessor(
    ...    plugin=plugin,
    ...    properties={},
    ...    project=snapcraft_legacy.project.SnapcraftProject(),
    ...    repo=repo)
    >>> processor.get_stage_packages()
    {'foo'}

    Build packages example:
    >>> from unittest import mock
    >>> import snapcraft_legacy
    >>> # Pretend that all packages are valid
    >>> repo = mock.Mock()
    >>> repo.is_valid.return_value = True
    >>> plugin = mock.Mock()
    >>> plugin.build_packages = [{'try': ['foo']}]
    >>> processor = PartGrammarProcessor(
    ...    plugin=plugin,
    ...    properties={},
    ...    project=snapcraft_legacy.project.SnapcraftProject(),
    ...    repo=repo)
    >>> processor.get_build_packages()
    {'foo'}

    Source example:
    >>> from unittest import mock
    >>> import snapcraft_legacy
    >>> plugin = mock.Mock()
    >>> plugin.properties = {'source': [{'on amd64': 'foo'}, 'else fail']}
    >>> processor = PartGrammarProcessor(
    ...    plugin=plugin,
    ...    properties=plugin.properties,
    ...    project=snapcraft_legacy.project.SnapcraftProject(),
    ...    repo=None)
    >>> processor.get_source()
    'foo'
    """

    def __init__(
        self,
        *,
        plugin: PluginV2,
        properties: Dict[str, Any],
        arch: str,
        target_arch: str,
        repo: "repo.Ubuntu",
    ) -> None:
        self._plugin = plugin
        self._properties = properties
        self._arch = arch
        self._target_arch = target_arch
        self._repo = repo

        self.__build_environment: List[Dict[str, str]] = list()
        self.__build_snaps = set()  # type: Set[str]
        self.__stage_snaps = set()  # type: Set[str]
        self.__build_packages = set()  # type: Set[str]
        self.__stage_packages = set()  # type: Set[str]

        source_grammar = properties.get("source", [""])
        if not isinstance(source_grammar, list):
            self._source_grammar = [source_grammar]
        else:
            self._source_grammar = source_grammar

        self.__source = ""

    def get_source(self) -> str:
        if not self.__source:
            # The grammar is array-based, even though we only support a single
            # source.
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=lambda s: True,
            )
            source_array = processor.process(grammar=self._source_grammar)
            if len(source_array) > 0:
                self.__source = source_array.pop()
        return self.__source

    def _get_property(self, attr: str) -> Grammar:
        prop = self._properties.get(attr, set())
        return getattr(self._plugin, attr.replace("-", "_"), prop)

    def get_build_environment(self) -> List[Dict[str, str]]:
        if not self.__build_environment:
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=lambda s: True,
            )
            self.__build_environment = processor.process(
                grammar=self._get_property("build-environment"),
            )

        return self.__build_environment

    def get_build_snaps(self) -> Set[str]:
        if not self.__build_snaps:
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=repo.snaps.SnapPackage.is_valid_snap,
            )
            self.__build_snaps = set(
                processor.process(grammar=self._get_property("build-snaps"))
            )

        return self.__build_snaps

    def get_stage_snaps(self) -> Set[str]:
        if not self.__stage_snaps:
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=repo.snaps.SnapPackage.is_valid_snap,
            )
            self.__stage_snaps = set(
                processor.process(grammar=self._get_property("stage-snaps"))
            )

        return self.__stage_snaps

    def get_build_packages(self) -> Set[str]:
        if not self.__build_packages:
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=self._repo.build_package_is_valid,
            )
            self.__build_packages = set(
                processor.process(
                    grammar=self._get_property("build-packages"),
                )
            )

        return self.__build_packages

    def get_stage_packages(self) -> Set[str]:
        if not self.__stage_packages:
            processor = GrammarProcessor(
                arch=self._arch,
                target_arch=self._target_arch,
                checker=self._repo.build_package_is_valid,
                transformer=package_transformer,
            )
            self.__stage_packages = set(
                processor.process(
                    grammar=self._get_property("stage-packages"),
                )
            )

        return self.__stage_packages
