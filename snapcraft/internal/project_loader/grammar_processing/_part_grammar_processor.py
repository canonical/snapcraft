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
from snapcraft.internal.project_loader import grammar
from snapcraft.internal import pluginhandler, repo

from ._package_transformer import package_transformer


class PartGrammarProcessor:
    """Process part properties that support grammar.

    Stage packages example:
    >>> from unittest import mock
    >>> import snapcraft
    >>> # Pretend that all packages are valid
    >>> repo = mock.Mock()
    >>> repo.is_valid.return_value = True
    >>> plugin = mock.Mock()
    >>> plugin.stage_packages = [{'try': ['foo']}]
    >>> processor = PartGrammarProcessor(
    ...    plugin=plugin,
    ...    properties={},
    ...    project=snapcraft.project.Project(),
    ...    repo=repo)
    >>> processor.get_stage_packages()
    {'foo'}

    Build packages example:
    >>> from unittest import mock
    >>> import snapcraft
    >>> # Pretend that all packages are valid
    >>> repo = mock.Mock()
    >>> repo.is_valid.return_value = True
    >>> plugin = mock.Mock()
    >>> plugin.build_packages = [{'try': ['foo']}]
    >>> processor = PartGrammarProcessor(
    ...    plugin=plugin,
    ...    properties={},
    ...    project=snapcraft.project.Project(),
    ...    repo=repo)
    >>> processor.get_build_packages()
    {'foo'}

    Source example:
    >>> from unittest import mock
    >>> import snapcraft
    >>> plugin = mock.Mock()
    >>> plugin.properties = {'source': [{'on amd64': 'foo'}, 'else fail']}
    >>> processor = PartGrammarProcessor(
    ...    plugin=plugin,
    ...    properties=plugin.properties,
    ...    project=snapcraft.project.Project(),
    ...    repo=None)
    >>> processor.get_source()
    'foo'
    """

    def __init__(
        self,
        *,
        plugin: pluginhandler.PluginHandler,
        properties: Dict[str, Any],
        project: project.Project,
        repo: "repo.Ubuntu"
    ) -> None:
        self._project = project
        self._repo = repo

        self._build_snap_grammar = getattr(plugin, "build_snaps", [])
        self.__build_snaps = set()  # type: Set[str]

        self._build_package_grammar = getattr(plugin, "build_packages", [])
        self.__build_packages = set()  # type: Set[str]

        self._stage_package_grammar = getattr(plugin, "stage_packages", [])
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
            processor = grammar.GrammarProcessor(
                self._source_grammar, self._project, lambda s: True
            )
            source_array = processor.process()
            if len(source_array) > 0:
                self.__source = source_array.pop()
        return self.__source

    def get_build_snaps(self) -> Set[str]:
        if not self.__build_snaps:
            processor = grammar.GrammarProcessor(
                self._build_snap_grammar,
                self._project,
                repo.snaps.SnapPackage.is_valid_snap,
            )
            self.__build_snaps = processor.process()

        return self.__build_snaps

    def get_build_packages(self) -> Set[str]:
        if not self.__build_packages:
            processor = grammar.GrammarProcessor(
                self._build_package_grammar,
                self._project,
                self._repo.build_package_is_valid,
                transformer=package_transformer,
            )
            self.__build_packages = processor.process()

        return self.__build_packages

    def get_stage_packages(self) -> Set[str]:
        if not self.__stage_packages:
            processor = grammar.GrammarProcessor(
                self._stage_package_grammar,
                self._project,
                self._repo.is_valid,
                transformer=package_transformer,
            )
            self.__stage_packages = processor.process()

        return self.__stage_packages
