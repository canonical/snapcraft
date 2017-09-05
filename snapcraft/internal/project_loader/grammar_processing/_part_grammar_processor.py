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

from snapcraft.internal.project_loader import grammar
from snapcraft.internal import repo


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
    ...    project_options=snapcraft.ProjectOptions(),
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
    ...    project_options=snapcraft.ProjectOptions(),
    ...    repo=repo)
    >>> processor.get_build_packages()
    {'foo'}
    """

    def __init__(self, *, plugin, properties, project_options, repo):
        self._project_options = project_options
        self._repo = repo

        self._build_snap_grammar = getattr(plugin, 'build_snaps', [])
        self.__build_snaps = set()

        self._build_package_grammar = getattr(plugin, 'build_packages', [])
        self.__build_packages = set()

        self._stage_package_grammar = getattr(plugin, 'stage_packages', [])
        self.__stage_packages = set()

    def get_build_snaps(self):
        if not self.__build_snaps:
            self.__build_snaps = grammar.process_grammar(
                self._build_snap_grammar, self._project_options,
                repo.snaps.SnapPackage.is_valid_snap)

        return self.__build_snaps

    def get_build_packages(self):
        if not self.__build_packages:
            self.__build_packages = grammar.process_grammar(
                self._build_package_grammar, self._project_options,
                self._repo.build_package_is_valid)

        return self.__build_packages

    def get_stage_packages(self):
        if not self.__stage_packages:
            self.__stage_packages = grammar.process_grammar(
                self._stage_package_grammar, self._project_options,
                self._repo.is_valid)

        return self.__stage_packages
