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

import logging

from snapcraft.internal import repo
from snapcraft.internal.pluginhandler import stage_package_grammar

logger = logging.getLogger(__name__)


class StagePackageHandler:
    """Interpret stage-packages grammar, and fetch/unpack stage-packages.

    Basic example:
    >>> import os
    >>> import tempfile
    >>> with tempfile.TemporaryDirectory() as tmp:
    ...    cache_dir = os.path.join(tmp, 'cache')
    ...    unpack_dir = os.path.join(tmp, 'unpack')
    ...    handler = StagePackageHandler(['foo'], cache_dir)
    ...    pkg_list = handler.fetch()
    ...    handler.unpack(unpack_dir)
    """

    def __init__(self, stage_packages_grammar, cache_dir, *, sources=None,
                 project_options=None):
        """Create new StagePackageHandler.

        :param list stage_packages: Unprocessed stage-packages grammar.
        :param str cache_dir: Path to working directory.
        :param str sources: Alternative sources list (host's sources are
                            default).
        :param project_options: Instance of ProjectOptions to use for this
                                operation.
        :type project_options: snapcraft.ProjectOptions
        """

        self._grammar = stage_packages_grammar
        self._cache_dir = cache_dir
        self._sources = sources
        self._project_options = project_options
        self.__stage_packages = None
        self.__repo = None

    @property
    def _repo(self):
        if not self.__repo:
            self.__repo = repo.Repo(
                self._cache_dir, sources=self._sources,
                project_options=self._project_options)

        return self.__repo

    @property
    def _stage_packages(self):
        # Comparing to None here since after calculation it may be an empty set
        if self.__stage_packages is None:
            self.__stage_packages = stage_package_grammar.process_grammar(
                self._grammar, self._project_options, self._repo)

        return self.__stage_packages

    def fetch(self):
        """Fetch stage packages into cache.

        The stage packages will not be fetched if they're already present in
        the cache.
        """

        pkg_list = []
        if self._stage_packages:
            logger.debug('Fetching stage-packages {!r}'.format(
                self._stage_packages))
            pkg_list = self._repo.get(self._stage_packages)

        return pkg_list

    def unpack(self, unpack_dir):
        """Unpack fetched stage packages into directory.

        :param str unpack_dir: Path to directory in which stage packages will
                               be unpacked.
        """

        if self._stage_packages:
            logger.debug('Unpacking stage-packages to {!r}'.format(
                unpack_dir))
            self._repo.unpack(unpack_dir)
