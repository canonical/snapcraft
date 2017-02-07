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

import re

from . import process_grammar
from .errors import (
    StagePackageSyntaxError,
    UnsatisfiedStatementError,
)


class OnStatement:
    """Process an 'on' statement in the stage packages grammar.

    For example:
    >>> import tempfile
    >>> from snapcraft import repo, ProjectOptions
    >>> with tempfile.TemporaryDirectory() as cache_dir:
    ...     ubuntu = repo.Ubuntu(cache_dir)
    ...     options = ProjectOptions(target_deb_arch='i386')
    ...     clause = OnStatement('on amd64', ['foo'], options, ubuntu)
    ...     clause.add_else(['bar'])
    ...     clause.process()
    {'bar'}
    """

    def __init__(self, on, body, project_options, repo_instance):
        """Create an _OnStatement instance.

        :param str on: The 'on <selectors>' part of the clause.
        :param list body: The body of the 'on' clause.
        :param project_options: Instance of ProjectOptions to use to process
                                clause.
        :type project_options: snapcraft.ProjectOptions
        :param repo_instance: repo.Ubuntu instance used for checking package
                              validity.
        :type repo_instance: repo.Ubuntu
        """

        self.selectors = _extract_on_clause_selectors(on)
        self._body = body
        self._project_options = project_options
        self._repo_instance = repo_instance
        self._else_bodies = []

    def add_else(self, else_body):
        """Add an 'else' clause to the statement.

        :param list else_body: The body of an 'else' clause.

        The 'else' clauses will be processed in the order they are added.
        """

        self._else_bodies.append(else_body)

    def process(self):
        """Process the clause.

        :return: Stage packages as determined by evaluating the statement.
        :rtype: list
        """

        packages = set()
        target_arch = self._project_options.deb_arch

        # The only selector currently supported is the target arch. Since
        # selectors are matched with an AND, not OR, there should only be one
        # selector.
        if (len(self.selectors) == 1) and (target_arch in self.selectors):
            packages = process_grammar(
                self._body, self._project_options, self._repo_instance)
        else:
            for else_body in self._else_bodies:
                if not else_body:
                    # Handle the 'else fail' case.
                    raise UnsatisfiedStatementError(self)

                packages = process_grammar(
                    else_body, self._project_options, self._repo_instance)
                if packages:
                    break

        return packages

    def __eq__(self, other):
        return self.selectors == other.selectors

    def __repr__(self):
        return "'on {}'".format(','.join(sorted(self.selectors)))


def _extract_on_clause_selectors(on):
    """Extract the list of selectors within an on clause.

    :param str on: The 'on <selector' part of the 'on' clause.

    :return: Selectors found within the 'on' clause.
    :rtype: set

    For example:
    >>> _extract_on_clause_selectors('on amd64,i386') == {'amd64', 'i386'}
    True
    """

    selector_pattern = re.compile(r'\Aon\s+([^,\s](?:,?[^,\s]+)*)\Z')
    match = selector_pattern.match(on)

    try:
        selector_group = match.group(1)
    except (AttributeError, IndexError):
        raise StagePackageSyntaxError(
            "{!r} is not a valid 'on' clause".format(on))

    return {selector.strip() for selector in selector_group.split(',')}
