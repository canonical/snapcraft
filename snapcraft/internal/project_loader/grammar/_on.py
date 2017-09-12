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
    OnStatementSyntaxError,
    UnsatisfiedStatementError,
)

_SELECTOR_PATTERN = re.compile(r'\Aon\s+([^,\s](?:,?[^,]+)*)\Z')
_WHITESPACE_PATTERN = re.compile(r'\A.*\s.*\Z')


class OnStatement:
    """Process an 'on' statement in the grammar.

    For example:
    >>> import tempfile
    >>> from snapcraft import ProjectOptions
    >>> def checker(primitive):
    ...     return True
    >>> with tempfile.TemporaryDirectory() as cache_dir:
    ...     options = ProjectOptions(target_deb_arch='i386')
    ...     clause = OnStatement(on='on amd64', body=['foo'],
    ...                          project_options=options,
    ...                          checker=checker)
    ...     clause.add_else(['bar'])
    ...     clause.process()
    {'bar'}
    """

    def __init__(self, *, on, body, project_options, checker):
        """Create an _OnStatement instance.

        :param str on: The 'on <selectors>' part of the clause.
        :param list body: The body of the 'on' clause.
        :param project_options: Instance of ProjectOptions to use to process
                                clause.
        :type project_options: snapcraft.ProjectOptions
        :param checker: callable accepting a single primitive, returning
                        true if it is valid
        :type checker: callable
        """

        self.selectors = _extract_on_clause_selectors(on)
        self._body = body
        self._project_options = project_options
        self._checker = checker
        self._else_bodies = []

    def add_else(self, else_body):
        """Add an 'else' clause to the statement.

        :param list else_body: The body of an 'else' clause.

        The 'else' clauses will be processed in the order they are added.
        """

        self._else_bodies.append(else_body)

    def process(self):
        """Process the clause.

        :return: Primitives as determined by evaluating the statement.
        :rtype: list
        """

        primitives = set()
        target_arch = self._project_options.deb_arch

        # The only selector currently supported is the target arch. Since
        # selectors are matched with an AND, not OR, there should only be one
        # selector.
        if (len(self.selectors) == 1) and (target_arch in self.selectors):
            primitives = process_grammar(
                self._body, self._project_options, self._checker)
        else:
            for else_body in self._else_bodies:
                if not else_body:
                    # Handle the 'else fail' case.
                    raise UnsatisfiedStatementError(self)

                primitives = process_grammar(
                    else_body, self._project_options, self._checker)
                if primitives:
                    break

        return primitives

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

    match = _SELECTOR_PATTERN.match(on)

    try:
        selector_group = match.group(1)
    except AttributeError:
        raise OnStatementSyntaxError(on, message='selectors are missing')
    except IndexError:
        raise OnStatementSyntaxError(on)

    # This could be part of the _SELECTOR_PATTERN, but that would require us
    # to provide a very generic error when we can try to be more helpful.
    if _WHITESPACE_PATTERN.match(selector_group):
        raise OnStatementSyntaxError(
            on, message='spaces are not allowed in the selectors')

    return {selector.strip() for selector in selector_group.split(',')}
