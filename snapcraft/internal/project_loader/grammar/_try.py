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

from . import process_grammar


class TryStatement:
    """Process a 'try' statement in the grammar.

    For example:
    >>> from snapcraft import ProjectOptions
    >>> def checker(primitive):
    ...     return 'invalid' not in primitive
    >>> options = ProjectOptions()
    >>> clause = TryStatement(body=['invalid'], project_options=options,
    ...                       checker=checker)
    >>> clause.add_else(['valid'])
    >>> clause.process()
    {'valid'}
    """

    def __init__(self, *, body, project_options, checker):
        """Create an _OnStatement instance.

        :param list body: The body of the 'try' clause.
        :param project_options: Instance of ProjectOptions to use to process
                                clause.
        :type project_options: snapcraft.ProjectOptions
        :param checker: callable accepting a single primitive, returning
                        true if it is valid
        :type checker: callable
        """

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

        primitives = process_grammar(
            self._body, self._project_options, self._checker)

        # If some of the primitives in the 'try' were invalid, then we need to
        # process the 'else' clauses.
        if not _all_primitives_valid(primitives, self._checker):
            if not self._else_bodies:
                # If there are no 'else' statements, the 'try' was considered
                # optional and it failed, which means it doesn't resolve to
                # any primitives.
                return set()

            for else_body in self._else_bodies:
                if not else_body:
                    continue

                primitives = process_grammar(
                    else_body, self._project_options, self._checker)

                # Stop once an 'else' clause gives us valid primitives
                if _all_primitives_valid(primitives, self._checker):
                    break

        return primitives

    def __repr__(self):
        return "'try'"


def _all_primitives_valid(primitives, checker):
    """Ensure that all primitives are valid.

    :param primitives: Iterable container of primitives.
    :param checker: callable accepting a single primitive, returning
                    true if it is valid
    :type checker: callable

    For example:
    >>> import tempfile
    >>> from snapcraft import ProjectOptions
    >>> def checker(primitive):
    ...     return 'invalid' not in primitive
    >>> with tempfile.TemporaryDirectory() as cache_dir:
    ...     _all_primitives_valid(['valid'], checker)
    ...     _all_primitives_valid(['valid', 'invalid'], checker)
    True
    False
    """

    for primitive in primitives:
        if not checker(primitive):
            return False
    return True
