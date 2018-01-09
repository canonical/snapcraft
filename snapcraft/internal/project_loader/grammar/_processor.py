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

from .errors import GrammarSyntaxError

_ON_CLAUSE_PATTERN = re.compile(r'\Aon\s+')
_TO_CLAUSE_PATTERN = re.compile(r'\Ato\s+')
_TRY_CLAUSE_PATTERN = re.compile(r'\Atry\Z')
_ELSE_CLAUSE_PATTERN = re.compile(r'\Aelse\Z')
_ELSE_FAIL_PATTERN = re.compile(r'\Aelse\s+fail\Z')


def process_grammar(grammar, project_options, checker):
    """Process grammar and extract desired primitives.

    :param list grammar: Unprocessed grammar.
    :param project_options: Instance of ProjectOptions to use to determine
                            appropriate primitives.
    :type project_options: snapcraft.ProjectOptions
    :param checker: callable accepting a single primitive, returning
                    true if it is valid
    :type checker: callable

    :return: Primitives selected
    :rtype: set
    """

    primitives = set()
    statements = _StatementCollection()
    statement = None

    for section in grammar:
        if isinstance(section, str):
            # If the secion is just a string, it's either "else fail" or a
            # primitive name.
            if _ELSE_FAIL_PATTERN.match(section):
                _handle_else(statement, None)
            else:
                primitives.add(section)
        elif isinstance(section, dict):
            statement = _parse_dict(
                section, statement, statements, project_options, checker)
        else:
            # jsonschema should never let us get here.
            raise GrammarSyntaxError(
                "expected grammar section to be either of type 'str' or "
                "type 'dict', but got {!r}".format(type(section)))

    # We've parsed the entire grammar, time to process it.
    statements.add(statement)
    primitives |= statements.process_all()

    return primitives


def _parse_dict(section, statement, statements, project_options,
                checker):
    from ._on import OnStatement
    from ._to import ToStatement
    from ._try import TryStatement

    for key, value in section.items():
        # Grammar is always written as a list of selectors but the value can
        # be a list or a string, in the latter case we wrap it so no special
        # care needs to be taken when fetching the result from the primitive
        if not isinstance(value, list):
            value = {value}

        if _ON_CLAUSE_PATTERN.match(key):
            # We've come across the beginning of an 'on' statement.
            # That means any previous statement we found is complete.
            # The first time through this may be None, but the
            # collection will ignore it.
            statements.add(statement)

            statement = OnStatement(
                on=key, body=value, project_options=project_options,
                checker=checker)

        if _TO_CLAUSE_PATTERN.match(key):
            # We've come across the beginning of a 'to' statement.
            # That means any previous statement we found is complete.
            # The first time through this may be None, but the
            # collection will ignore it.
            statements.add(statement)

            statement = ToStatement(
                to=key, body=value, project_options=project_options,
                checker=checker)

        if _TRY_CLAUSE_PATTERN.match(key):
            # We've come across the beginning of a 'try' statement.
            # That means any previous statement we found is complete.
            # The first time through this may be None, but the
            # collection will ignore it.
            statements.add(statement)

            statement = TryStatement(
                body=value, project_options=project_options,
                checker=checker)

        if _ELSE_CLAUSE_PATTERN.match(key):
            _handle_else(statement, value)

    return statement


def _handle_else(statement, else_body):
    """Add else body to current statement.

    :param statement: The currently-active statement. If None it will be
                      ignored.
    :param else_body: The body of the else clause to add.

    :raises GrammarSyntaxError: If there isn't a currently-active
                                     statement.
    """

    try:
        statement.add_else(else_body)
    except AttributeError:
        raise GrammarSyntaxError(
            "'else' doesn't seem to correspond to an 'on' or "
            "'try'")


class _StatementCollection:
    """Unique collection of statements to run at a later time."""

    def __init__(self):
        self._statements = []

    def add(self, statement):
        """Add new statement to collection.

        :param statement: New statement.

        :raises GrammarSyntaxError: If statement is already in collection.
        """

        if not statement:
            return

        if statement in self._statements:
            raise GrammarSyntaxError(
                "found duplicate {!r} statements. These should be "
                'merged.'.format(statement))

        self._statements.append(statement)

    def process_all(self):
        """Process all statements in collection.

        :return: Selected primitives as judged by all statements in collection.
        :rtype: set
        """
        primitives = set()
        for statement in self._statements:
            primitives |= statement.process()

        return primitives
