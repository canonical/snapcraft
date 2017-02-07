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

from .errors import StagePackageSyntaxError


def process_grammar(grammar, project_options, ubuntu):
    """Process stage packages grammar and extract packages to actually stage.

    Arguments:
        grammar: List containing stage-packages grammar.
        project_options: Instance of ProjectOptions to use to determine stage
                         packages.
        ubuntu: repo.Ubuntu instance used for checking package validity.

    Returns:
        Set of packages to stage.
    """

    from ._on import OnStatement
    from ._try import TryStatement

    packages = set()
    on_clause_pattern = re.compile(r'\Aon\s+')
    try_clause_pattern = re.compile(r'\Atry\Z')
    else_clause_pattern = re.compile(r'\Aelse\Z')
    else_fail_pattern = re.compile(r'\Aelse\s+fail\Z')
    statements = _StatementCollection()
    statement = None

    for section in grammar:
        try:
            if else_fail_pattern.match(section):
                _handle_else(statement, None)
            else:
                # If it's just a string, add it to the package set.
                packages.add(section)
        except TypeError:
            # Alright, it wasn't an unconditional stage package, so it must be
            # a dict.
            for key, value in section.items():
                if on_clause_pattern.match(key):
                    # We've come across the begining of an 'on' statement.
                    # That means any previous statement we found is complete.
                    statements.add(statement)

                    statement = OnStatement(
                        key, value, project_options, ubuntu)

                if try_clause_pattern.match(key):
                    # We've come across the begining of a 'try' statement.
                    # That means any previous statement we found is complete.
                    statements.add(statement)

                    statement = TryStatement(
                        value, project_options, ubuntu)

                if else_clause_pattern.match(key):
                    _handle_else(statement, value)

    # We've parsed the entire grammar, time to process it.
    statements.add(statement)
    packages |= statements.process_all()

    return packages


def _handle_else(statement, else_body):
    """Add else body to current statement.

    Arguments:
        statement: The currently-active statement (if any).
        else_body: The body of the else clause to add.

    Raises:
        StagePackageSyntaxError if there isn't a currently-active statement.
    """

    try:
        statement.add_else(else_body)
    except AttributeError:
        raise StagePackageSyntaxError(
            "'else' doesn't seem to correspond to an 'on' or "
            "'try'")


class _StatementCollection:
    """Unique collection of statements to run at a later time."""

    def __init__(self):
        self._statements = []

    def add(self, statement):
        """Add new statement to collection.

        Arguments:
            statement: New statement.

        Raises:
            StagePackageSyntaxError if statement is already in collection.
        """

        if not statement:
            return

        if statement in self._statements:
            raise StagePackageSyntaxError(
                "found duplicate {!r} statements. These should be "
                'merged.'.format(statement))

        self._statements.append(statement)

    def process_all(self):
        """Process all statements in collection.

        Returns:
            Set of packages to stage as judged by all statements in collection.
        """
        packages = set()
        for statement in self._statements:
            packages |= statement.process()

        return packages
