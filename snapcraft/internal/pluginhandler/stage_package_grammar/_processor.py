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


def process_grammar(grammar, project_options, repo_instance):
    """Process stage packages grammar and extract packages to actually stage.

    :param list grammar: Unprocessed stage-packages grammar.
    :param project_options: Instance of ProjectOptions to use to determine
                            stage packages.
    :type project_options: snapcraft.ProjectOptions
    :param repo_instance: repo.Ubuntu instance used for checking package
                          validity.
    :type repo_instance: repo.Ubuntu

    :return: Packages to stage
    :rtype: set
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
        if isinstance(section, str):
            # If the secion is just a string, it's either "else fail" or a
            # package name.
            if else_fail_pattern.match(section):
                _handle_else(statement, None)
            else:
                packages.add(section)
        elif isinstance(section, dict):
            for key, value in section.items():
                if on_clause_pattern.match(key):
                    # We've come across the begining of an 'on' statement.
                    # That means any previous statement we found is complete.
                    # The first time through this may be None, but the
                    # collection will ignore it.
                    statements.add(statement)

                    statement = OnStatement(
                        on=key, body=value, project_options=project_options,
                        repo_instance=repo_instance)

                if try_clause_pattern.match(key):
                    # We've come across the begining of a 'try' statement.
                    # That means any previous statement we found is complete.
                    # The first time through this may be None, but the
                    # collection will ignore it.
                    statements.add(statement)

                    statement = TryStatement(
                        body=value, project_options=project_options,
                        repo_instance=repo_instance)

                if else_clause_pattern.match(key):
                    _handle_else(statement, value)
        else:
            # jsonschema should never let us get here.
            raise StagePackageSyntaxError(
                "expected grammar section to be either of type 'str' or "
                "type 'dict', but got {!r}".format(type(section)))

    # We've parsed the entire grammar, time to process it.
    statements.add(statement)
    packages |= statements.process_all()

    return packages


def _handle_else(statement, else_body):
    """Add else body to current statement.

    :param statement: The currently-active statement. If None it will be
                      ignored.
    :param else_body: The body of the else clause to add.

    :raises StagePackageSyntaxError: If there isn't a currently-active
                                     statement.
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

        :param statement: New statement.

        :raises StagePackageSyntaxError: If statement is already in collection.
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

        :return: Packages to stage as judged by all statements in collection.
        :rtype: set
        """
        packages = set()
        for statement in self._statements:
            packages |= statement.process()

        return packages
