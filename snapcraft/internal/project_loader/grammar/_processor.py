# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2018 Canonical Ltd
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
from typing import Any, Callable, Dict, List, Set

from snapcraft import project
from .errors import GrammarSyntaxError
from . import typing
from ._statement import Statement
from ._on import OnStatement
from ._to import ToStatement
from ._try import TryStatement
from ._compound import CompoundStatement

_ON_TO_CLAUSE_PATTERN = re.compile(r"(\Aon\s+\S+)\s+(to\s+\S+\Z)")
_ON_CLAUSE_PATTERN = re.compile(r"\Aon\s+")
_TO_CLAUSE_PATTERN = re.compile(r"\Ato\s+")
_TRY_CLAUSE_PATTERN = re.compile(r"\Atry\Z")
_ELSE_CLAUSE_PATTERN = re.compile(r"\Aelse\Z")
_ELSE_FAIL_PATTERN = re.compile(r"\Aelse\s+fail\Z")


class GrammarProcessor:
    """The GrammarProcessor extracts desired primitives from grammar."""

    def __init__(
        self,
        grammar: typing.Grammar,
        project: project.Project,
        checker: Callable[[str], bool],
        *,
        transformer: Callable[[List[Statement], str, project.Project], str] = None
    ) -> None:
        """Create a new GrammarProcessor.

        :param list grammar: Unprocessed grammar.
        :param project: Instance of Project to use to determine appropriate
                        primitives.
        :type project: snapcraft.project.Project
        :param callable checker: callable accepting a single primitive,
                                 returning true if it is valid.
        :param callable transformer: callable accepting a call stack, single
                                     primitive, and project, and returning a
                                     transformed primitive.
        """
        self._grammar = grammar
        self.project = project
        self.checker = checker

        if transformer:
            self._transformer = transformer
        else:
            # By default, no transformation
            self._transformer = lambda s, p, o: p

    def process(
        self, *, grammar: typing.Grammar = None, call_stack: typing.CallStack = None
    ):
        """Process grammar and extract desired primitives.

        :param list grammar: Unprocessed grammar (defaults to that set in
                             init).
        :param list call_stack: Call stack of statements leading to now.

        :return: Primitives selected
        :rtype: set
        """

        if grammar is None:
            grammar = self._grammar

        if call_stack is None:
            call_stack = []

        primitives = set()  # type: Set[str]
        statements = _StatementCollection()
        statement = None  # type: Statement

        for section in grammar:
            if isinstance(section, str):
                # If the section is just a string, it's either "else fail" or a
                # primitive name.
                if _ELSE_FAIL_PATTERN.match(section):
                    _handle_else(statement, None)
                else:
                    primitives.add(self._transformer(call_stack, section, self.project))
            elif isinstance(section, dict):
                statement = self._parse_dict(section, statement, statements, call_stack)
            else:
                # jsonschema should never let us get here.
                raise GrammarSyntaxError(
                    "expected grammar section to be either of type 'str' or "
                    "type 'dict', but got {!r}".format(type(section))
                )

        # We've parsed the entire grammar, time to process it.
        statements.add(statement)
        primitives |= statements.process_all()

        return primitives

    def _parse_dict(
        self,
        section: Dict[str, Any],
        statement: Statement,
        statements: "_StatementCollection",
        call_stack: typing.CallStack,
    ):
        for key, value in section.items():
            # Grammar is always written as a list of selectors but the value
            # can be a list or a string. In the latter case we wrap it so no
            # special care needs to be taken when fetching the result from the
            # primitive.
            if not isinstance(value, list):
                value = {value}

            if _ON_TO_CLAUSE_PATTERN.match(key):
                # We've come across the beginning of a compound statement
                # with both 'on' and 'to'.

                # First, extract each statement's part of the string
                on, to = _ON_TO_CLAUSE_PATTERN.match(key).groups()

                # Now create a list of statements, in order
                compound_statements = [
                    OnStatement(
                        on=on, body=None, processor=self, call_stack=call_stack
                    ),
                    ToStatement(
                        to=to, body=None, processor=self, call_stack=call_stack
                    ),
                ]

                # Now our statement is a compound statement
                statement = CompoundStatement(
                    statements=compound_statements,
                    body=value,
                    processor=self,
                    call_stack=call_stack,
                )

            elif _ON_CLAUSE_PATTERN.match(key):
                # We've come across the beginning of an 'on' statement.
                # That means any previous statement we found is complete.
                # The first time through this may be None, but the
                # collection will ignore it.
                statements.add(statement)

                statement = OnStatement(
                    on=key, body=value, processor=self, call_stack=call_stack
                )

            elif _TO_CLAUSE_PATTERN.match(key):
                # We've come across the beginning of a 'to' statement.
                # That means any previous statement we found is complete.
                # The first time through this may be None, but the
                # collection will ignore it.
                statements.add(statement)

                statement = ToStatement(
                    to=key, body=value, processor=self, call_stack=call_stack
                )

            elif _TRY_CLAUSE_PATTERN.match(key):
                # We've come across the beginning of a 'try' statement.
                # That means any previous statement we found is complete.
                # The first time through this may be None, but the
                # collection will ignore it.
                statements.add(statement)

                statement = TryStatement(
                    body=value, processor=self, call_stack=call_stack
                )

            elif _ELSE_CLAUSE_PATTERN.match(key):
                _handle_else(statement, value)

        return statement


def _handle_else(statement: Statement, else_body: typing.Grammar):
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
            "'else' doesn't seem to correspond to an 'on' or 'try'"
        )


class _StatementCollection:
    """Unique collection of statements to run at a later time."""

    def __init__(self) -> None:
        self._statements = []  # type: List[Statement]

    def add(self, statement: Statement) -> None:
        """Add new statement to collection.

        :param statement: New statement.

        :raises GrammarSyntaxError: If statement is already in collection.
        """

        if not statement:
            return

        if statement in self._statements:
            raise GrammarSyntaxError(
                "found duplicate {!r} statements. These should be "
                "merged.".format(statement)
            )

        self._statements.append(statement)

    def process_all(self) -> Set[str]:
        """Process all statements in collection.

        :return: Selected primitives as judged by all statements in collection.
        :rtype: set
        """
        primitives = set()  # type: Set[str]
        for statement in self._statements:
            primitives |= statement.process()

        return primitives
