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
from typing import Any, Callable, Dict, List, Optional, Tuple

from snapcraft import project

from . import typing
from ._compound import CompoundStatement
from ._on import OnStatement
from ._statement import Statement
from ._to import ToStatement
from ._try import TryStatement
from .errors import GrammarSyntaxError

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
        checker: Callable[[Any], bool],
        *,
        transformer: Callable[[List[Statement], str, project.Project], str] = None,
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
    ) -> List[Any]:
        """Process grammar and extract desired primitives.

        :param list grammar: Unprocessed grammar (defaults to that set in
                             init).
        :param list call_stack: Call stack of statements leading to now.

        :return: Primitives selected
        """

        if grammar is None:
            grammar = self._grammar

        if call_stack is None:
            call_stack = []

        primitives: List[Any] = list()
        statements = _StatementCollection()
        statement: Optional[Statement] = None

        for section in grammar:
            if isinstance(section, str):
                # If the section is just a string, it's either "else fail" or a
                # primitive name.
                if _ELSE_FAIL_PATTERN.match(section):
                    _handle_else(statement, None)
                else:
                    # Processing a string primitive indicates the previous section
                    # is finalized (if any), process it first before this primitive.
                    self._process_statement(
                        statement=statement,
                        statements=statements,
                        primitives=primitives,
                    )
                    statement = None

                    primitive = self._transformer(call_stack, section, self.project)
                    primitives.append(primitive)
            elif isinstance(section, dict):
                statement, finalized_statement = self._parse_section_dictionary(
                    call_stack=call_stack, section=section, statement=statement,
                )

                # Process any finalized statement (if any).
                if finalized_statement is not None:
                    self._process_statement(
                        statement=finalized_statement,
                        statements=statements,
                        primitives=primitives,
                    )

                # If this section does not belong to a statement, it is
                # a primitive to be recorded.
                if statement is None:
                    primitives.append(section)

            else:
                # jsonschema should never let us get here.
                raise GrammarSyntaxError(
                    "expected grammar section to be either of type 'str' or "
                    "type 'dict', but got {!r}".format(type(section))
                )

        # Process the final statement (if any).
        self._process_statement(
            statement=statement, statements=statements, primitives=primitives,
        )

        return primitives

    def _process_statement(
        self,
        *,
        statement: Optional[Statement],
        statements: "_StatementCollection",
        primitives: List[Any],
    ):
        if statement is None:
            return

        statements.add(statement)
        processed_primitives = statement.process()
        primitives.extend(processed_primitives)

    def _parse_section_dictionary(
        self,
        *,
        section: Dict[str, Any],
        statement: Optional[Statement],
        call_stack: typing.CallStack,
    ) -> Tuple[Optional[Statement], Optional[Statement]]:
        finalized_statement: Optional[Statement] = None
        for key, value in section.items():
            # Grammar is always written as a list of selectors but the value
            # can be a list or a string. In the latter case we wrap it so no
            # special care needs to be taken when fetching the result from the
            # primitive.
            if not isinstance(value, list):
                value = [value]

            on_to_clause_match = _ON_TO_CLAUSE_PATTERN.match(key)
            on_clause_match = _ON_CLAUSE_PATTERN.match(key)
            if on_to_clause_match:
                # We've come across the beginning of a compound statement
                # with both 'on' and 'to'.
                finalized_statement = statement

                # First, extract each statement's part of the string
                on, to = on_to_clause_match.groups()

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

            elif on_clause_match:
                # We've come across the beginning of an 'on' statement.
                # That means any previous statement we found is complete.
                finalized_statement = statement

                statement = OnStatement(
                    on=key, body=value, processor=self, call_stack=call_stack
                )

            elif _TO_CLAUSE_PATTERN.match(key):
                # We've come across the beginning of a 'to' statement.
                # That means any previous statement we found is complete.
                finalized_statement = statement

                statement = ToStatement(
                    to=key, body=value, processor=self, call_stack=call_stack
                )

            elif _TRY_CLAUSE_PATTERN.match(key):
                # We've come across the beginning of a 'try' statement.
                # That means any previous statement we found is complete.
                finalized_statement = statement

                statement = TryStatement(
                    body=value, processor=self, call_stack=call_stack
                )

            elif _ELSE_CLAUSE_PATTERN.match(key):
                _handle_else(statement, value)
            else:
                # Since this section is a dictionary, if there are no
                # markers to indicate the start or change of statement,
                # the current statement is complete and this section
                # is a primitive to be collected.
                finalized_statement = statement
                statement = None

        return statement, finalized_statement


def _handle_else(statement: Optional[Statement], else_body: Optional[typing.Grammar]):
    """Add else body to current statement.

    :param statement: The currently-active statement. If None it will be
                      ignored.
    :param else_body: The body of the else clause to add.

    :raises GrammarSyntaxError: If there isn't a currently-active
                                     statement.
    """

    if statement is None:
        raise GrammarSyntaxError(
            "'else' doesn't seem to correspond to an 'on' or 'try'"
        )

    statement.add_else(else_body)


class _StatementCollection:
    """Unique collection of statements to run at a later time."""

    def __init__(self) -> None:
        self._statements = []  # type: List[Statement]

    def add(self, statement: Optional[Statement]) -> None:
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
