# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from typing import List, TYPE_CHECKING

from . import typing
from ._statement import Statement

# Don't use circular imports unless type checking
if TYPE_CHECKING:
    from ._processor import GrammarProcessor  # noqa: F401


class CompoundStatement(Statement):
    """Multiple statements that need to be treated as a group."""

    def __init__(
        self,
        *,
        statements: List[Statement],
        body: typing.Grammar,
        processor: "GrammarProcessor",
        call_stack: typing.CallStack = None
    ) -> None:
        """Create an CompoundStatement instance.

        :param list statements: List of compound statements
        :param list body: The body of the clause.
        :param GrammarProcessor process: GrammarProcessor to use for processing
                                         this statement.
        :param list call_stack: Call stack leading to this statement.
        """
        super().__init__(body=body, processor=processor, call_stack=call_stack)

        self.statements = statements

    def _check(self) -> bool:
        """Check if each statement checks True, in order

        :return: True if each statement agrees that they should be processed,
                 False if elses should be processed.
        :rtype: bool
        """
        for statement in self.statements:
            if not statement._check():
                return False

        return True

    def __eq__(self, other) -> bool:
        if type(other) is type(self):
            return self.statements == other.statements

        return False

    def __str__(self) -> str:
        representation = ""
        for statement in self.statements:
            representation += "{!s} ".format(statement)

        return representation.strip()
