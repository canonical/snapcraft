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

from typing import TYPE_CHECKING

from . import typing
from ._statement import Statement

# Don't use circular imports unless type checking
if TYPE_CHECKING:
    from ._processor import GrammarProcessor  # noqa: F401


class TryStatement(Statement):
    """Process a 'try' statement in the grammar.

    For example:
    >>> from snapcraft import ProjectOptions
    >>> from ._processor import GrammarProcessor
    >>> def checker(primitive):
    ...     return 'invalid' not in primitive
    >>> options = ProjectOptions()
    >>> processor = GrammarProcessor(None, options, checker)
    >>> clause = TryStatement(body=['invalid'], processor=processor)
    >>> clause.add_else(['valid'])
    >>> clause.process()
    {'valid'}
    """

    def __init__(
        self,
        *,
        body: typing.Grammar,
        processor: "GrammarProcessor",
        call_stack: typing.CallStack = None
    ) -> None:
        """Create a TryStatement instance.

        :param list body: The body of the clause.
        :param GrammarProcessor process: GrammarProcessor to use for processing
                                         this statement.
        :param list call_stack: Call stack leading to this statement.
        """
        super().__init__(
            body=body, processor=processor, call_stack=call_stack, check_primitives=True
        )

    def _check(self) -> bool:
        """Check if a statement main body should be processed.

        :return: True if main body should be processed, False if elses should
                 be processed.
        :rtype: bool
        """
        return self._validate_primitives(self._process_body())

    def __eq__(self, other) -> bool:
        return False

    def __str__(self) -> str:
        return "try"
