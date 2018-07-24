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
from typing import Set, TYPE_CHECKING

from .errors import ToStatementSyntaxError
from . import typing
from ._statement import Statement

# Don't use circular imports unless type checking
if TYPE_CHECKING:
    from ._processor import GrammarProcessor  # noqa: F401

_SELECTOR_PATTERN = re.compile(r"\Ato\s+([^,\s](?:,?[^,]+)*)\Z")
_WHITESPACE_PATTERN = re.compile(r"\A.*\s.*\Z")


class ToStatement(Statement):
    """Process a 'to' statement in the grammar.

    For example:
    >>> import tempfile
    >>> from snapcraft import ProjectOptions
    >>> from snapcraft.internal.project_loader import grammar
    >>> def checker(primitive):
    ...     return True
    >>> options = ProjectOptions(target_deb_arch='i386')
    >>> processor = grammar.GrammarProcessor(None, options, checker)
    >>> clause = ToStatement(to='to armhf', body=['foo'], processor=processor)
    >>> clause.add_else(['bar'])
    >>> clause.process()
    {'bar'}
    """

    def __init__(
        self,
        *,
        to: str,
        body: typing.Grammar,
        processor: "GrammarProcessor",
        call_stack: typing.CallStack = None
    ) -> None:
        """Create a ToStatement instance.

        :param str to: The 'to <selectors>' part of the clause.
        :param list body: The body of the clause.
        :param GrammarProcessor process: GrammarProcessor to use for processing
                                         this statement.
        :param list call_stack: Call stack leading to this statement.
        """
        super().__init__(body=body, processor=processor, call_stack=call_stack)

        self.selectors = _extract_to_clause_selectors(to)

    def _check(self) -> bool:
        """Check if a statement main body should be processed.

        :return: True if main body should be processed, False if elses should
                 be processed.
        :rtype: bool
        """
        target_arch = self._processor.project.deb_arch

        # The only selector currently supported is the target arch. Since
        # selectors are matched with an AND, not OR, there should only be one
        # selector.
        return (len(self.selectors) == 1) and (target_arch in self.selectors)

    def __eq__(self, other) -> bool:
        if type(other) is type(self):
            return self.selectors == other.selectors

        return False

    def __str__(self) -> str:
        return "to {}".format(",".join(sorted(self.selectors)))


def _extract_to_clause_selectors(to: str) -> Set[str]:
    """Extract the list of selectors within a to clause.

    :param str to: The 'to <selector>' part of the 'to' clause.

    :return: Selectors found within the 'to' clause.
    :rtype: set

    For example:
    >>> _extract_to_clause_selectors('to amd64,i386') == {'amd64', 'i386'}
    True
    """

    match = _SELECTOR_PATTERN.match(to)

    try:
        selector_group = match.group(1)
    except AttributeError:
        raise ToStatementSyntaxError(to, message="selectors are missing")
    except IndexError:
        raise ToStatementSyntaxError(to)

    # This could be part of the _SELECTOR_PATTERN, but that would require us
    # to provide a very generic error when we can try to be more helpful.
    if _WHITESPACE_PATTERN.match(selector_group):
        raise ToStatementSyntaxError(
            to, message="spaces are not allowed in the selectors"
        )

    return {selector.strip() for selector in selector_group.split(",")}
