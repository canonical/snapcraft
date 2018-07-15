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

import snapcraft

from .errors import OnStatementSyntaxError
from . import typing
from ._statement import Statement

# Don't use circular imports unless type checking
if TYPE_CHECKING:
    from ._processor import GrammarProcessor  # noqa: F401

_SELECTOR_PATTERN = re.compile(r"\Aon\s+([^,\s](?:,?[^,]+)*)\Z")
_WHITESPACE_PATTERN = re.compile(r"\A.*\s.*\Z")


class OnStatement(Statement):
    """Process an 'on' statement in the grammar.

    For example:
    >>> from snapcraft import ProjectOptions
    >>> from snapcraft.internal.project_loader import grammar
    >>> from unittest import mock
    >>>
    >>> def checker(primitive):
    ...     return True
    >>> options = ProjectOptions()
    >>> processor = grammar.GrammarProcessor(None, options, checker)
    >>>
    >>> clause = OnStatement(on='on amd64', body=['foo'], processor=processor)
    >>> clause.add_else(['bar'])
    >>> with mock.patch('platform.machine') as mock_machine:
    ...     # Pretend this machine is an i686, not amd64
    ...     mock_machine.return_value = 'i686'
    ...     clause.process()
    {'bar'}
    """

    def __init__(
        self,
        *,
        on: str,
        body: typing.Grammar,
        processor: "GrammarProcessor",
        call_stack: typing.CallStack = None
    ) -> None:
        """Create an OnStatement instance.

        :param str on: The 'on <selectors>' part of the clause.
        :param list body: The body of the clause.
        :param GrammarProcessor process: GrammarProcessor to use for processing
                                         this statement.
        :param list call_stack: Call stack leading to this statement.
        """
        super().__init__(body=body, processor=processor, call_stack=call_stack)

        self.selectors = _extract_on_clause_selectors(on)

    def _check(self) -> bool:
        """Check if a statement main body should be processed.

        :return: True if main body should be processed, False if elses should
                 be processed.
        :rtype: bool
        """
        # A new ProjectOptions instance defaults to the host architecture
        # whereas self._project_options would yield the target architecture
        host_arch = snapcraft.ProjectOptions().deb_arch

        # The only selector currently supported is the host arch. Since
        # selectors are matched with an AND, not OR, there should only be one
        # selector.
        return (len(self.selectors) == 1) and (host_arch in self.selectors)

    def __eq__(self, other) -> bool:
        if type(other) is type(self):
            return self.selectors == other.selectors

        return False

    def __str__(self) -> str:
        return "on {}".format(",".join(sorted(self.selectors)))


def _extract_on_clause_selectors(on: str) -> Set[str]:
    """Extract the list of selectors within an on clause.

    :param str on: The 'on <selector>' part of the 'on' clause.

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
        raise OnStatementSyntaxError(on, message="selectors are missing")
    except IndexError:
        raise OnStatementSyntaxError(on)

    # This could be part of the _SELECTOR_PATTERN, but that would require us
    # to provide a very generic error when we can try to be more helpful.
    if _WHITESPACE_PATTERN.match(selector_group):
        raise OnStatementSyntaxError(
            on, message="spaces are not allowed in the selectors"
        )

    return {selector.strip() for selector in selector_group.split(",")}
