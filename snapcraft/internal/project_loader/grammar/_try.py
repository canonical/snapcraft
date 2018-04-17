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

from ._statement import Statement


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

    def __init__(self, *, body, processor, call_stack=None):
        """Create an TryStatement instance.

        :param list body: The body of the 'try' clause.
        :param project_options: Instance of ProjectOptions to use to process
                                clause.
        :type project_options: snapcraft.ProjectOptions
        :param checker: callable accepting a single primitive, returning
                        true if it is valid
        :type checker: callable
        """
        super().__init__(
            body=body, processor=processor, call_stack=call_stack,
            check_primitives=True)

    def _check(self):
        return self._validate_primitives(self._process_body())

    def __repr__(self):
        return "'try'"

    def __eq__(self, other):
        return False
