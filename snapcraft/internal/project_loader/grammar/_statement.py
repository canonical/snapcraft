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

import re

from .errors import UnsatisfiedStatementError

_SELECTOR_PATTERN = re.compile(r'\Aon\s+([^,\s](?:,?[^,]+)*)\Z')
_WHITESPACE_PATTERN = re.compile(r'\A.*\s.*\Z')


class Statement:
    """Base class for all grammar statements"""

    def __init__(self, *, body, processor, call_stack, check_primitives=False):
        """Create an Statement instance.

        :param list call_stack: Call stack leading to this statement
        """
        if call_stack:
            self.__call_stack = call_stack
        else:
            self.__call_stack = []

        self._body = body
        self._processor = processor
        self._check_primitives = check_primitives
        self._else_bodies = []

        self.__processed_body = None
        self.__processed_else = None

    def add_else(self, else_body):
        """Add an 'else' clause to the statement.

        :param list else_body: The body of an 'else' clause.

        The 'else' clauses will be processed in the order they are added.
        """
        self._else_bodies.append(else_body)

    def process(self):
        """Process this statement.

        :return: Primitives as determined by evaluating the statement or its
                 else clauses.
        :rtype: set
        """
        if self._check():
            return self._process_body()
        else:
            return self._process_else()

    def _process_body(self):
        if self.__processed_body is None:
            self.__processed_body = self._processor.process(
                grammar=self._body,
                call_stack=self._call_stack(include_self=True))

        return self.__processed_body

    def _process_else(self):
        if self.__processed_else is None:
            self.__processed_else = set()
            for else_body in self._else_bodies:
                if not else_body:
                    # Handle the 'else fail' case.
                    raise UnsatisfiedStatementError(self)

                self.__processed_else = self._processor.process(
                    grammar=else_body,
                    call_stack=self._call_stack())
                if self.__processed_else:
                    if (not self._check_primitives or
                            self._validate_primitives(self.__processed_else)):
                        break

        return self.__processed_else

    def _validate_primitives(self, primitives):
        """Ensure that all primitives are valid.

        :param primitives: Iterable container of primitives.
        :param checker: callable accepting a single primitive, returning
                        true if it is valid
        :type checker: callable

        For example:
        >>> import tempfile
        >>> from snapcraft import ProjectOptions
        >>> def checker(primitive):
        ...     return 'invalid' not in primitive
        >>> with tempfile.TemporaryDirectory() as cache_dir:
        ...     _all_primitives_valid(['valid'], checker)
        ...     _all_primitives_valid(['valid', 'invalid'], checker)
        True
        False
        """

        for primitive in primitives:
            if not self._processor.checker(primitive):
                return False
        return True

    def _call_stack(self, *, include_self=False):
        if include_self:
            return self.__call_stack + [self]
        else:
            return self.__call_stack

    def _check(self):
        """Check if a statement main body should be processed.

        :return: True if main body should be processed, False if elses should
                 be processed.
        :rtype: bool
        """
        raise NotImplementedError('this must be implemented by child classes')

    def __eq__(self, other):
        raise NotImplementedError('this must be implemented by child classes')

    def __repr__(self):
        raise NotImplementedError('this must be implemented by child classes')
