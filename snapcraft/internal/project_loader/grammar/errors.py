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

from snapcraft.internal import errors


class GrammarError(errors.SnapcraftError):
    """Base class for grammar-related errors."""

    pass


class GrammarSyntaxError(GrammarError):

    fmt = "Invalid grammar syntax: {message}"

    def __init__(self, message):
        super().__init__(message=message)


class OnStatementSyntaxError(GrammarSyntaxError):
    def __init__(self, on_statement, *, message=None):
        components = ["{!r} is not a valid 'on' clause".format(on_statement)]
        if message:
            components.append(message)
        super().__init__(message=": ".join(components))


class ToStatementSyntaxError(GrammarSyntaxError):
    def __init__(self, to_statement, *, message=None):
        components = ["{!r} is not a valid 'to' clause".format(to_statement)]
        if message:
            components.append(message)
        super().__init__(message=": ".join(components))


class UnsatisfiedStatementError(GrammarError):

    fmt = "Unable to satisfy {statement!r}, failure forced"

    def __init__(self, statement):
        super().__init__(statement=statement)
