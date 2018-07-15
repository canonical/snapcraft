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

from snapcraft import project
from snapcraft.internal.project_loader.grammar import (
    CompoundStatement,
    Statement,
    ToStatement,
    typing,
)


def _is_or_contains_to_statement(statement: Statement) -> bool:
    # If the statement is a ToStatement, then return True.
    if isinstance(statement, ToStatement):
        return True

    # If the statement is a CompoundStatement, check if a ToStatement is
    # contained within its statements. If so, return True.
    if isinstance(statement, CompoundStatement):
        return any(isinstance(s, ToStatement) for s in statement.statements)

    return False


def package_transformer(
    call_stack: typing.CallStack, package_name: str, project: project.Project
) -> str:
    if any(_is_or_contains_to_statement(s) for s in call_stack):
        if ":" not in package_name:
            # deb_arch is target arch or host arch if both are the same
            package_name += ":{}".format(project.deb_arch)

    return package_name
