# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import logging
import os
import shlex

from . import errors
from ._utils import _executable_is_valid

logger = logging.getLogger(__name__)


class Command:
    """Representation of a command string."""

    def __str__(self) -> str:
        return self.command

    def __init__(self, *, app_name: str, command_name: str, command: str) -> None:
        self._app_name = app_name
        self._command_name = command_name
        self.command = command

    @property
    def command_name(self) -> str:
        """Read-only to ensure consistency with app dictionary mappings."""
        return self._command_name

    def prime_command(self, *, prime_dir: str) -> str:
        """Finalize and prime command, massaging as necessary.

        Check if command is in prime_dir and raise exception if not valid."""

        command_parts = shlex.split(self.command)
        command_path = os.path.join(prime_dir, command_parts[0])

        if not os.path.exists(command_path):
            raise errors.InvalidAppCommandNotFound(
                command=self.command, app_name=self._app_name
            )

        if not _executable_is_valid(command_path):
            raise errors.InvalidAppCommandNotExecutable(
                command=self.command, app_name=self._app_name
            )

        return self.command
