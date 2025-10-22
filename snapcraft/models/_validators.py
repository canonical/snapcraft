# Copyright 2025 Canonical Ltd.
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
"""Generic validators used for multiple models."""

import re


def validate_command_chain(command_chains: list[str]) -> list[str]:
    """Validate command_chain."""
    for command_chain in command_chains:
        if not re.match(r"^[A-Za-z0-9/._#:$-]*$", command_chain):
            raise ValueError(
                f"{command_chain!r} is not a valid command chain. Command chain entries must "
                "be strings, and can only use ASCII alphanumeric characters and the following "
                "special characters: / . _ # : $ -"
            )
    return command_chains
