# This file is part of starcraft.
#
# Copyright 2023 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 3, as published
# by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
# SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Starcraft package demo."""
from importlib.metadata import version
from typing import List, Optional, Any

__version__ = version(__name__)


def hello(people: Optional[List[Any]] = None) -> None:
    """Says hello."""
    print("Hello *craft team!")
    if people:
        for person in people:
            print(f"Hello {person}!")
