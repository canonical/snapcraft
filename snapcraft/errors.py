# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd.
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

"""Snapcraft error classes."""

from abc import ABC, abstractmethod
from typing import Optional

from craft_cli.errors import CraftError


class SnapcraftError(CraftError):
    """A snapcraft operation failed."""


class SnapcraftException(Exception, ABC):
    """Base class for Snapcraft Exceptions."""

    @abstractmethod
    def get_brief(self) -> str:
        """Concise, single-line description of the error."""

    @abstractmethod
    def get_resolution(self) -> str:
        """Concise suggestion for user to resolve error."""

    def get_details(self) -> Optional[str]:
        """Detailed technical information, if required for user to debug issue."""
        return None

    def get_docs_url(self) -> Optional[str]:
        """Link to documentation on docs.snapcraft.io, if applicable."""
        return None

    def get_exit_code(self) -> int:
        """Exit code to use when exiting snapcraft due to this exception."""
        return 2

    def get_reportable(self) -> bool:
        """Defines if error is reportable (an exception trace should be shown)."""
        return False

    def __str__(self) -> str:
        return self.get_brief()
