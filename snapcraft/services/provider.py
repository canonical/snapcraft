# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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
"""Snapcraft Provider service."""
from craft_application import services
from overrides import override


class Provider(services.ProviderService):
    """Provider service for snapcraft."""

    @override
    def setup(self) -> None:
        """Configure the provider for snapcraft"""
        super().setup()
        # Required for adding apt repositories.
        self.packages.extend(("gpg", "dirmngr"))
