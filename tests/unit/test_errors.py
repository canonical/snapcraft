# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snapcraft error tests."""

import pytest

from snapcraft import errors


@pytest.mark.parametrize("base,channel", [("core", "4.x"), ("core18", "7.x")])
def test_maintenance_base(base, channel):
    message = f"{base!r} is not supported on this version of Snapcraft."
    resolution = f"Install from or refresh to the {channel!r} channel."

    error = errors.MaintenanceBase(base)

    assert str(error) == message
    assert error.resolution == resolution
    assert error.docs_url == "https://snapcraft.io/docs/base-snaps"


def test_maintenance_base_fallback():
    """For when the base is not explicitly handled in the exception."""
    error = errors.MaintenanceBase("unknown-base")

    assert str(error) == "'unknown-base' is not supported on this version of Snapcraft."
    assert error.resolution is None
    assert error.docs_url == "https://snapcraft.io/docs/base-snaps"
