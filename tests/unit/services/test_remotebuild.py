# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Snapcraft Remote Build service tests."""

import pytest
from craft_application import launchpad


@pytest.mark.usefixtures("emitter")
def test_recipe_class(fake_services):
    """Snapcraft should use SnapRecipes."""
    service = fake_services.get("remote_build")

    assert service.RecipeClass == launchpad.SnapRecipe
