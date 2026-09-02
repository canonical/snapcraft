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
"""Tests for loading real projects."""

import pathlib

import craft_parts
import pytest

from snapcraft.application import create_app
from snapcraft.models import Project


@pytest.fixture(autouse=True)
def _reset_craft_parts():
    craft_parts.Features.reset()


@pytest.fixture(
    params=[
        *(
            pytest.param(path, id=path.name)
            for path in (pathlib.Path(__file__).parent / "valid_projects").iterdir()
        ),
        # Get all the full snapcraft.yaml files from our spread tests.
        *(
            pytest.param(path.parent, id=str(path.parent).split("core", maxsplit=1)[1])
            for path in (pathlib.Path(__file__).parents[2] / "spread").glob(
                "core??/**/snapcraft.yaml"
            )
            if "core20" not in (d.name for d in path.parents)
        ),
    ]
)
def project_path(monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest):
    monkeypatch.chdir(request.param)


@pytest.mark.usefixtures("project_path")
def test_load_valid_project() -> None:
    app = create_app()
    app._configure_early_services()

    project_service = app.services.get("project")
    project_service.configure(platform=None, build_for=None)

    app._enable_craft_parts_features()

    project_service.get()

    project = project_service.get()
    assert isinstance(project, Project)
    # Check that the core22 methods still work.
    if project.base == "core22" or project.build_base == "core22":
        project.get_build_on()
        project.get_build_for()
