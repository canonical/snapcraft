# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Tests for the init service."""

import importlib.resources
import pathlib

import pytest

from snapcraft import errors
from snapcraft.parts.yaml_utils import _SNAP_PROJECT_FILES


@pytest.fixture()
def init_service(fake_services):
    from snapcraft.application import (  # noqa: PLC0415 (import-outside-top-level)
        APP_METADATA,
    )
    from snapcraft.services import Init  # noqa: PLC0415 (import-outside-top-level)

    service = Init(app=APP_METADATA, services=fake_services)

    return service


def template_dir():
    resource = importlib.resources.files("snapcraft") / "templates" / "simple"
    with importlib.resources.as_file(resource) as resource_file:
        return resource_file


@pytest.mark.parametrize(
    "name",
    [
        "name",
        "name-with-dashes",
        "name0123",
        "0123name",
        "a234567890123456789012345678901234567890",
    ],
)
def test_init_valid_name(name, init_service, new_dir, emitter):
    """Initialise a project with a valid snap name."""
    init_service.initialise_project(
        project_dir=new_dir, project_name=name, template_dir=template_dir()
    )

    assert (new_dir / "snap/snapcraft.yaml").exists()
    emitter.assert_message(
        "See https://documentation.ubuntu.com/snapcraft/stable/reference/project-file "
        "for reference information about the snapcraft.yaml format."
    )


@pytest.mark.parametrize(
    "name,error",
    [
        ("name_with_underscores", "snap names can only use"),
        ("name-with-UPPERCASE", "snap names can only use"),
        ("name with spaces", "snap names can only use"),
        ("-name-starts-with-hyphen", "snap names cannot start with a hyphen"),
        ("name-ends-with-hyphen-", "snap names cannot end with a hyphen"),
        ("name-has--two-hyphens", "snap names cannot have two hyphens in a row"),
        ("123456", "snap names can only use"),
        (
            "a2345678901234567890123456789012345678901",
            "snap names must be 40 characters or less",
        ),
    ],
)
def test_init_invalid_name(name, error, init_service, new_dir):
    """Error on invalid names."""
    expected_error = f"Invalid snap name {name!r}: {error}."

    with pytest.raises(errors.SnapcraftError, match=expected_error):
        init_service.initialise_project(
            project_dir=new_dir,
            project_name=name,
            template_dir=template_dir(),
        )


def test_init_snap_dir_exists(init_service, new_dir, emitter):
    """Initialise a project even if the 'snap/' directory already exists."""
    snapcraft_yaml = new_dir / "snap/snapcraft.yaml"
    snapcraft_yaml.parent.mkdir(parents=True)

    init_service.check_for_existing_files(
        project_dir=new_dir, template_dir=template_dir()
    )
    init_service.initialise_project(
        project_dir=new_dir, project_name="test-snap-name", template_dir=template_dir()
    )

    assert snapcraft_yaml.exists()
    emitter.assert_message(
        "See https://documentation.ubuntu.com/snapcraft/stable/reference/project-file "
        "for reference information about the snapcraft.yaml format."
    )


@pytest.mark.parametrize(
    "project_file", [project.project_file for project in _SNAP_PROJECT_FILES]
)
def test_init_exists(init_service, project_file, in_project_path):
    """Raise an error if a snapcraft.yaml file already exists."""
    snapcraft_yaml = pathlib.Path(in_project_path) / project_file
    snapcraft_yaml.parent.mkdir(parents=True, exist_ok=True)
    snapcraft_yaml.touch()
    expected = (
        "Could not initialise a new snapcraft project because "
        f"{str(project_file)!r} already exists"
    )

    with pytest.raises(errors.SnapcraftError, match=expected):
        init_service.check_for_existing_files(
            project_dir=in_project_path, template_dir=template_dir()
        )
