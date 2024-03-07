# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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
"""Unit tests for application classes."""
import json
import os
from textwrap import dedent

import pytest
from craft_application.errors import CraftValidationError
from craft_application.models import BuildInfo
from craft_providers.bases import BaseName

from snapcraft import application, services
from snapcraft.models.project import Architecture
from snapcraft.utils import get_host_architecture


@pytest.fixture(
    params=[
        ["amd64"],
        ["riscv64"],
        ["amd64", "riscv64", "s390x"],
        [Architecture(build_on="amd64", build_for="riscv64")],
    ]
)
def architectures(request):
    return request.param


@pytest.mark.parametrize(
    ("platforms", "expected_build_infos"),
    [
        pytest.param(
            {"amd64": None},
            [
                BuildInfo(
                    build_on="amd64",
                    build_for="amd64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="amd64",
                )
            ],
            id="single_platform_as_arch",
        ),
        pytest.param(
            {
                "arm64": {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["arm64"],
                },
            },
            [
                BuildInfo(
                    build_on="arm64",
                    build_for="arm64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="arm64",
                ),
                BuildInfo(
                    build_on="armhf",
                    build_for="arm64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="arm64",
                ),
            ],
            id="multiple_build_on",
        ),
        pytest.param(
            {
                "amd64v2": {
                    "build-on": ["amd64"],
                    "build-for": "amd64",
                },
            },
            [
                BuildInfo(
                    build_on="amd64",
                    build_for="amd64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="amd64v2",
                )
            ],
            id="custom_platform_name",
        ),
    ],
)
def test_build_planner_get_build_plan(platforms, expected_build_infos):
    """Test `get_build_plan()` function with different platforms."""
    planner = application.SnapcraftBuildPlanner.parse_obj(
        {"name": "test-snap", "base": "core24", "platforms": platforms}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == expected_build_infos


def test_platform_default():
    """Default value for platforms is the host architecture."""
    planner = application.SnapcraftBuildPlanner.parse_obj(
        {"name": "test-snap", "base": "core24"}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            build_on=get_host_architecture(),
            build_for=get_host_architecture(),
            base=BaseName(name="ubuntu", version="24.04"),
            platform=get_host_architecture(),
        )
    ]


def test_build_planner_get_build_plan_base(mocker):
    """Test `get_build_plan()` uses the correct base."""
    mock_get_effective_base = mocker.patch(
        "snapcraft.application.get_effective_base", return_value="core24"
    )
    planner = application.SnapcraftBuildPlanner.parse_obj(
        {
            "name": "test-snap",
            "base": "test-base",
            "build-base": "test-build-base",
            "platforms": {"amd64": None},
            "project_type": "test-type",
        }
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            platform="amd64",
            build_on="amd64",
            build_for="amd64",
            base=BaseName(name="ubuntu", version="24.04"),
        )
    ]
    mock_get_effective_base.assert_called_once_with(
        base="test-base",
        build_base="test-build-base",
        project_type="test-type",
        name="test-snap",
    )


def test_project_platform_error_has_context():
    """Platform validation errors include which platform entry is invalid."""
    with pytest.raises(CraftValidationError) as raised:
        application.SnapcraftBuildPlanner.parse_obj(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"test-platform": {"build-for": ["amd64"]}},
                "project_type": "test-type",
            }
        )

    assert "'test-platform': 'build_for' expects 'build_on'" in str(raised.value)


def test_project_platform_mismatch():
    """Raise an error if platform name and build-for are valid but different archs."""
    with pytest.raises(CraftValidationError) as raised:
        application.SnapcraftBuildPlanner.parse_obj(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"amd64": {"build-on": ["amd64"], "build-for": ["arm64"]}},
                "project_type": "test-type",
            }
        )

    assert (
        "if 'build_for' is provided and the platform entry label "
        "corresponds to a valid architecture, then both values must match. "
        "amd64 != arm64" in str(raised.value)
    )


def test_project_platform_unknown_name():
    """Raise an error if an empty platform is not a valid architecture."""
    with pytest.raises(CraftValidationError) as raised:
        application.SnapcraftBuildPlanner.parse_obj(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"unknown": None},
                "project_type": "test-type",
            }
        )

    assert (
        "platform entry label must correspond to a valid architecture "
        "if 'build-for' is not provided." in str(raised.value)
    )


@pytest.mark.parametrize("env_vars", application.MAPPED_ENV_VARS.items())
def test_application_map_build_on_env_var(monkeypatch, env_vars):
    """Test that instantiating the Snapcraft application class will set the value of the
    SNAPCRAFT_* environment variables to CRAFT_*.
    """
    craft_var = env_vars[0]
    snapcraft_var = env_vars[1]
    env_val = "woop"

    monkeypatch.setenv(snapcraft_var, env_val)
    assert os.getenv(craft_var) is None

    snapcraft_services = services.SnapcraftServiceFactory(app=application.APP_METADATA)
    application.Snapcraft(app=application.APP_METADATA, services=snapcraft_services)

    assert os.getenv(craft_var) == env_val
    assert os.getenv(snapcraft_var) == env_val


@pytest.fixture()
def extension_source(default_project):
    source = default_project.marshal()
    source["confinement"] = "strict"
    source["apps"] = {
        "app1": {
            "command": "app1",
            "extensions": ["fake-extension"],
        }
    }
    return source


@pytest.mark.usefixtures("fake_extension")
def test_application_expand_extensions(emitter, monkeypatch, extension_source, new_dir):
    monkeypatch.setenv("CRAFT_DEBUG", "1")

    (new_dir / "snap").mkdir()
    (new_dir / "snap/snapcraft.yaml").write_text(json.dumps(extension_source))

    monkeypatch.setattr("sys.argv", ["snapcraft", "expand-extensions"])
    application.main()
    emitter.assert_message(
        dedent(
            """\
            name: default
            version: '1.0'
            summary: default project
            description: default project
            base: core24
            build-base: devel
            license: MIT
            parts:
                fake-extension/fake-part:
                    plugin: nil
            confinement: strict
            grade: devel
            apps:
                app1:
                    command: app1
                    plugs:
                    - fake-plug
        """
        )
    )


@pytest.mark.usefixtures("fake_extension")
def test_application_build_with_extensions(monkeypatch, extension_source, new_dir):
    """Test that extensions are correctly applied in regular builds."""
    monkeypatch.setenv("CRAFT_DEBUG", "1")

    project_path = new_dir / "snap/snapcraft.yaml"
    (new_dir / "snap").mkdir()
    project_path.write_text(json.dumps(extension_source))

    # Calling a lifecycle command will create a Project. Creating a Project
    # without applying the extensions will fail because the "extensions" field
    # will still be present on the yaml data, so it's enough to run "pull".
    monkeypatch.setattr("sys.argv", ["snapcraft", "pull", "--destructive-mode"])
    app = application.create_app()
    app.run()

    project = app.get_project()
    assert "fake-extension/fake-part" in project.parts
