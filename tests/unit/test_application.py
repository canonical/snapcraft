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
from typing import cast

import pytest
from craft_providers import bases

from snapcraft import application, services
from snapcraft.models.project import Architecture


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
    ("base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_default_architecture(base, expected_base):
    data = {
        "base": base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    archs = cast(list[Architecture], planner.architectures)
    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in archs]
        assert [build_info.build_on] in [a.build_on for a in archs]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


@pytest.mark.parametrize(
    ("base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_base_only(architectures, base, expected_base):
    data = {
        "architectures": architectures,
        "base": base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    archs = cast(list[Architecture], planner.architectures)
    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in archs]
        assert [build_info.build_on] in [a.build_on for a in archs]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


@pytest.mark.parametrize("base", ["core20", "core22", "core24"])
@pytest.mark.parametrize(
    ("build_base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_build_base(
    architectures, base, build_base, expected_base
):
    data = {
        "architectures": architectures,
        "base": base,
        "build-base": build_base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    archs = cast(list[Architecture], planner.architectures)
    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in archs]
        assert [build_info.build_on] in [a.build_on for a in archs]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


@pytest.mark.parametrize("base", ["core20", "core22", "core24"])
@pytest.mark.parametrize(
    ("build_base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_architecture_all(base, build_base, expected_base):
    data = {
        "architectures": [{"build-on": ["amd64"], "build-for": "all"}],
        "base": base,
        "build-base": build_base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    architectures = cast(list[Architecture], planner.architectures)
    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_on] in [a.build_on for a in architectures]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"

    assert "all" not in [a.build_on for a in architectures]


@pytest.mark.parametrize(
    ("base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
@pytest.mark.parametrize(
    ("project_type", "needs_build_base"),
    [
        ("app", False),
        ("base", True),
    ],
)
def test_build_planner_project_type(
    architectures, base, expected_base, project_type, needs_build_base
):
    data = {
        "architectures": architectures,
        "base": base,
        "type": project_type,
    }

    if needs_build_base:
        data["build-base"] = base

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    archs = cast(list[Architecture], planner.architectures)

    assert planner.project_type == project_type

    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in archs]
        assert [build_info.build_on] in [a.build_on for a in archs]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


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
