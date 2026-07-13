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

"""Tests for Components in Snapcraft's Package service."""

from pathlib import Path
from textwrap import dedent
from unittest.mock import call

import pytest
from pytest_mock import MockerFixture

from snapcraft import linters, pack


@pytest.fixture
def extra_project_params(extra_project_params):
    extra_project_params["components"] = {
        "firstcomponent": {
            "type": "test",
            "summary": "first component",
            "description": "lorem ipsum",
            "version": "1.0",
            "hooks": {
                "install": {
                    "command-chain": ["test-command-chain"],
                    "environment": {
                        "test-variable-1": "test",
                        "test-variable-2": "test",
                    },
                    "plugs": ["home", "network"],
                    "passthrough": {"somefield": ["some", "value"]},
                },
                "post-refresh": {},
            },
        },
        "secondcomponent": {
            "type": "test",
            "summary": "second component",
            "description": "lorem ipsum",
            "version": "1.0",
        },
    }

    return extra_project_params


@pytest.fixture(params=["snap", "build-aux/snap"])
def project_assets_dir(in_project_path, request):
    assets_dir = in_project_path / request.param
    assets_dir.mkdir(parents=True)
    yield assets_dir


@pytest.mark.usefixtures("enable_partitions_feature")
def test_pack(default_project, fake_services, setup_project, mocker):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    lifecycle_service = fake_services.get("lifecycle")
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
    mock_pack_component = mocker.patch.object(pack, "pack_component")

    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")

    package_service.pack(prime_dir=Path("prime"), dest=Path())

    # Check that the regular pack.pack_snap() function was called with the correct
    # parameters.
    mock_pack_snap.assert_called_once_with(
        Path("prime"),
        name="default",
        version="1.0",
        compression="xz",
        output=".",
        target="amd64",
    )

    mock_pack_component.assert_has_calls(
        [
            call(
                lifecycle_service._work_dir
                / "partitions/component/firstcomponent/prime",
                compression="xz",
                output_dir=Path("."),
            ),
            call().__fspath__(),
            call(
                lifecycle_service._work_dir
                / "partitions/component/secondcomponent/prime",
                compression="xz",
                output_dir=Path("."),
            ),
            call().__fspath__(),
        ]
    )


@pytest.mark.usefixtures("enable_partitions_feature")
def test_pack_component_compression(
    default_project, fake_services, setup_project, mocker
):
    """Component-level compression overrides the snap-level compression."""
    project_data = default_project.marshal()
    project_data["compression"] = "xz"
    # 'firstcomponent' defines a compression, 'secondcomponent' defaults to the snaps compression
    project_data["components"]["firstcomponent"]["compression"] = "lzo"
    setup_project(fake_services, project_data)
    package_service = fake_services.get("package")
    lifecycle_service = fake_services.get("lifecycle")
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
    mock_pack_component = mocker.patch.object(pack, "pack_component")

    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")

    package_service.pack(prime_dir=Path("prime"), dest=Path())

    mock_pack_snap.assert_called_once_with(
        Path("prime"),
        name="default",
        version="1.0",
        compression="xz",
        output=".",
        target="amd64",
    )

    mock_pack_component.assert_has_calls(
        [
            call(
                lifecycle_service._work_dir
                / "partitions/component/firstcomponent/prime",
                compression="lzo",
                output_dir=Path("."),
            ),
            call().__fspath__(),
            call(
                lifecycle_service._work_dir
                / "partitions/component/secondcomponent/prime",
                compression="xz",
                output_dir=Path("."),
            ),
            call().__fspath__(),
        ]
    )


@pytest.mark.usefixtures("enable_partitions_feature")
def test_get_artifacts(default_project, fake_services, setup_project, tmp_path):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    (tmp_path / "artifacts").mkdir()
    package_service.set_output_dir(tmp_path / "artifacts")

    assert package_service.get_artifacts() == {
        None: tmp_path / "artifacts" / "default_1.0_amd64.snap",
        "firstcomponent": tmp_path / "artifacts" / "default+firstcomponent_1.0.comp",
        "secondcomponent": tmp_path / "artifacts" / "default+secondcomponent_1.0.comp",
    }


@pytest.mark.usefixtures("enable_partitions_feature")
def test_pack_artifact_component(
    default_project, fake_services, setup_project, mocker, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    lifecycle_service = fake_services.get("lifecycle")
    mock_pack_component = mocker.patch.object(
        pack,
        "pack_component",
        return_value="default+firstcomponent_1.0.comp",
    )

    artifact_path = tmp_path / "artifacts" / "default+firstcomponent_1.0.comp"
    package_service._pack(name="firstcomponent", path=artifact_path)

    mock_pack_component.assert_called_once_with(
        lifecycle_service._work_dir / "partitions/component/firstcomponent/prime",
        compression="xz",
        output_dir=artifact_path.parent,
    )


@pytest.mark.usefixtures("enable_partitions_feature")
def test_get_component_yaml(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service._get_component_yaml("firstcomponent") == dedent(
        """\
        component: default+firstcomponent
        type: test
        version: '1.0'
        summary: first component
        description: lorem ipsum
    """
    )

    assert package_service._get_component_yaml("secondcomponent") == dedent(
        """\
        component: default+secondcomponent
        type: test
        version: '1.0'
        summary: second component
        description: lorem ipsum
    """
    )


@pytest.mark.usefixtures("enable_partitions_feature")
def test_get_hook_assets_for_component(
    default_project, fake_services, setup_project, project_assets_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    component_prime_dir = tmp_path / "partitions" / "component" / "firstcomponent" / "prime"
    built_hooks_dir = component_prime_dir / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    built_hook = built_hooks_dir / "install"
    project_hook = project_assets_dir / "component" / "firstcomponent" / "hooks" / "configure"
    project_hook.parent.mkdir(parents=True)
    built_hook.write_text("built_install", encoding="utf-8")
    project_hook.write_text("project_configure", encoding="utf-8")
    built_hook.chmod(0o755)
    project_hook.chmod(0o755)

    assert package_service._get_hook_assets("firstcomponent") == [
        (built_hook, component_prime_dir / "meta" / "hooks" / "install"),
        (project_hook, component_prime_dir / "meta" / "hooks" / "configure"),
        ("#!/bin/true\n", component_prime_dir / "meta" / "hooks" / "post-refresh"),
    ]


@pytest.mark.usefixtures("enable_partitions_feature")
def test_get_gui_assets_for_component(
    default_project, fake_services, setup_project, project_assets_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    component_prime_dir = tmp_path / "partitions" / "component" / "firstcomponent" / "prime"
    desktop = project_assets_dir / "component" / "firstcomponent" / "gui" / "first.desktop"
    icon = project_assets_dir / "component" / "firstcomponent" / "gui" / "icon.png"
    desktop.parent.mkdir(parents=True)
    desktop.write_text("desktop_file", encoding="utf-8")
    icon.write_text("component_icon", encoding="utf-8")

    assert package_service._get_gui_assets("firstcomponent") == [
        (desktop, component_prime_dir / "meta" / "gui" / "first.desktop"),
        (icon, component_prime_dir / "meta" / "gui" / "icon.png"),
    ]


@pytest.mark.usefixtures("enable_partitions_feature")
def test_write_metadata(
    default_project,
    fake_services,
    setup_project,
    project_assets_dir,
    tmp_path,
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    lifecycle_service = fake_services.get("lifecycle")
    # create an executable to run via the command-chain
    command_chain_exe = (
        tmp_path
        / "partitions"
        / "component"
        / "firstcomponent"
        / "prime"
        / "test-command-chain"
    )
    command_chain_exe.parent.mkdir(parents=True)
    command_chain_exe.touch()
    command_chain_exe.chmod(0o755)

    # Create some hooks
    (project_assets_dir / "component/firstcomponent/hooks").mkdir(parents=True)
    component_hook = project_assets_dir / "component/firstcomponent/hooks/install"
    component_hook.write_text("install_hook")
    component_hook.chmod(0o755)
    component_gui = project_assets_dir / "component/firstcomponent/gui/icon.png"
    component_gui.parent.mkdir(parents=True)
    component_gui.write_text("component_icon")
    (project_assets_dir / "post-refresh").write_text("post-refresh")

    prime_dir = tmp_path / "prime"
    meta_dir = prime_dir / "meta"

    package_service.write_metadata(prime_dir)

    assert (meta_dir / "snap.yaml").read_text() == dedent(
        """\
        name: default
        version: '1.0'
        summary: default project
        description: default project
        license: MIT
        architectures:
        - amd64
        base: core24
        confinement: devmode
        grade: devel
        environment:
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        components:
          firstcomponent:
            summary: first component
            description: lorem ipsum
            type: test
            hooks:
              install:
                command-chain:
                - test-command-chain
                environment:
                  test-variable-1: test
                  test-variable-2: test
                plugs:
                - home
                - network
                passthrough:
                  somefield:
                  - some
                  - value
              post-refresh: {}
          secondcomponent:
            summary: second component
            description: lorem ipsum
            type: test
    """
    )

    assert (
        lifecycle_service.get_prime_dir("firstcomponent") / "meta" / "component.yaml"
    ).read_text() == dedent(
        """\
        component: default+firstcomponent
        type: test
        version: '1.0'
        summary: first component
        description: lorem ipsum
    """
    )

    assert (
        lifecycle_service.get_prime_dir("secondcomponent") / "meta" / "component.yaml"
    ).read_text() == dedent(
        """\
        component: default+secondcomponent
        type: test
        version: '1.0'
        summary: second component
        description: lorem ipsum
    """
    )
    component_meta_hook = (
        lifecycle_service.get_prime_dir("firstcomponent") / "meta" / "hooks" / "install"
    )
    assert component_meta_hook.read_text() == "install_hook"
    assert component_meta_hook.stat().st_mode & 0o111
    assert (
        lifecycle_service.get_prime_dir("firstcomponent") / "meta" / "gui" / "icon.png"
    ).read_text() == "component_icon"


@pytest.mark.usefixtures("enable_partitions_feature")
def test_write_metadata_generates_component_declared_hook_stub(
    default_project, fake_services, setup_project, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    lifecycle_service = fake_services.get("lifecycle")
    command_chain_exe = (
        lifecycle_service.get_prime_dir("firstcomponent") / "test-command-chain"
    )
    command_chain_exe.parent.mkdir(parents=True, exist_ok=True)
    command_chain_exe.touch()
    command_chain_exe.chmod(0o755)

    package_service.write_metadata(tmp_path / "prime")

    hook_path = (
        lifecycle_service.get_prime_dir("firstcomponent")
        / "meta"
        / "hooks"
        / "post-refresh"
    )
    assert hook_path.read_text() == "#!/bin/true\n"
    assert hook_path.stat().st_mode & 0o111


@pytest.mark.usefixtures("enable_partitions_feature")
def test_pack_component_makes_organized_meta_hook_executable(
    default_project, fake_services, setup_project, tmp_path, mocker: MockerFixture
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    lifecycle_service = fake_services.get("lifecycle")
    component_prime_dir = lifecycle_service.get_prime_dir("firstcomponent")
    remove_hook = component_prime_dir / "meta" / "hooks" / "remove"
    remove_hook.parent.mkdir(parents=True, exist_ok=True)
    remove_hook.write_text("#!/bin/true\n", encoding="utf-8")
    remove_hook.chmod(0o664)

    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")
    mock_pack_component = mocker.patch.object(pack, "pack_component", return_value="default+firstcomponent_1.0.comp")

    package_service._pack(
        name="firstcomponent",
        path=tmp_path / "default+firstcomponent_1.0.comp",
    )

    assert remove_hook.stat().st_mode & 0o111
    mock_pack_component.assert_called_once()
