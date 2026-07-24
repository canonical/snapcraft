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

import re
from pathlib import Path
from textwrap import dedent
from unittest.mock import call

import pytest
from craft_application.services.package import PackageFileEntry

from snapcraft import linters, pack
from snapcraft.errors import SnapcraftError


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
def project_assets_dir(new_dir, request):
    assets_dir = new_dir / request.param
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
def test_get_artifacts(default_project, fake_services, setup_project, tmp_path):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    package_service.set_output_dir(tmp_path)

    assert package_service.get_artifacts() == {
        None: tmp_path / "default_1.0_amd64.snap",
        "firstcomponent": tmp_path / "default+firstcomponent_1.0.comp",
        "secondcomponent": tmp_path / "default+secondcomponent_1.0.comp",
    }


@pytest.mark.usefixtures("enable_partitions_feature")
def test_pack_component_artifact(
    default_project, fake_services, setup_project, mocker, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    package_service.set_output_dir(tmp_path)

    mock_pack_component = mocker.patch.object(
        pack, "pack_component", return_value="default+firstcomponent_1.0.comp"
    )
    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")

    package_service._pack(
        name="firstcomponent", path=tmp_path / "default+firstcomponent_1.0.comp"
    )

    mock_pack_component.assert_called_once_with(
        tmp_path / "partitions/component/firstcomponent/prime",
        compression="xz",
        output_dir=tmp_path,
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
def test_get_component_yaml(default_project, fake_services, setup_project):
    """Mediation is invoked with the bare component name used by get_artifacts()."""
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    expected = dedent(
        """\
        component: default+firstcomponent
        type: test
        version: '1.0'
        summary: first component
        description: lorem ipsum
        """
    )

    # Bare component name is the artifact key from get_artifacts() and the form
    # passed through the mediation pipeline.
    assert package_service._get_component_yaml("firstcomponent") == expected
    # Fully qualified partition name remains accepted for direct callers.
    assert package_service._get_component_yaml("component/firstcomponent") == expected


@pytest.mark.usefixtures("enable_partitions_feature")
def test_get_component_yaml_unknown_component(
    default_project, fake_services, setup_project
):
    """Requesting an unknown component surfaces a clear error."""
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    with pytest.raises(SnapcraftError, match="Component does not exist."):
        package_service._get_component_yaml("component/does-not-exist")


@pytest.mark.usefixtures("enable_partitions_feature")
def test_component_yaml_not_registered_for_default_partition(
    default_project, fake_services, setup_project
):
    """The component.yaml generator must run for the snap artifact."""
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    entries = [
        entry
        for entry in package_service._package_file_registry
        if isinstance(entry, PackageFileEntry)
        and entry.method_name == "_get_component_yaml"
    ]
    assert entries, "component.yaml generator is not registered"
    entry = entries[0]

    assert re.fullmatch(entry.partition_re, "default") is None
    assert re.fullmatch(entry.partition_re, "firstcomponent") is not None


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
    (project_assets_dir / "component/firstcomponent/hooks/install").write_text(
        "install_hook"
    )
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
