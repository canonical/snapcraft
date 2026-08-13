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

"""Tests for the Snapcraft Package service."""

import datetime
import os
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from pathlib import Path
from textwrap import dedent
from typing import Any, cast

import pytest
import yaml
from craft_application import ServiceFactory, util
from craft_cli.pytest_plugin import RecordingEmitter
from pytest_mock import MockerFixture

from snapcraft import __version__, linters, meta, models, pack
from snapcraft.errors import SnapcraftError, SnapcraftPrecreationEscapesPrimeError
from snapcraft.meta import ExtractedMetadata
from snapcraft.parts import extract_metadata, update_metadata
from snapcraft.parts.setup_assets import get_mediated_icon_asset
from snapcraft.services import Package


def test_pack(default_project, fake_services, setup_project, mocker):
    setup_project(fake_services, default_project.marshal())
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")
    package_service = fake_services.get("package")

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


def test_pack_target_arch(
    default_project, fake_services, setup_project, mocker, tmp_path
):
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core24",
            "platforms": {"s390x": {"build-on": ["amd64"], "build-for": ["s390x"]}},
        },
    )
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")
    package_service = fake_services.get("package")

    package_service.pack(prime_dir=tmp_path / "prime", dest=tmp_path)

    assert mock_pack_snap.call_args.kwargs["target"] == "s390x"


def test_metadata(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service.metadata == meta.SnapMetadata.unmarshal(
        {
            "name": "default",
            "title": None,
            "version": "1.0",
            "summary": "default project",
            "description": "default project",
            "license": "MIT",
            "type": None,
            "architectures": ["amd64"],
            "base": "core24",
            "assumes": None,
            "epoch": None,
            "apps": None,
            "confinement": "devmode",
            "grade": "devel",
            "environment": {
                "LD_LIBRARY_PATH": "${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}",
                "PATH": "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH",
            },
            "plugs": None,
            "slots": None,
            "hooks": None,
            "layout": None,
            "system_usernames": None,
            "provenance": None,
            "links": None,
            "components": None,
        }
    )


def test_write_metadata(default_project, fake_services, setup_project, new_dir):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    prime_dir = new_dir / "prime"
    meta_dir = prime_dir / "meta"

    package_service.write_metadata(prime_dir)

    assert (meta_dir / "snap.yaml").read_text() == dedent("""\
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
    """)

    assert not (prime_dir / "snap" / "manifest.yaml").exists()


def test_get_snap_yaml(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service._get_snap_yaml() == util.dump_yaml(
        package_service.metadata.marshal()
    )


def test_get_manifest_yaml_disabled(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = cast(Package, fake_services.get("package"))

    assert package_service._get_manifest_yaml() is False


def test_get_manifest_yaml_enabled(
    monkeypatch, default_project, fake_services, setup_project
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal())
    package_service = cast(Package, fake_services.get("package"))

    manifest_dict = yaml.safe_load(package_service._get_manifest_yaml())
    manifest = models.Manifest.model_validate(manifest_dict)

    assert manifest.snapcraft_version == __version__
    assert manifest.name == package_service.metadata.name


def test_get_gadget_yaml(fake_services, setup_project):
    project = {
        "name": "pc",
        "type": "gadget",
        "base": "core24",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))
    project_dir = package_service._services.lifecycle.project_info.project_dir
    (project_dir / "gadget.yaml").write_text("volumes: {}\n", encoding="utf-8")

    assert package_service._get_gadget_yaml() == "volumes: {}\n"


def test_get_gadget_yaml_missing_raises(fake_services, setup_project):
    project = {
        "name": "pc",
        "type": "gadget",
        "base": "core24",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))

    with pytest.raises(
        SnapcraftError, match="gadget.yaml is required for gadget snaps"
    ):
        package_service._get_gadget_yaml()


def test_get_kernel_yaml(fake_services, setup_project):
    project = {
        "name": "custom-kernel",
        "type": "kernel",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
        "build-base": "core24",
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))
    project_dir = package_service._services.lifecycle.project_info.project_dir
    (project_dir / "kernel.yaml").write_text(
        "kernel-key: kernel-value\n", encoding="utf-8"
    )

    assert package_service._get_kernel_yaml() == "kernel-key: kernel-value\n"


def test_get_kernel_yaml_missing_returns_none(fake_services, setup_project):
    project = {
        "name": "custom-kernel",
        "type": "kernel",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
        "build-base": "core24",
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))

    assert package_service._get_kernel_yaml() is None


def test_get_gadget_yaml_non_gadget_returns_false(
    default_project, fake_services, setup_project
):
    """Non-gadget projects should not touch meta/gadget.yaml via mediation."""
    setup_project(fake_services, default_project.marshal())
    package_service = cast(Package, fake_services.get("package"))

    assert package_service._get_gadget_yaml() is False


def test_get_kernel_yaml_non_kernel_returns_false(
    default_project, fake_services, setup_project
):
    """Non-kernel projects should not touch meta/kernel.yaml via mediation."""
    setup_project(fake_services, default_project.marshal())
    package_service = cast(Package, fake_services.get("package"))

    assert package_service._get_kernel_yaml() is False


def test_get_artifacts(default_project, fake_services, setup_project, tmp_path):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    package_service.set_output_dir(tmp_path)

    assert package_service.get_artifacts() == {
        None: tmp_path / "default_1.0_amd64.snap"
    }


def test_pack_artifact(default_project, fake_services, setup_project, mocker, tmp_path):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    package_service.set_output_dir(tmp_path)

    mock_pack_snap = mocker.patch.object(
        pack, "pack_snap", return_value="default_1.0_amd64.snap"
    )
    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")

    package_service._pack(path=tmp_path / "default_1.0_amd64.snap")

    mock_pack_snap.assert_called_once_with(
        tmp_path / "prime",
        name="default",
        version="1.0",
        compression="xz",
        output=str(tmp_path / "default_1.0_amd64.snap"),
        target="amd64",
    )


def test_supports_conditional_repack(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service.supports_conditional_repack is True


def test_write_metadata_with_manifest(
    monkeypatch, default_project, fake_services, setup_project, tmp_path
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")

    prime_dir = tmp_path / "prime"
    meta_dir = prime_dir / "meta"

    package_service.write_metadata(prime_dir)

    snap_yaml = yaml.safe_load((meta_dir / "snap.yaml").read_text())

    # This will be different every time due to started_at differing, we can check
    # that it's a valid manifest and compare some fields to snap.yaml.
    manifest_dict = yaml.safe_load((prime_dir / "snap" / "manifest.yaml").read_text())
    manifest = models.Manifest.model_validate(manifest_dict)

    assert manifest.snapcraft_version == __version__
    assert (
        datetime.datetime.fromisoformat(manifest.snapcraft_started_at[:-1])
        == fake_services.lifecycle._start_time
    )
    assert manifest.name == snap_yaml["name"]
    assert manifest.grade == snap_yaml["grade"]
    assert manifest.architectures == snap_yaml["architectures"]
    assert not (prime_dir / "snap" / "snapcraft.yaml").exists()


def test_write_metadata_writes_gadget_yaml(fake_services, setup_project, tmp_path):
    project = {
        "name": "pc",
        "type": "gadget",
        "base": "core24",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))
    project_dir = package_service._services.lifecycle.project_info.project_dir
    (project_dir / "gadget.yaml").write_text("volumes: {}\n", encoding="utf-8")

    prime_dir = tmp_path / "prime"
    package_service.write_metadata(prime_dir)

    assert (prime_dir / "meta" / "gadget.yaml").read_text(
        encoding="utf-8"
    ) == "volumes: {}\n"


def test_write_metadata_writes_kernel_yaml(fake_services, setup_project, tmp_path):
    project = {
        "name": "custom-kernel",
        "type": "kernel",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
        "build-base": "core24",
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))
    project_dir = package_service._services.lifecycle.project_info.project_dir
    (project_dir / "kernel.yaml").write_text(
        "kernel-key: kernel-value\n", encoding="utf-8"
    )

    prime_dir = tmp_path / "prime"
    package_service.write_metadata(prime_dir)

    assert (prime_dir / "meta" / "kernel.yaml").read_text(
        encoding="utf-8"
    ) == "kernel-key: kernel-value\n"


def test_write_metadata_missing_kernel_yaml_leaves_no_file(
    fake_services, setup_project, tmp_path
):
    project = {
        "name": "custom-kernel",
        "type": "kernel",
        "confinement": "strict",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "grade": "stable",
        "parts": {},
        "build-base": "core24",
    }
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))

    prime_dir = tmp_path / "prime"
    package_service.write_metadata(prime_dir)

    assert not (prime_dir / "meta" / "kernel.yaml").exists()


@pytest.fixture(params=["snap", "build-aux/snap"])
def project_hooks_dir(in_project_path, request):
    hooks_dir = in_project_path / request.param / "hooks"
    hooks_dir.mkdir(parents=True)
    yield hooks_dir


def test_write_metadata_with_project_hooks(
    default_project, fake_services, setup_project, project_hooks_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    # Create some hooks
    (project_hooks_dir / "configure").write_text("configure_hook")
    (project_hooks_dir / "install").write_text("install_hook")

    prime_dir = tmp_path / "prime"
    meta_dir = prime_dir / "meta"

    package_service.write_metadata(prime_dir)

    assert (meta_dir / "snap.yaml").read_text() == dedent("""\
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
    """)

    # Hooks are mediated to snap/hooks by the packaging flow, not copied by
    # write_metadata.
    assert not (meta_dir / "hooks").exists()


def test_write_metadata_with_built_hooks(
    default_project, fake_services, setup_project, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    # Create some hooks
    prime_dir = tmp_path / "prime"
    built_hooks_dir = prime_dir / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    (built_hooks_dir / "configure").write_text("configure_hook")
    (built_hooks_dir / "install").write_text("install_hook")

    package_service.write_metadata(prime_dir)

    meta_dir = prime_dir / "meta"
    assert (meta_dir / "snap.yaml").read_text() == dedent("""\
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
    """)

    # Built hooks are not copied into meta/hooks by write_metadata; the
    # mediated packaging flow provisions meta/hooks during asset materialization.
    assert not (meta_dir / "hooks").exists()
    assert (built_hooks_dir / "configure").read_text() == "configure_hook"
    assert (built_hooks_dir / "install").read_text() == "install_hook"


def test_write_metadata_with_project_gui(
    default_project, fake_services, setup_project, in_project_path, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    # Create some gui
    project_gui_dir = in_project_path / "snap" / "gui"
    project_gui_dir.mkdir(parents=True)
    (project_gui_dir / "default.default.desktop").write_text("desktop_file")
    (project_gui_dir / "icon.png").write_text("package_png_icon")

    prime_dir = tmp_path / "prime"
    meta_dir = prime_dir / "meta"

    package_service.write_metadata(prime_dir)

    assert (meta_dir / "snap.yaml").read_text() == dedent("""\
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
    """)

    # GUI assets are mediated to meta/gui by the packaging flow, not copied by
    # write_metadata. The directory itself is created unconditionally.
    assert (meta_dir / "gui").is_dir()
    assert not (meta_dir / "gui" / "default.default.desktop").exists()
    assert not (meta_dir / "gui" / "icon.png").exists()


def test_gen_extra_assets_with_project_hooks(
    default_project, fake_services, setup_project
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    project_hooks_dir = package_service._get_assets_dir() / "hooks"
    project_hooks_dir.mkdir(parents=True)
    (project_hooks_dir / "configure").write_text("configure_hook")
    (project_hooks_dir / "install").write_text("install_hook")

    assert sorted(package_service._gen_extra_assets()) == sorted(
        [
            (
                project_hooks_dir / "configure",
                fake_services.lifecycle.prime_dir / "meta/hooks/configure",
            ),
            (
                project_hooks_dir / "install",
                fake_services.lifecycle.prime_dir / "meta/hooks/install",
            ),
        ]
    )


def test_gen_extra_assets_with_built_hooks(
    default_project, fake_services, setup_project, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    built_hooks_dir = tmp_path / "prime" / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    (built_hooks_dir / "configure").write_text("configure_hook")
    (built_hooks_dir / "install").write_text("install_hook")

    assert package_service._gen_extra_assets() == []


def test_gen_extra_assets_project_hooks_override_built_hooks(
    default_project, fake_services, setup_project, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    project_hooks_dir = package_service._get_assets_dir() / "hooks"
    project_hooks_dir.mkdir(parents=True)
    built_hooks_dir = tmp_path / "prime" / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    (built_hooks_dir / "configure").write_text("built_configure_hook")
    (project_hooks_dir / "configure").write_text("project_configure_hook")

    assert package_service._gen_extra_assets() == [
        (
            project_hooks_dir / "configure",
            fake_services.lifecycle.prime_dir / "meta/hooks/configure",
        ),
    ]


def test_gen_extra_assets_with_manifest_project_file(
    monkeypatch, default_project, fake_services, setup_project
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))

    assert (
        package_service._services.get("project").resolve_project_file_path(),
        fake_services.lifecycle.prime_dir / "snap" / "snapcraft.yaml",
    ) in package_service._gen_extra_assets()


def test_gen_extra_assets_with_project_gui(
    default_project, fake_services, setup_project
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    project_gui_dir = package_service._get_assets_dir() / "gui"
    project_gui_dir.mkdir(parents=True)
    (project_gui_dir / "default.default.desktop").write_text("desktop_file")
    (project_gui_dir / "icon.png").write_text("package_png_icon")

    assert sorted(package_service._gen_extra_assets()) == sorted(
        [
            (
                project_gui_dir / "default.default.desktop",
                fake_services.lifecycle.prime_dir / "meta/gui/default.default.desktop",
            ),
            (
                project_gui_dir / "icon.png",
                fake_services.lifecycle.prime_dir / "meta/gui/icon.png",
            ),
        ]
    )


def test_materialize_extra_assets_generated_desktop_and_icon(
    default_project, fake_services, setup_project, tmp_path
):
    project = {
        **default_project.marshal(),
        "apps": {"app1": {"command": "bin/app1", "desktop": "test.desktop"}},
        "icon": "usr/share/icons/my-icon.svg",
    }
    setup_project(fake_services, project, write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    prime_dir = fake_services.lifecycle.prime_dir

    (prime_dir / "usr/share/icons").mkdir(parents=True)
    (prime_dir / "usr/share/icons/my-icon.svg").write_text("icon-data")
    (prime_dir / "test.desktop").write_text(
        dedent(
            """\
            [Desktop Entry]
            Name=appstream-desktop
            Exec=appstream
            Type=Application
            Icon=/usr/share/icons/my-icon.svg
            """
        )
    )

    package_service._mediated_icon_asset = get_mediated_icon_asset(
        package_service._project,
        assets_dir=package_service._get_assets_dir(),
        prime_dir=prime_dir,
    )
    package_service._materialize_extra_assets(None)

    assert (prime_dir / "meta/gui/icon.svg").read_text() == "icon-data"
    assert (prime_dir / "meta/gui/app1.desktop").read_text() == dedent(
        """\
        [Desktop Entry]
        Name=appstream-desktop
        Exec=default.app1
        Type=Application
        Icon=${SNAP}/meta/gui/icon.svg

        """
    )


def test_needs_packing_generated_desktop(
    default_project, fake_services, setup_project, mocker
):
    project = {
        **default_project.marshal(),
        "apps": {"app1": {"command": "bin/app1", "desktop": "test.desktop"}},
    }
    setup_project(fake_services, project, write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    prime_dir = fake_services.lifecycle.prime_dir
    mocker.patch.dict(package_service._app.__dict__, {"always_repack": False})
    package_service._project_was_updated = False

    project_gui_dir = package_service._get_assets_dir() / "gui"
    project_gui_dir.mkdir(parents=True)
    (project_gui_dir / "icon.svg").write_text("icon-data")
    source = prime_dir / "test.desktop"
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text(
        dedent(
            """\
            [Desktop Entry]
            Name=appstream-desktop
            Exec=appstream
            Type=Application
            Icon=icon.svg
            """
        )
    )

    package_service._materialize_package_files(None)
    package_service._materialize_extra_assets(None)
    package_service.get_artifacts()[None].touch()

    assert package_service.needs_packing() is False

    source.write_text(
        dedent(
            """\
            [Desktop Entry]
            Name=appstream-desktop
            Exec=appstream --new
            Type=Application
            Icon=icon.svg
            """
        )
    )
    materialized = prime_dir / "meta/gui/app1.desktop"
    os.utime(source, (materialized.stat().st_mtime + 10, materialized.stat().st_mtime + 10))

    assert package_service.needs_packing() is True


def test_needs_packing_generated_icon(
    default_project, fake_services, setup_project, mocker
):
    project = {
        **default_project.marshal(),
        "icon": "usr/share/icons/my-icon.svg",
    }
    setup_project(fake_services, project, write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    prime_dir = fake_services.lifecycle.prime_dir
    mocker.patch.dict(package_service._app.__dict__, {"always_repack": False})
    package_service._project_was_updated = False

    source = prime_dir / "usr/share/icons/my-icon.svg"
    source.parent.mkdir(parents=True)
    source.write_text("icon-data")

    package_service._mediated_icon_asset = get_mediated_icon_asset(
        package_service._project,
        assets_dir=package_service._get_assets_dir(),
        prime_dir=prime_dir,
    )
    package_service._materialize_package_files(None)
    package_service._materialize_extra_assets(None)
    package_service.get_artifacts()[None].touch()

    assert package_service.needs_packing() is False

    source.write_text("modified-icon-data")
    destination = prime_dir / "meta/gui/icon.svg"
    os.utime(source, (destination.stat().st_mtime + 10, destination.stat().st_mtime + 10))

    assert package_service.needs_packing() is True


def test_write_asset_preserves_hook_executable(
    default_project, fake_services, setup_project
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    project_hooks_dir = package_service._get_assets_dir() / "hooks"
    project_hooks_dir.mkdir(parents=True)
    source = project_hooks_dir / "configure"
    source.write_text("configure_hook")
    source.chmod(0o644)
    destination = fake_services.lifecycle.prime_dir / "snap" / "hooks" / "configure"

    package_service._write_asset(source, destination)

    assert destination.read_text() == "configure_hook"
    assert oct(destination.stat().st_mode)[-3:] == "755"
    assert oct(source.stat().st_mode)[-3:] == "644"


def test_materialize_extra_assets_project_hooks_override_built_hooks(
    default_project, fake_services, setup_project
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    prime_dir = fake_services.lifecycle.prime_dir

    built_hooks_dir = prime_dir / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    (built_hooks_dir / "configure").write_text("built_configure_hook")

    project_hooks_dir = package_service._get_assets_dir() / "hooks"
    project_hooks_dir.mkdir(parents=True)
    source = project_hooks_dir / "configure"
    source.write_text("project_configure_hook")
    source.chmod(0o644)

    package_service._materialize_extra_assets(None)

    # Built hook remains untouched in snap/hooks
    assert (
        prime_dir / "snap" / "hooks" / "configure"
    ).read_text() == "built_configure_hook"
    # Project hook in meta/hooks overrides the built hook provision
    destination = prime_dir / "meta" / "hooks" / "configure"
    assert destination.read_text() == "project_configure_hook"
    assert oct(destination.stat().st_mode)[-3:] == "755"


def test_materialize_extra_assets_provisions_meta_hooks(
    default_project, fake_services, setup_project
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    prime_dir = fake_services.lifecycle.prime_dir

    # Code-generated hook from part lifecycle
    built_hooks_dir = prime_dir / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    (built_hooks_dir / "configure").write_text("built_hook")

    package_service._materialize_extra_assets(None)

    # Built hook remains in snap/hooks
    assert (prime_dir / "snap" / "hooks" / "configure").read_text() == "built_hook"
    # Built hook is provisioned directly into meta/hooks
    provisioned_hook = prime_dir / "meta" / "hooks" / "configure"
    assert provisioned_hook.read_text() == "built_hook"
    assert oct(provisioned_hook.stat().st_mode)[-3:] == "755"


def test_needs_packing_project_hooks(
    default_project, fake_services, setup_project, new_dir, mocker
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    # Force the asset checks to be the deciding factor in needs_packing.
    mocker.patch.dict(package_service._app.__dict__, {"always_repack": False})
    package_service._project_was_updated = False

    project_hooks_dir = package_service._get_assets_dir() / "hooks"
    project_hooks_dir.mkdir(parents=True)
    source = project_hooks_dir / "configure"
    source.write_text("configure_hook")

    package_service._materialize_package_files(None)
    package_service._materialize_extra_assets(None)
    package_service.get_artifacts()[None].touch()

    assert package_service.needs_packing() is False

    # Modifying the project hook makes the primed hook stale.
    source.write_text("modified_hook")
    # Bump the source mtime past the primed copy to account for coarse
    # filesystem timestamp granularity.
    for src, dest in package_service._gen_extra_assets(None):
        assert isinstance(src, Path)
        os.utime(src, (dest.stat().st_mtime + 10, dest.stat().st_mtime + 10))
    assert package_service.needs_packing() is True


def test_needs_packing_manifest_project_file(
    monkeypatch, default_project, fake_services, setup_project, mocker
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    prime_dir = fake_services.lifecycle.prime_dir
    mocker.patch.dict(package_service._app.__dict__, {"always_repack": False})
    package_service._project_was_updated = False

    package_service._materialize_package_files(None)
    package_service._materialize_extra_assets(None)
    package_service.get_artifacts()[None].touch()

    assert package_service.needs_packing() is False

    source = package_service._services.get("project").resolve_project_file_path()
    destination = prime_dir / "snap" / source.name
    source.write_text(source.read_text() + "\n# changed\n", encoding="utf-8")
    os.utime(source, (destination.stat().st_mtime + 10, destination.stat().st_mtime + 10))

    assert package_service.needs_packing() is True


def test_needs_packing_project_gui(
    default_project, fake_services, setup_project, new_dir, mocker
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast(Package, fake_services.get("package"))
    # Force the asset checks to be the deciding factor in needs_packing.
    mocker.patch.dict(package_service._app.__dict__, {"always_repack": False})
    package_service._project_was_updated = False

    project_gui_dir = package_service._get_assets_dir() / "gui"
    project_gui_dir.mkdir(parents=True)
    source = project_gui_dir / "icon.png"
    source.write_text("icon_data")

    package_service._materialize_package_files(None)
    package_service._materialize_extra_assets(None)
    package_service.get_artifacts()[None].touch()

    assert package_service.needs_packing() is False

    # Modifying the project icon makes the primed icon stale.
    source.write_text("modified_icon_data")
    # Bump the source mtime past the primed copy to account for coarse
    # filesystem timestamp granularity.
    for src, dest in package_service._gen_extra_assets(None):
        assert isinstance(src, Path)
        os.utime(src, (dest.stat().st_mtime + 10, dest.stat().st_mtime + 10))
    assert package_service.needs_packing() is True


def test_update_project_parse_info(
    default_project, fake_services, setup_project, in_project_path, tmp_path, mocker
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    project_service = fake_services.get("project")
    lifecycle = fake_services.lifecycle
    project_info = lifecycle.project_info
    project_info.execution_finished = True

    fake_metadata = ExtractedMetadata()
    mocked_extract = mocker.patch.object(
        extract_metadata, "extract_lifecycle_metadata", return_value=[fake_metadata]
    )
    mocked_update = mocker.patch.object(
        update_metadata, "update_from_extracted_metadata"
    )
    mocker.patch.object(
        project_service,
        "get_parse_info",
        return_value={"my-part": ["file.metadata.xml"]},
    )

    parse_info = {"my-part": ["file.metadata.xml"]}

    package_service.update_project()

    mocked_extract.assert_called_once_with(
        default_project.adopt_info, parse_info, tmp_path, partitions=None
    )
    mocked_update.assert_called_once_with(
        fake_services.get("project").get(),
        metadata_list=[fake_metadata],
        assets_dir=in_project_path / "snap",
        prime_dir=tmp_path / "prime",
    )


def test_extra_project_updates_makes_targets_core26(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    mocker: MockerFixture,
) -> None:
    setup_project(fake_services, snapcraft_yaml(base="core26"))
    package_service = fake_services.get("package")
    mock_precreate_layout = mocker.patch.object(
        package_service, "_precreate_layout_targets"
    )
    mocker.patch.object(package_service, "_precreate_plug_targets")

    package_service.update_project()

    mock_precreate_layout.assert_called_once()


@pytest.mark.parametrize(
    ("layouts", "expected_files"),
    [
        pytest.param({}, {"dirs": [], "files": []}, id="none"),
        pytest.param(
            {
                "/opt/foo": {"bind": "$SNAP/foo"},
            },
            {"dirs": [Path("foo")], "files": []},
            id="dir",
        ),
        pytest.param(
            {"/usr/lib/foo": {"bind": "$SNAP/foo/gpu-2404"}},
            {"dirs": [Path("foo"), Path("foo", "gpu-2404")], "files": []},
            id="dir-recursive",
        ),
        pytest.param(
            {"/usr/lib/foo/ids": {"bind-file": "$SNAP/foo/gpu-2404.ids"}},
            {
                "dirs": [Path("foo")],
                "files": [Path("foo", "gpu-2404.ids")],
            },
            id="file",
        ),
        pytest.param(
            {
                "/var/lib/bar": {"bind": "$SNAP_DATA/bar"},
            },
            {
                "dirs": [],
                "files": [],
            },
            id="no-op",
        ),
        pytest.param(
            {
                "$SNAP/baz": {"type": "tmpfs"},
            },
            {
                "dirs": [Path("baz")],
                "files": [],
            },
            id="path-in-key",
        ),
        pytest.param(
            {
                "/opt/foo": {"bind": "$SNAP/foo"},
                "/usr/lib/foo": {"bind": "$SNAP/foo/gpu-2404"},
                "/usr/lib/foo/ids": {"bind-file": "$SNAP/foo/gpu-2404.ids"},
                "/var/lib/bar": {"bind": "$SNAP_DATA/bar"},
                "$SNAP/baz": {"type": "tmpfs"},
            },
            {
                "dirs": [Path("foo"), Path("foo", "gpu-2404"), Path("baz")],
                "files": [Path("foo", "gpu-2404.ids")],
            },
            id="all",
        ),
    ],
)
def test_precreate_layout_targets(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    layouts: dict[str, Any],
    expected_files: dict[str, list[Path]],
    tmp_path: Path,
) -> None:
    project = snapcraft_yaml(layout=layouts, base="core26")
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))

    package_service._precreate_layout_targets()

    prime_dir = fake_services.lifecycle.prime_dir
    prime_dir.mkdir(0o755, exist_ok=True)

    primed_files = [
        file.relative_to(tmp_path / "prime") for file in prime_dir.rglob("*")
    ]

    all_expected_files = [*expected_files["dirs"], *expected_files["files"]]
    assert sorted(primed_files) == sorted(all_expected_files)

    for path in expected_files["dirs"]:
        file = tmp_path / "prime" / path
        assert file.stat().st_mode & 0o0755 == 0o0755
        assert file.is_dir()

    for path in expected_files["files"]:
        file = tmp_path / "prime" / path
        assert file.stat().st_mode & 0o0644 == 0o0644
        assert file.is_file()


def test_precreate_layout_targets_messages(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    emitter: RecordingEmitter,
) -> None:
    layout = {
        "/opt/foo": {"bind": "$SNAP/foo"},
        "/usr/lib/foo/ids": {"bind-file": "$SNAP/foo/gpu-2404.ids"},
    }
    project = snapcraft_yaml(layout=layout, base="core26")
    setup_project(fake_services, project)
    package_service = cast("Package", fake_services.get("package"))

    package_service._precreate_layout_targets()

    emitter.assert_debug("Pre-creating layout targets inside of snap")
    emitter.assert_debug(
        "Layout target directory '$SNAP/foo' maps to 'foo' inside of the snap"
    )
    emitter.assert_debug("Creating 'foo' in the prime directory")
    emitter.assert_debug(
        "Layout target file '$SNAP/foo/gpu-2404.ids' maps to 'foo/gpu-2404.ids' inside of the snap"
    )
    emitter.assert_debug("Creating 'foo/gpu-2404.ids' in the prime directory")


@pytest.mark.parametrize(
    ("plugs", "expected_files"),
    [
        pytest.param({}, [], id="none"),
        pytest.param(
            {
                "usb": {
                    "interface": "content",
                    "target": "$SNAP/usb",
                },
            },
            [Path("usb")],
            id="one",
        ),
        pytest.param(
            {
                "usb": {
                    "interface": "content",
                    "target": "$SNAP/usb",
                },
                "serial": {
                    "interface": "content",
                    "target": "$SNAP/serial",
                },
            },
            [Path("usb"), Path("serial")],
            id="two",
        ),
        pytest.param(
            {
                "hdmi": {
                    "interface": "content",
                    "target": "$SNAP/hdmi/2.1",
                },
            },
            [Path("hdmi"), Path("hdmi", "2.1")],
            id="recursive",
        ),
        pytest.param(
            {
                "content": {
                    "target": "$SNAP/aux",
                },
            },
            [Path("aux")],
            id="in-name",
        ),
        pytest.param(
            {
                "flash-drive": {
                    "interface": "content",
                    "target": "$SNAP_DATA/flash-drive",
                },
            },
            [],
            id="no-op",
        ),
        pytest.param({"bluetooth": {"private": True}}, [], id="non-content"),
        pytest.param(
            {
                "usb": {
                    "interface": "content",
                    "target": "$SNAP/usb",
                },
                "serial": {
                    "interface": "content",
                    "target": "$SNAP/serial",
                },
                "hdmi": {
                    "interface": "content",
                    "target": "$SNAP/hdmi/2.1",
                },
                "flash-drive": {
                    "interface": "content",
                    "target": "$SNAP_DATA/flash-drive",
                },
                "content": {
                    "target": "$SNAP/aux",
                },
                "bluetooth": {"private": True},
            },
            [
                Path("usb"),
                Path("serial"),
                Path("hdmi"),
                Path("hdmi", "2.1"),
                Path("aux"),
            ],
            id="all",
        ),
    ],
)
def test_precreate_plug_targets(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    plugs: dict[str, Any],
    expected_files: list[Path],
    tmp_path: Path,
) -> None:
    project = snapcraft_yaml(plugs=plugs, base="core26")
    setup_project(fake_services, project)
    package_service = cast(Package, fake_services.get("package"))

    package_service._precreate_plug_targets()

    prime_dir = fake_services.lifecycle.prime_dir
    prime_dir.mkdir(0o755, exist_ok=True)

    primed_files = [
        file.relative_to(tmp_path / "prime") for file in prime_dir.rglob("*")
    ]

    assert sorted(primed_files) == sorted(expected_files)

    for path in expected_files:
        file = tmp_path / "prime" / path
        assert file.stat().st_mode & 0o0755 == 0o0755
        assert file.is_dir()


def test_precreate_plug_targets_messages(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    emitter: RecordingEmitter,
) -> None:
    plugs = {
        "usb": {
            "interface": "content",
            "target": "$SNAP/usb",
        }
    }
    project = snapcraft_yaml(plugs=plugs, base="core26")
    setup_project(fake_services, project)
    package_service = cast("Package", fake_services.get("package"))

    package_service._precreate_plug_targets()

    emitter.assert_debug("Pre-creating plug targets inside of snap")
    emitter.assert_debug(
        "Plug target directory '$SNAP/usb' maps to 'usb' inside of the snap"
    )
    emitter.assert_debug("Creating 'usb' in the prime directory")


@pytest.mark.parametrize(
    ("in_path", "expected"),
    [
        pytest.param("$SNAP/foo", Path("foo"), id="simple"),
        pytest.param("/foo", Path("foo"), id="absolute"),
        pytest.param("foo", Path("foo"), id="relative"),
        pytest.param("$SNAP/foo/bar", Path("foo/bar"), id="nested"),
        pytest.param("$SNAP", None, id="no-op-base"),
        pytest.param("/", None, id="no-op-absolute"),
        pytest.param("$SNAP/", None, id="no-op-suffixed"),
        pytest.param("///foo", Path("foo"), id="absolute-recursive"),
        pytest.param("///", None, id="no-op-recursive"),
        pytest.param("$SNAP_DATA/foo", None, id="no-op-unwanted"),
    ],
)
def test_maybe_get_target_in_snap(
    fake_services: ServiceFactory, in_path: str, expected: Path | None
) -> None:
    package_service = cast("Package", fake_services.get("package"))
    assert package_service._maybe_get_target_in_snap(in_path) == expected


@pytest.mark.parametrize(
    ("path", "expectation"),
    [
        pytest.param(Path("foo"), nullcontext(), id="simple"),
        pytest.param(Path("foo", "bar"), nullcontext(), id="parts"),
        pytest.param(
            Path("..", "..", ".."),
            pytest.raises(SnapcraftPrecreationEscapesPrimeError),
            id="far-back",
        ),
        pytest.param(
            Path("..", "stage"),
            pytest.raises(SnapcraftPrecreationEscapesPrimeError),
            id="to-stage",
        ),
    ],
)
def test_concat_with_prime_dir(
    fake_services: ServiceFactory,
    default_project: models.Project,
    setup_project: Callable[..., Any],
    path: Path,
    expectation: AbstractContextManager,
) -> None:
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = cast("Package", fake_services.get("package"))

    with expectation:
        package_service._concat_with_prime_dir(path)


@pytest.mark.parametrize(
    ("src", "layout", "expected"),
    [
        pytest.param(
            "/opt/foo", {"bind": "$SNAP/foo"}, ("$SNAP/foo", "bind"), id="bind"
        ),
        pytest.param(
            "/opt/foo/conf",
            {"bind-file": "$SNAP/foo/conf"},
            ("$SNAP/foo/conf", "bind-file"),
            id="bind-file",
        ),
        pytest.param(
            "/opt/foo/elsewhere",
            {"symlink": "$SNAP/foo/elsewhere"},
            None,
            id="skip-symlink",
        ),
        pytest.param(
            "$SNAP/scratch", {"type": "tmpfs"}, ("$SNAP/scratch", "tmpfs"), id="tmpfs"
        ),
        pytest.param("/opt/foo", {"type": "secret-other-thing"}, None, id="bad"),
    ],
)
def test_parse_layout_target(
    src: str, layout: dict[Any, str], expected: tuple[str, Any]
) -> None:
    assert Package._parse_layout_target(src, layout) == expected
