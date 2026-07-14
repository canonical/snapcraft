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
from collections.abc import Callable
from contextlib import AbstractContextManager, nullcontext
from pathlib import Path
from textwrap import dedent
from types import SimpleNamespace
from typing import Any, cast

import pytest
import requests
import yaml
from craft_application import ServiceFactory
from craft_cli.pytest_plugin import RecordingEmitter
from pytest_mock import MockerFixture

from snapcraft import __version__, const, errors, linters, meta, models, pack
from snapcraft.errors import SnapcraftPrecreationEscapesPrimeError
from snapcraft.meta import ExtractedMetadata
from snapcraft.parts import extract_metadata, update_metadata
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


def test_get_snap_yaml(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service._get_snap_yaml() == dedent("""\
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


def test_get_artifacts(default_project, fake_services, setup_project, tmp_path):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    package_service.set_output_dir(tmp_path / "test-output.snap")

    assert package_service.get_artifacts() == {None: tmp_path / "test-output.snap"}


def test_get_artifacts_defaults_to_cwd(
    default_project, fake_services, setup_project, monkeypatch, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    monkeypatch.chdir(tmp_path)

    assert package_service.get_artifacts() == {
        None: tmp_path / "default_1.0_amd64.snap"
    }


def test_pack_artifact_snap(
    default_project, fake_services, setup_project, mocker, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")

    package_service._pack(name=None, path=tmp_path / "test-output.snap")

    mock_pack_snap.assert_called_once_with(
        tmp_path / "prime",
        name="default",
        version="1.0",
        compression="xz",
        output=str(tmp_path / "test-output.snap"),
        target="amd64",
    )


def test_get_manifest_yaml_disabled(
    monkeypatch, default_project, fake_services, setup_project
):
    monkeypatch.delenv("SNAPCRAFT_BUILD_INFO", raising=False)
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service._get_manifest_yaml() is None


def test_get_manifest_yaml_enabled(
    monkeypatch, default_project, fake_services, setup_project
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")

    manifest = yaml.safe_load(cast("str", package_service._get_manifest_yaml()))

    assert manifest["name"] == "default"
    assert manifest["version"] == "1.0"
    assert manifest["architectures"] == ["amd64"]
    assert manifest["grade"] == "devel"


def test_supports_conditional_repack(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service.supports_conditional_repack is True


def test_project_file_copy_disabled(
    monkeypatch, default_project, fake_services, setup_project
):
    monkeypatch.delenv("SNAPCRAFT_BUILD_INFO", raising=False)
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    assert package_service._project_file_copy_enabled() is False


def test_project_file_copy_enabled(
    monkeypatch, default_project, fake_services, setup_project
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")

    assert package_service._project_file_copy_enabled() is True


def test_gen_extra_assets_includes_project_file(
    monkeypatch, default_project, fake_services, setup_project, tmp_path
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")

    project_file = fake_services.get("project").resolve_project_file_path()

    assert package_service._gen_extra_assets() == [
        (project_file, tmp_path / "prime" / "snap" / project_file.name)
    ]


def test_gen_extra_assets_removes_project_file_when_disabled(
    monkeypatch, default_project, fake_services, setup_project, tmp_path
):
    monkeypatch.delenv("SNAPCRAFT_BUILD_INFO", raising=False)
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    project_file = fake_services.get("project").resolve_project_file_path()

    assert package_service._gen_extra_assets() == [
        (None, tmp_path / "prime" / "snap" / project_file.name)
    ]


@pytest.fixture(params=["snap", "build-aux/snap"])
def project_hooks_dir(in_project_path, request):
    hooks_dir = in_project_path / request.param / "hooks"
    hooks_dir.mkdir(parents=True)
    yield hooks_dir


def test_gen_extra_assets_includes_default_hook_assets(
    default_project, fake_services, setup_project, project_hooks_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    built_hooks_dir = tmp_path / "prime" / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    (built_hooks_dir / "configure").write_text("built_configure", encoding="utf-8")
    (project_hooks_dir / "install").write_text("project_install", encoding="utf-8")

    extra_assets = package_service._gen_extra_assets()

    assert (
        built_hooks_dir / "configure",
        tmp_path / "prime" / "meta" / "hooks" / "configure",
    ) in extra_assets
    assert (
        project_hooks_dir / "install",
        tmp_path / "prime" / "meta" / "hooks" / "install",
    ) in extra_assets


def test_gen_extra_assets_project_hooks_override_built_hooks(
    default_project, fake_services, setup_project, project_hooks_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    built_hooks_dir = tmp_path / "prime" / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    built_hook = built_hooks_dir / "configure"
    project_hook = project_hooks_dir / "configure"
    built_hook.write_text("built_configure", encoding="utf-8")
    project_hook.write_text("project_configure", encoding="utf-8")

    extra_assets = package_service._get_hook_assets()

    assert extra_assets == [
        (project_hook, tmp_path / "prime" / "meta" / "hooks" / "configure"),
    ]


def test_write_metadata_project_hook_overrides_newer_built_hook(
    default_project, fake_services, setup_project, project_hooks_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    built_hooks_dir = tmp_path / "prime" / "snap" / "hooks"
    built_hooks_dir.mkdir(parents=True)
    built_hook = built_hooks_dir / "configure"
    project_hook = project_hooks_dir / "configure"
    project_hook.write_text("project_configure", encoding="utf-8")
    built_hook.write_text("built_configure", encoding="utf-8")

    package_service.write_metadata(tmp_path / "prime")

    assert (tmp_path / "prime" / "meta" / "hooks" / "configure").read_text() == (
        "project_configure"
    )


def test_get_gui_assets(
    default_project, fake_services, setup_project, in_project_path, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    project_gui_dir = in_project_path / "snap" / "gui"
    project_gui_dir.mkdir(parents=True)
    desktop = project_gui_dir / "default.default.desktop"
    icon = project_gui_dir / "icon.png"
    desktop.write_text("desktop_file", encoding="utf-8")
    icon.write_text("package_png_icon", encoding="utf-8")

    assert package_service._get_gui_assets() == [
        (desktop, tmp_path / "prime" / "meta" / "gui" / "default.default.desktop"),
        (icon, tmp_path / "prime" / "meta" / "gui" / "icon.png"),
    ]


def test_get_desktop_assets(default_project, fake_services, setup_project, tmp_path):
    project_data = default_project.marshal()
    project_data["apps"] = {"app1": {"command": "bin/test", "desktop": "test.desktop"}}
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    prime_dir = tmp_path / "prime"
    (prime_dir / "bin").mkdir(parents=True)
    (prime_dir / "bin" / "test").write_text("#!/bin/true\n", encoding="utf-8")
    (prime_dir / "bin" / "test").chmod(0o755)
    (prime_dir / "test.desktop").write_text(
        dedent("""\
            [Desktop Entry]
            Name=test
            Exec=test
            Type=Application
            Icon=/usr/share/icons/test.png
            """),
        encoding="utf-8",
    )
    (prime_dir / "usr/share/icons").mkdir(parents=True)
    (prime_dir / "usr/share/icons" / "test.png").write_text("icon", encoding="utf-8")

    assert package_service._get_desktop_assets() == [
        (
            dedent("""\
                [Desktop Entry]
                Name=test
                Exec=default.app1
                Type=Application
                Icon=${SNAP}/usr/share/icons/test.png

                """),
            tmp_path / "prime" / "meta" / "gui" / "app1.desktop",
        )
    ]


def test_get_icon_assets_remote(
    monkeypatch, default_project, fake_services, setup_project, mocker, tmp_path
):
    project_data = default_project.marshal()
    project_data["icon"] = "https://example.com/icon.png"
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    mock_response = mocker.Mock()
    mock_response.content = b"png-data"
    mock_response.raise_for_status.return_value = None
    mocker.patch("snapcraft.services.package.requests.get", return_value=mock_response)

    assert package_service._get_icon_assets() == [
        (b"png-data", tmp_path / "prime" / "meta" / "gui" / "icon.png")
    ]


def test_get_icon_assets_remote_http_error(
    default_project, fake_services, setup_project, mocker
):
    project_data = default_project.marshal()
    project_data["icon"] = "https://example.com/icon.png"
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    mock_response = mocker.Mock()
    mock_response.raise_for_status.side_effect = requests.HTTPError("404 Client Error")
    mocker.patch("snapcraft.services.package.requests.get", return_value=mock_response)

    with pytest.raises(errors.SnapcraftError, match="Failed to fetch icon") as raised:
        package_service._get_icon_assets()

    assert raised.value.details == "404 Client Error"


def test_get_icon_assets_remote_request_error(
    default_project, fake_services, setup_project, mocker
):
    project_data = default_project.marshal()
    project_data["icon"] = "https://example.com/icon.png"
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    mocker.patch(
        "snapcraft.services.package.requests.get",
        side_effect=requests.RequestException("temporary failure in name resolution"),
    )

    with pytest.raises(errors.SnapcraftError, match="Failed to fetch icon") as raised:
        package_service._get_icon_assets()

    assert raised.value.details == "temporary failure in name resolution"


def test_get_system_metadata_assets_gadget(
    default_project, fake_services, setup_project, in_project_path, tmp_path
):
    project_data = default_project.marshal()
    project_data["type"] = "gadget"
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    gadget_yaml = in_project_path / "gadget.yaml"
    gadget_yaml.write_text("volumes: {}\n", encoding="utf-8")

    assert package_service._get_system_metadata_assets() == [
        (gadget_yaml, tmp_path / "prime" / "meta" / "gadget.yaml")
    ]


def test_get_system_metadata_assets_kernel_missing(
    default_project, fake_services, setup_project, mocker, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    mocker.patch.object(
        type(package_service),
        "_project",
        new_callable=mocker.PropertyMock,
        return_value=SimpleNamespace(type=const.ProjectType.KERNEL),
    )

    assert package_service._get_system_metadata_assets() == [
        (None, tmp_path / "prime" / "meta" / "kernel.yaml")
    ]


def test_manifest_changed_ignores_started_at(
    monkeypatch, default_project, fake_services, setup_project, tmp_path
):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "1")
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    prime_dir = tmp_path / "prime"
    snap_dir = prime_dir / "snap"
    snap_dir.mkdir(parents=True)

    manifest = yaml.safe_load(cast("str", package_service._get_manifest_yaml()))
    manifest["snapcraft-started-at"] = "2001-02-03T04:05:06Z"
    (snap_dir / "manifest.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")

    assert package_service._manifest_changed(None) is False


def test_needs_packing_when_write_metadata_changes_prime(
    default_project, fake_services, setup_project, tmp_path
):
    project_data = default_project.marshal()
    project_data["apps"] = {"app1": {"command": "bin/test", "desktop": "test.desktop"}}
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    artifact_path = tmp_path / "test-output.snap"
    artifact_path.write_text("artifact", encoding="utf-8")

    prime_dir = tmp_path / "prime"
    (prime_dir / "bin").mkdir(parents=True)
    (prime_dir / "bin" / "test").write_text("#!/bin/true\n", encoding="utf-8")
    (prime_dir / "bin" / "test").chmod(0o755)
    (prime_dir / "test.desktop").write_text(
        dedent("""\
            [Desktop Entry]
            Name=test
            Exec=test
            Type=Application
            """),
        encoding="utf-8",
    )

    package_service.write_metadata(prime_dir)

    assert package_service.needs_packing(None) is True


def test_write_metadata_second_run_is_stable(
    default_project, fake_services, setup_project, tmp_path
):
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")
    prime_dir = tmp_path / "prime"

    package_service.write_metadata(prime_dir)
    snap_yaml = prime_dir / "meta" / "snap.yaml"
    first_mtime = snap_yaml.stat().st_mtime_ns

    package_service._project_was_updated = False
    package_service.write_metadata(prime_dir)

    assert package_service._project_was_updated is False
    assert snap_yaml.stat().st_mtime_ns == first_mtime


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


def test_write_metadata_removes_manifest_when_disabled(
    monkeypatch, default_project, fake_services, setup_project, tmp_path
):
    monkeypatch.delenv("SNAPCRAFT_BUILD_INFO", raising=False)
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    manifest_path = tmp_path / "prime" / "snap" / "manifest.yaml"
    manifest_path.parent.mkdir(parents=True)
    manifest_path.write_text("stale manifest", encoding="utf-8")

    package_service.write_metadata(tmp_path / "prime")

    assert not manifest_path.exists()


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
    project_file = fake_services.get("project").resolve_project_file_path()
    assert (
        prime_dir / "snap" / project_file.name
    ).read_text() == project_file.read_text()


def test_write_metadata_removes_project_file_when_disabled(
    monkeypatch, default_project, fake_services, setup_project, tmp_path
):
    monkeypatch.delenv("SNAPCRAFT_BUILD_INFO", raising=False)
    setup_project(fake_services, default_project.marshal())
    package_service = fake_services.get("package")

    project_file = fake_services.get("project").resolve_project_file_path()
    copied_project_file = tmp_path / "prime" / "snap" / project_file.name
    copied_project_file.parent.mkdir(parents=True)
    copied_project_file.write_text("stale project file", encoding="utf-8")

    package_service.write_metadata(tmp_path / "prime")

    assert not copied_project_file.exists()


def test_write_metadata_with_project_hooks(
    default_project, fake_services, setup_project, project_hooks_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    # Create some hooks
    (project_hooks_dir / "configure").write_text("configure_hook")
    (project_hooks_dir / "install").write_text("install_hook")
    (project_hooks_dir / "configure").chmod(0o755)
    (project_hooks_dir / "install").chmod(0o755)

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

    assert (meta_dir / "hooks").exists()
    # Ensure the hook is the one we provided in the project
    # and not a wrapped hook.
    assert (meta_dir / "hooks" / "configure").exists()
    assert (meta_dir / "hooks" / "configure").read_text() == "configure_hook"
    assert (meta_dir / "hooks" / "configure").stat().st_mode & 0o111
    assert (meta_dir / "hooks" / "install").exists()
    assert (meta_dir / "hooks" / "install").read_text() == "install_hook"
    assert (meta_dir / "hooks" / "install").stat().st_mode & 0o111


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
    (built_hooks_dir / "configure").chmod(0o755)
    (built_hooks_dir / "install").chmod(0o755)

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

    assert (meta_dir / "hooks").exists()
    # Ensure the hook is the one we provided in the project
    # and not a wrapped hook.
    assert (meta_dir / "hooks" / "configure").exists()
    assert (meta_dir / "hooks" / "configure").read_text() == "configure_hook"
    assert (meta_dir / "hooks" / "configure").stat().st_mode & 0o111
    assert (meta_dir / "hooks" / "install").exists()
    assert (meta_dir / "hooks" / "install").read_text() == "install_hook"
    assert (meta_dir / "hooks" / "install").stat().st_mode & 0o111


def test_write_metadata_generates_declared_hook_stub_as_executable(
    default_project, fake_services, setup_project, tmp_path
):
    project_data = default_project.marshal()
    project_data["hooks"] = {"post-refresh": {}}
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")

    prime_dir = tmp_path / "prime"
    package_service.write_metadata(prime_dir)

    hook_path = prime_dir / "meta" / "hooks" / "post-refresh"
    assert hook_path.read_text() == "#!/bin/true\n"
    assert hook_path.stat().st_mode & 0o111


def test_write_metadata_makes_non_executable_hook_asset_executable(
    default_project, fake_services, setup_project, project_hooks_dir, tmp_path
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    project_hook = project_hooks_dir / "configure"
    project_hook.write_text("configure_hook", encoding="utf-8")
    project_hook.chmod(0o644)

    prime_dir = tmp_path / "prime"
    package_service.write_metadata(prime_dir)

    hook_path = prime_dir / "meta" / "hooks" / "configure"
    assert hook_path.read_text() == "configure_hook"
    assert hook_path.stat().st_mode & 0o111


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

    assert (meta_dir / "gui").exists()
    # Ensure the hook is the one we provided in the project
    # and not a wrapped hook.
    assert (meta_dir / "gui" / "default.default.desktop").exists()
    assert (meta_dir / "gui" / "default.default.desktop").read_text() == "desktop_file"
    assert (meta_dir / "gui" / "icon.png").exists()
    assert (meta_dir / "gui" / "icon.png").read_text() == "package_png_icon"


def test_write_metadata_generates_desktop_and_marks_project_updated(
    default_project, fake_services, setup_project, tmp_path
):
    project_data = default_project.marshal()
    project_data["apps"] = {"app1": {"command": "bin/test", "desktop": "test.desktop"}}
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    prime_dir = tmp_path / "prime"
    (prime_dir / "bin").mkdir(parents=True)
    (prime_dir / "bin" / "test").write_text("#!/bin/true\n", encoding="utf-8")
    (prime_dir / "bin" / "test").chmod(0o755)
    (prime_dir / "test.desktop").write_text(
        dedent("""\
            [Desktop Entry]
            Name=test
            Exec=test
            Type=Application
            """),
        encoding="utf-8",
    )

    package_service.write_metadata(prime_dir)

    assert (prime_dir / "meta" / "gui" / "app1.desktop").read_text() == dedent("""\
        [Desktop Entry]
        Name=test
        Exec=default.app1
        Type=Application

        """)
    assert package_service._project_was_updated is True


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


def test_update_project_parse_info_unchanged_does_not_mark_project_updated(
    default_project, fake_services, setup_project, in_project_path, tmp_path, mocker
):
    setup_project(fake_services, default_project.marshal(), write_project=True)
    package_service = fake_services.get("package")
    project_service = fake_services.get("project")
    lifecycle = fake_services.lifecycle
    project_info = lifecycle.project_info
    project_info.execution_finished = True

    fake_metadata = ExtractedMetadata()
    mocker.patch.object(
        extract_metadata, "extract_lifecycle_metadata", return_value=[fake_metadata]
    )
    mocker.patch.object(update_metadata, "update_from_extracted_metadata")
    mocker.patch.object(
        project_service,
        "get_parse_info",
        return_value={"my-part": ["file.metadata.xml"]},
    )

    package_service.update_project()

    assert package_service._project_was_updated is False


def test_update_project_parse_info_changed_marks_project_updated(
    default_project, fake_services, setup_project, in_project_path, tmp_path, mocker
):
    project_data = default_project.marshal()
    project_data["license"] = None
    setup_project(fake_services, project_data, write_project=True)
    package_service = fake_services.get("package")
    project_service = fake_services.get("project")
    lifecycle = fake_services.lifecycle
    project_info = lifecycle.project_info
    project_info.execution_finished = True

    fake_metadata = ExtractedMetadata(license="GPL-3.0")
    mocker.patch.object(
        extract_metadata, "extract_lifecycle_metadata", return_value=[fake_metadata]
    )
    mocker.patch.object(
        project_service,
        "get_parse_info",
        return_value={"my-part": ["file.metadata.xml"]},
    )

    package_service.update_project()

    assert project_service.get().license == "GPL-3.0"
    assert package_service._project_was_updated is True


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
    mock_precreate_plugs = mocker.patch.object(
        package_service, "_precreate_plug_targets"
    )

    package_service.update_project()

    mock_precreate_layout.assert_called_once()
    mock_precreate_plugs.assert_called_once()


def test_extra_project_updates_core26_no_precreate_changes_not_marked_updated(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    mocker: MockerFixture,
) -> None:
    setup_project(fake_services, snapcraft_yaml(base="core26"))
    package_service = fake_services.get("package")
    mocker.patch.object(
        package_service, "_precreate_layout_targets", return_value=False
    )
    mocker.patch.object(package_service, "_precreate_plug_targets", return_value=False)

    package_service.update_project()

    assert package_service._project_was_updated is False


def test_extra_project_updates_core26_precreate_change_marks_updated(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    mocker: MockerFixture,
) -> None:
    setup_project(fake_services, snapcraft_yaml(base="core26"))
    package_service = fake_services.get("package")
    mocker.patch.object(package_service, "_precreate_layout_targets", return_value=True)
    mocker.patch.object(package_service, "_precreate_plug_targets", return_value=False)

    package_service.update_project()

    assert package_service._project_was_updated is True


@pytest.mark.parametrize(
    "base",
    [
        "core22",
        "core24",
    ],
)
def test_extra_project_updates_no_make_targets_legacy(
    snapcraft_yaml: Callable[..., Any],
    setup_project: Callable[..., Any],
    fake_services: ServiceFactory,
    mocker: MockerFixture,
    base: str,
) -> None:
    setup_project(fake_services, snapcraft_yaml(base=base))
    package_service = fake_services.get("package")
    mock_precreate_layout = mocker.patch.object(
        package_service, "_precreate_layout_targets"
    )
    mock_precreate_plugs = mocker.patch.object(
        package_service, "_precreate_plug_targets"
    )

    package_service.update_project()

    mock_precreate_layout.assert_not_called()
    mock_precreate_plugs.assert_not_called()


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
