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
from pathlib import Path
from textwrap import dedent

import pytest
import yaml

from snapcraft import __version__, linters, meta, models, pack
from snapcraft.meta import ExtractedMetadata
from snapcraft.parts import extract_metadata, update_metadata


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
    """
    )

    assert not (prime_dir / "snap" / "manifest.yaml").exists()


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
    """
    )

    assert (meta_dir / "hooks").exists()
    # Ensure the hook is the one we provided in the project
    # and not a wrapped hook.
    assert (meta_dir / "hooks" / "configure").exists()
    assert (meta_dir / "hooks" / "configure").read_text() == "configure_hook"
    assert (meta_dir / "hooks" / "install").exists()
    assert (meta_dir / "hooks" / "install").read_text() == "install_hook"


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
    """
    )

    assert (meta_dir / "hooks").exists()
    # Ensure the hook is the one we provided in the project
    # and not a wrapped hook.
    assert (meta_dir / "hooks" / "configure").exists()
    assert (meta_dir / "hooks" / "configure").read_text() == "configure_hook"
    assert (meta_dir / "hooks" / "install").exists()
    assert (meta_dir / "hooks" / "install").read_text() == "install_hook"


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
    """
    )

    assert (meta_dir / "gui").exists()
    # Ensure the hook is the one we provided in the project
    # and not a wrapped hook.
    assert (meta_dir / "gui" / "default.default.desktop").exists()
    assert (meta_dir / "gui" / "default.default.desktop").read_text() == "desktop_file"
    assert (meta_dir / "gui" / "icon.png").exists()
    assert (meta_dir / "gui" / "icon.png").read_text() == "package_png_icon"


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
        default_project,
        metadata_list=[fake_metadata],
        assets_dir=in_project_path / "snap",
        prime_dir=tmp_path / "prime",
    )
