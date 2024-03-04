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

from pathlib import Path
from textwrap import dedent

import pytest
from craft_application.models import SummaryStr, VersionStr

from snapcraft import linters, meta, pack, services
from snapcraft.application import APP_METADATA


@pytest.mark.usefixtures("default_factory")
def test_pack(package_service, mocker):
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
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
        target_arch="amd64",
    )


def test_pack_target_arch(default_project, default_factory, mocker):
    mock_pack_snap = mocker.patch.object(pack, "pack_snap")
    mocker.patch.object(linters, "run_linters")
    mocker.patch.object(linters, "report")

    package_service = services.Package(
        app=APP_METADATA,
        project=default_project,
        services=default_factory,
        platform="amd64",
        build_for="s390x",
    )

    package_service.pack(prime_dir=Path("prime"), dest=Path())

    assert mock_pack_snap.call_args.kwargs["target_arch"] == "s390x"


def test_metadata(package_service, default_factory, default_build_plan, new_dir):
    default_factory.set_kwargs(
        "lifecycle",
        work_dir=Path("work"),
        cache_dir=new_dir,
        build_plan=default_build_plan,
    )

    assert package_service.metadata == meta.SnapMetadata(
        name="default",
        title=None,
        version=VersionStr("1.0"),
        summary=SummaryStr("default project"),
        description="default project",
        license="MIT",
        type=None,
        architectures=["amd64"],
        base="core24",
        assumes=None,
        epoch=None,
        apps=None,
        confinement="devmode",
        grade="devel",
        environment={
            "LD_LIBRARY_PATH": "${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}",
            "PATH": "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH",
        },
        plugs=None,
        slots=None,
        hooks=None,
        layout=None,
        system_usernames=None,
        provenance=None,
        links=None,
    )


def test_write_metadata(
    package_service,
    default_factory,
    default_build_plan,
    new_dir,
):
    default_factory.set_kwargs(
        "lifecycle",
        work_dir=Path("work"),
        cache_dir=new_dir,
        build_plan=default_build_plan,
    )

    package_service.write_metadata(new_dir)

    assert (new_dir / "meta" / "snap.yaml").read_text() == dedent(
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
