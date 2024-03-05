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

"""Tests for the Snapcraft Lifecycle service."""
import json
from unittest import mock

import pytest

from snapcraft import __version__, models, os_release, utils


def test_lifecycle_installs_base(lifecycle_service, mocker):
    install_snaps = mocker.patch("craft_parts.packages.snaps.install_snaps")

    lifecycle_service.setup()
    lifecycle_service.run("pull")

    info = lifecycle_service.project_info
    assert info.base == "core24"
    install_snaps.assert_called_once_with(
        {"core24"},
    )


@pytest.mark.parametrize(
    ("parts", "stage_packages"),
    [
        ({}, []),
        (
            {
                "simplified": {
                    "plugin": "nil",
                    "stage-packages": ["hello"],
                    "stage": [],
                    "prime": [],
                    "build-packages": [],
                },
                "traditional": {
                    "plugin": "nil",
                    "stage-packages": ["hello-traditional"],
                    "stage": [],
                    "prime": [],
                    "build-packages": [],
                },
            },
            ["hello", "hello-traditional"],
        ),
    ],
)
@pytest.mark.parametrize("image_info", [{}, {"custom_image": True}])
def test_generate_manifest(
    monkeypatch, lifecycle_service, default_project, image_info, parts, stage_packages
):
    monkeypatch.setenv("SNAPCRAFT_IMAGE_INFO", json.dumps(image_info))
    osrel = os_release.OsRelease()
    default_project.parts = parts
    lifecycle_service.setup()
    lifecycle_service.get_pull_assets = mock.Mock(return_value={})
    lifecycle_service.get_primed_stage_packages = mock.Mock(return_value=stage_packages)

    manifest = lifecycle_service.generate_manifest()

    assert manifest == models.Manifest(
        snapcraft_version=__version__,
        snapcraft_started_at=lifecycle_service._start_time.isoformat("T") + "Z",
        snapcraft_os_release_id=osrel.name().lower(),
        snapcraft_os_release_version_id=osrel.version_id().lower(),
        name=default_project.name,
        version=default_project.version,
        summary=default_project.summary,
        description=default_project.description,
        base=default_project.base,
        grade=default_project.grade,
        confinement=default_project.confinement,
        parts=parts,
        architectures=[utils.get_host_architecture()],
        image_info=image_info,
        build_packages=default_project.build_packages or [],
        build_snaps=default_project.build_snaps or [],
        primed_stage_packages=stage_packages,
        apps=None,
    )

    assert lifecycle_service.get_pull_assets.mock_calls == [
        mock.call(part_name=part_name) for part_name in parts
    ]
    assert lifecycle_service.get_primed_stage_packages.mock_calls == [
        mock.call(part_name=part_name) for part_name in parts
    ]
