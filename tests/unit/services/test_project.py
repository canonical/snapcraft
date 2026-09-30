# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

"""Tests for the Snapcraft project service."""

import itertools
import pathlib
from typing import Any
from unittest.mock import call

import pytest
import pytest_mock
from craft_application.errors import CraftValidationError
from craft_cli.pytest_plugin import RecordingEmitter

from snapcraft import const
from snapcraft.application import APP_METADATA
from snapcraft.services.project import Project


@pytest.fixture(autouse=True)
def reset_warnings():
    """Reset the one-shot warning flags between tests."""
    Project._ua_service_warning = False
    Project._license_spdx_warning = False
    yield
    Project._ua_service_warning = False
    Project._license_spdx_warning = False


@pytest.mark.parametrize(
    ("raw_project", "expected"),
    [
        pytest.param(
            {
                "base": "core22",
                "architectures": [
                    {"build-on": ["amd64", "arm64"], "build-for": ["all"]}
                ],
            },
            {"all": {"build-on": ["amd64", "arm64"], "build-for": ["all"]}},
        ),
        *(
            pytest.param(
                {"base": "core22", "architectures": [arch.value]},
                {arch.value: {"build-on": [arch.value], "build-for": [arch.value]}},
                id=arch.value,
            )
            for arch in const.SnapArch
        ),
        *(
            pytest.param(
                {"base": "core22", "architectures": [arch1.value, arch2.value]},
                {
                    arch1.value: {
                        "build-on": [arch1.value],
                        "build-for": [arch1.value],
                    },
                    arch2.value: {
                        "build-on": [arch2.value],
                        "build-for": [arch2.value],
                    },
                },
                id=f"{arch1.value}-{arch2.value}",
            )
            for arch1, arch2 in itertools.combinations(const.SnapArch, 2)
        ),
    ],
)
def test_render_legacy_platforms_success(
    mocker: pytest_mock.MockerFixture,
    in_project_path: pathlib.Path,
    raw_project: dict[str, Any],
    expected: dict[str, dict[str, list[str]]],
):
    service = Project(
        app=APP_METADATA,
        services=None,  # ty: ignore[invalid-argument-type] other services not needed
        project_dir=in_project_path,
    )
    mocker.patch.object(service, "get_raw", return_value=raw_project)

    assert service._app_render_legacy_platforms() == expected


@pytest.mark.parametrize(
    "raw_project",
    [
        {"base": "core22", "platforms": None},
        {"base": "core22", "platforms": {"s390x": None}},
        {"base": "core22", "platforms": None, "architectures": ["s390x"]},
    ],
)
def test_render_legacy_platforms_core22_platforms_error(
    mocker: pytest_mock.MockerFixture,
    in_project_path: pathlib.Path,
    raw_project: dict[str, Any],
):
    service = Project(
        app=APP_METADATA,
        services=None,  # ty: ignore[invalid-argument-type] other services not needed
        project_dir=in_project_path,
    )
    mocker.patch.object(service, "get_raw", return_value=raw_project)

    with pytest.raises(CraftValidationError, match="not supported for base 'core22'"):
        service._app_render_legacy_platforms()


class TestValidateUaServices:
    @pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
    def test_warns_for_ua_services(self, base, emitter):
        """Warn for using 'ua-services' on core24+."""
        project = {"base": base, "ua-services": ["esm-apps"]}

        Project.validate_ua_services(project)
        Project.validate_ua_services(project)

        emitter.assert_warning(
            f"The 'ua-services' key is ignored for {base!r}. "
            "Use '--pro=<services>' instead."
        )
        # assert it was only shown once
        assert len(emitter.interactions) == 1

    @pytest.mark.parametrize("base", const.CURRENT_BASES)
    def test_no_warning_without_ua_services(self, base, emitter):
        """Don't warn if 'ua-services' isn't defined."""
        project = {"base": base}

        Project.validate_ua_services(project)

        emitter.assert_interactions(None)

    @pytest.mark.parametrize("base", const.CURRENT_BASES)
    def test_no_warning_in_managed_mode(self, base, emitter, mocker):
        """Don't warn in managed-mode."""
        mocker.patch("snapcraft.services.project.is_managed_mode", return_value=True)
        project = {"base": base, "ua-services": ["esm-apps"]}

        Project.validate_ua_services(project)

        emitter.assert_interactions(None)

    def test_no_warning_for_core22(self, emitter):
        """Don't warn for using 'ua-services' on core22."""
        project = {"base": "core22", "ua-services": ["esm-apps"]}

        Project.validate_ua_services(project)

        emitter.assert_interactions(None)


class TestValidateLicense:
    def test_non_spdx_deprecation_warns_once(self, emitter: RecordingEmitter) -> None:
        project = {"license": "maybe"}

        Project.validate_license_spdx(project)
        Project.validate_license_spdx(project)

        emitter.assert_warning(
            "Non-SPDX licenses are deprecated. Use SPDX license strings or 'proprietary' instead. For more information, see https://spdx.org/licenses/."
        )
        # assert it was only shown once
        assert len(emitter.interactions) == 1

    def test_no_warning_in_managed_mode(
        self, emitter: RecordingEmitter, mocker: pytest_mock.MockerFixture
    ):
        """Don't warn in managed-mode."""
        mocker.patch("snapcraft.services.project.is_managed_mode", return_value=True)
        project = {"license": "maybe"}

        Project.validate_ua_services(project)

        emitter.assert_interactions(None)

    @pytest.mark.parametrize(
        ("lic", "should_warn"),
        [
            ("MIT", False),
            ("proprietary", False),
            (None, False),
            ("DemonicContract", True),
        ],
    )
    def test_non_spdx_deprecation(
        self,
        lic: str | None,
        should_warn: bool,
        emitter: RecordingEmitter,
    ) -> None:
        project = {"license": lic}
        Project.validate_license_spdx(project)

        assert should_warn == (
            call(
                "warning",
                "Non-SPDX licenses are deprecated. Use SPDX license strings or 'proprietary' instead. For more information, see https://spdx.org/licenses/.",
            )
            in emitter.interactions
        )
        # License should always remain unchanged
        assert project.get("license") == lic
