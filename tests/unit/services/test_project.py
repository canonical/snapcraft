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
    Project._snap_dir_source_warning = False
    yield
    Project._ua_service_warning = False
    Project._license_spdx_warning = False
    Project._snap_dir_source_warning = False


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


class TestWarnIfSourcesResolveFromParent:
    """Verify the warning for sources that only resolve from a parent directory."""

    @staticmethod
    def _project_dir(
        tmp_path: pathlib.Path,
        *,
        name: str = "snap",
        project_file: str = "snapcraft.yaml",
    ) -> pathlib.Path:
        """Create a project directory containing a project file."""
        project_dir = tmp_path / name
        (project_dir / project_file).parent.mkdir(parents=True, exist_ok=True)
        (project_dir / project_file).touch()
        return project_dir

    @staticmethod
    def _service(project_dir: pathlib.Path) -> Project:
        return Project(
            app=APP_METADATA,
            services=None,  # ty: ignore[invalid-argument-type] other services not needed
            project_dir=project_dir,
        )

    @staticmethod
    def _warnings(emitter: RecordingEmitter) -> list[Any]:
        """Return only the warnings from the recorded emitter interactions."""
        return [c for c in emitter.interactions if c.args[0] == "warning"]

    def test_warns_when_source_resolves_from_parent(
        self, emitter: RecordingEmitter, tmp_path: pathlib.Path
    ) -> None:
        """Warn when a local source exists in the parent of a 'snap' directory."""
        (tmp_path / "snap" / "local").mkdir(parents=True)
        project_dir = self._project_dir(tmp_path)
        project = {"parts": {"wrappers": {"plugin": "dump", "source": "snap/local"}}}

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        emitter.assert_warning(
            f"Part source 'snap/local' does not exist relative to {str(project_dir)!r}, "
            "but does exist relative to its parent. Snapcraft may be running from the "
            f"wrong directory - try running it from {str(project_dir.parent)!r} instead."
        )

    def test_warns_only_once(
        self, emitter: RecordingEmitter, tmp_path: pathlib.Path
    ) -> None:
        """The warning is shown no more than once."""
        (tmp_path / "snap" / "local").mkdir(parents=True)
        project_dir = self._project_dir(tmp_path)
        project = {"parts": {"wrappers": {"plugin": "dump", "source": "snap/local"}}}
        service = self._service(project_dir)

        service._warn_if_sources_resolve_from_parent(project)
        service._warn_if_sources_resolve_from_parent(project)

        assert len(self._warnings(emitter)) == 1

    def test_warns_for_every_unresolved_source(
        self, emitter: RecordingEmitter, tmp_path: pathlib.Path
    ) -> None:
        """Every source that only resolves from the parent is named."""
        (tmp_path / "alpha").mkdir()
        (tmp_path / "beta").mkdir()
        project_dir = self._project_dir(tmp_path)
        project = {
            "parts": {
                "one": {"plugin": "nil", "source": "alpha"},
                "two": {"plugin": "nil", "source": "beta"},
            }
        }

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        emitter.assert_warning(
            "Part source 'alpha' and 'beta' does not exist relative to "
            f"{str(project_dir)!r}, but does exist relative to its parent. "
            "Snapcraft may be running from the wrong directory - try running it "
            f"from {str(project_dir.parent)!r} instead."
        )

    def test_no_warning_when_source_resolves_in_place(
        self, emitter: RecordingEmitter, tmp_path: pathlib.Path
    ) -> None:
        """Don't warn when the source exists in the 'snap' directory."""
        (tmp_path / "snap" / "local").mkdir(parents=True)
        project_dir = self._project_dir(tmp_path)
        project = {"parts": {"wrappers": {"plugin": "dump", "source": "local"}}}

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        assert not self._warnings(emitter)

    @pytest.mark.parametrize(
        "source",
        [
            pytest.param("https://github.com/juju/juju.git", id="url"),
            pytest.param("git+https://github.com/juju/juju.git", id="git-url"),
            pytest.param(None, id="no-source"),
            pytest.param("", id="empty-source"),
        ],
    )
    def test_no_warning_without_local_sources(
        self,
        emitter: RecordingEmitter,
        tmp_path: pathlib.Path,
        source: str | None,
    ) -> None:
        """Don't warn when no part has a local source that could be misresolved."""
        project_dir = self._project_dir(tmp_path)
        project = {"parts": {"juju": {"plugin": "nil", "source": source}}}

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        assert not self._warnings(emitter)

    def test_no_warning_outside_snap_directory(
        self, emitter: RecordingEmitter, tmp_path: pathlib.Path
    ) -> None:
        """Don't warn when the project directory isn't named 'snap'."""
        (tmp_path / "snap" / "local").mkdir(parents=True)
        project_dir = self._project_dir(tmp_path, name="juju")
        project = {"parts": {"wrappers": {"plugin": "dump", "source": "snap/local"}}}

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        assert not self._warnings(emitter)

    def test_no_warning_for_nested_project_file(
        self, emitter: RecordingEmitter, tmp_path: pathlib.Path
    ) -> None:
        """Don't warn when the project file isn't directly in the project directory."""
        (tmp_path / "alpha").mkdir()
        project_dir = self._project_dir(
            tmp_path, project_file="build-aux/snap/snapcraft.yaml"
        )
        project = {"parts": {"one": {"plugin": "nil", "source": "alpha"}}}

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        assert not self._warnings(emitter)

    @pytest.mark.parametrize(
        "project",
        [
            pytest.param({}, id="no-parts"),
            pytest.param({"parts": None}, id="null-parts"),
            pytest.param({"parts": {"empty": None}}, id="empty-part"),
            pytest.param(
                {"parts": {"none": {"plugin": "nil"}}}, id="part-without-source"
            ),
            pytest.param(
                {"parts": {"grammar": {"source": {"on amd64": "alpha"}}}},
                id="grammar-source",
            ),
        ],
    )
    def test_no_warning_without_resolvable_parts(
        self,
        emitter: RecordingEmitter,
        tmp_path: pathlib.Path,
        project: dict[str, Any],
    ) -> None:
        """Don't warn for parts without a plain local source."""
        (tmp_path / "alpha").mkdir()
        project_dir = self._project_dir(tmp_path)

        self._service(project_dir)._warn_if_sources_resolve_from_parent(project)

        assert not self._warnings(emitter)
