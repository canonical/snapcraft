# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

from pathlib import Path
from typing import Any
from unittest.mock import Mock

import pytest

from snapcraft import linters, models
from snapcraft.elf import ElfFile, elf_utils
from snapcraft.linters.gpu_linter import GpuLinter
from snapcraft.meta import snap_yaml

_YAML_DATA = {
    "name": "mytest",
    "version": "1.0.0",
    "summary": "Test snap",
    "description": "test-description",
    "confinement": "strict",
    "parts": {},
}


def setup_function():
    elf_utils.get_elf_files.cache_clear()


@pytest.fixture
def mock_elf_files(mocker):
    """Mock elf_utils.get_elf_files to return controlled ElfFile instances."""

    def _mock_elf_files(files):
        """Create mock ElfFile objects with specified sonames."""
        elf_files = []
        for filepath, soname in files:
            mock_elf = Mock(spec=ElfFile)
            mock_elf.path = Path(filepath)
            mock_elf.soname = soname
            elf_files.append(mock_elf)

        mocker.patch("snapcraft.elf.elf_utils.get_elf_files", return_value=elf_files)

    return _mock_elf_files


@pytest.mark.parametrize(
    "files,expected_count",
    [
        # Libraries that ARE detected on core22
        pytest.param(
            [("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1")], 1, id="libGL"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1")], 1, id="libEGL"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libGLESv2.so.2", "libGLESv2.so.2")],
            1,
            id="libGLESv2",
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1")],
            1,
            id="libvulkan",
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libgbm.so.1", "libgbm.so.1")], 1, id="libgbm"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libGL_nvidia.so.1", None)], 1, id="libGL-nvidia"
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/dri/i965_dri.so", None)], 1, id="i965-dri"
        ),
        # Libraries that are NOT detected on core22
        pytest.param(
            [("lib/x86_64-linux-gnu/libGLESv1_CM.so.1", "libGLESv1_CM.so.1")],
            0,
            id="libGLESv1-not-detected",
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/gbm/nvidia_gbm.so", None)],
            0,
            id="nvidia-gbm-not-detected",
        ),
    ],
)
def test_gpu_linter_core22(mocker, new_dir, mock_elf_files, files, expected_count):
    """Test GPU linter detection for core22 base (excludes GLESv1 and gbm backends)."""
    mock_elf_files(files)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"gpu": GpuLinter})

    yaml_data: dict[str, Any] = {**_YAML_DATA, "base": "core22"}

    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    assert len(issues) == expected_count
    for issue in issues:
        assert issue.name == "gpu"
        assert (
            issue.text
            == "GPU support library should be provided by a content interface."
        )
        assert (
            issue.url
            == "https://canonical-ubuntu-frame-documentation.readthedocs-hosted.com/how-to/use-snap-graphics-on-base-core22/"
        )


@pytest.mark.parametrize(
    "files,expected_count",
    [
        # Standard GPU libraries
        pytest.param(
            [("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1")], 1, id="libGL"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1")], 1, id="libEGL"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libGLESv1_CM.so.1", "libGLESv1_CM.so.1")],
            1,
            id="libGLESv1",
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libGLESv2.so.2", "libGLESv2.so.2")],
            1,
            id="libGLESv2",
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libdrm.so.2", "libdrm.so.2")], 1, id="libdrm"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1")],
            1,
            id="libvulkan",
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libgbm.so.1", "libgbm.so.1")], 1, id="libgbm"
        ),
        # Vendor-specific libraries
        pytest.param(
            [("lib/x86_64-linux-gnu/libGL_nvidia.so.1", None)], 1, id="libGL-nvidia"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libEGL_mesa.so.0", None)], 1, id="libEGL-mesa"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libGLX_nvidia.so.0", None)], 1, id="libGLX-nvidia"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libdrm_amdgpu.so.1", None)], 1, id="libdrm-amdgpu"
        ),
        pytest.param(
            [("lib/x86_64-linux-gnu/libvulkan_radeon.so", None)],
            1,
            id="libvulkan-radeon",
        ),
        # Driver modules
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/gbm/nvidia_gbm.so", None)], 1, id="nvidia-gbm"
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/dri/i965_dri.so", None)], 1, id="i965-dri"
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/dri/radeonsi_dri.so", None)],
            1,
            id="radeonsi-dri",
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/dri/iris_dri.so", None)], 1, id="iris-dri"
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/dri/nouveau_dri.so", None)], 1, id="nouveau-dri"
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/dri/nvidia_drv_video.so", None)],
            1,
            id="nvidia-drv-video",
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/vdpau/libvdpau_nvidia.so.1", None)],
            1,
            id="vdpau-nvidia",
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/vdpau/libvdpau_r600.so.1", None)],
            1,
            id="vdpau-r600",
        ),
        pytest.param([("usr/lib/x86_64-linux-gnu/libva.so.2", None)], 1, id="libva"),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/libva-drm.so.2", None)], 1, id="libva-drm"
        ),
        pytest.param(
            [("usr/lib/x86_64-linux-gnu/libvdpau.so.1", None)], 1, id="libvdpau"
        ),
        # Multiple libraries
        pytest.param(
            [
                ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
                ("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1"),
                ("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1"),
            ],
            3,
            id="multiple-libs",
        ),
        # Mixed standard and vendor-specific
        pytest.param(
            [
                ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
                ("lib/x86_64-linux-gnu/libGL_nvidia.so.1", None),
                ("usr/lib/x86_64-linux-gnu/dri/i965_dri.so", None),
            ],
            3,
            id="mixed-libs",
        ),
        # Non-GPU libraries (should be ignored)
        pytest.param(
            [
                ("lib/x86_64-linux-gnu/libc.so.6", "libc.so.6"),
                ("lib/x86_64-linux-gnu/libm.so.6", "libm.so.6"),
            ],
            0,
            id="non-gpu-libs",
        ),
    ],
)
def test_gpu_linter_core24(mocker, new_dir, mock_elf_files, files, expected_count):
    """Test GPU linter detection for core24 base."""
    mock_elf_files(files)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"gpu": GpuLinter})

    yaml_data: dict[str, Any] = {**_YAML_DATA, "base": "core24"}

    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    assert len(issues) == expected_count
    for issue in issues:
        assert issue.name == "gpu"
        assert (
            issue.text
            == "GPU support library should be provided by a content interface."
        )
        assert (
            issue.url
            == "https://canonical-ubuntu-frame-documentation.readthedocs-hosted.com/how-to/use-snap-graphics-on-base-core24/"
        )


@pytest.mark.parametrize(
    ("base_config"),
    [
        pytest.param({"type": str(proj_type), "build-base": "core24"}, id=proj_type) for proj_type in const.ProjectType if proj_type != const.ProjectType.APP
    ],
)
def test_gpu_linter_non_app_snap_types(mocker, new_dir, mock_elf_files, base_config):
    """Test that GPU linter is skipped for non-app snap types."""
    mock_elf_files(
        [
            ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
            ("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1"),
            ("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1"),
        ]
    )
    mocker.patch("snapcraft.linters.linters.LINTERS", {"gpu": GpuLinter})

    yaml_data: dict[str, Any] = {**_YAML_DATA, **base_config}

    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    # Non-app snap types should never trigger GPU linter warnings
    assert len(issues) == 0


@pytest.mark.parametrize(
    "files,ignored_pattern,expected_count",
    [
        # Ignore single library
        pytest.param(
            [
                ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
                ("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1"),
            ],
            "**/libGL.so.1",
            1,
            id="ignore-single-lib",
        ),
        # Ignore all libraries
        pytest.param(
            [
                ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
                ("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1"),
                ("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1"),
            ],
            "**/*.so.*",
            0,
            id="ignore-all-libs",
        ),
        # Ignore with wildcard pattern
        pytest.param(
            [
                ("usr/lib/x86_64-linux-gnu/dri/i965_dri.so", None),
                ("usr/lib/x86_64-linux-gnu/dri/radeonsi_dri.so", None),
                ("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1"),
            ],
            "**/dri/*",
            1,
            id="ignore-dri-wildcard",
        ),
    ],
)
def test_gpu_linter_with_ignore_filter(
    mocker, new_dir, mock_elf_files, files, ignored_pattern, expected_count
):
    """Test that GPU linter respects ignore filters."""
    mock_elf_files(files)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"gpu": GpuLinter})

    yaml_data: dict[str, Any] = {**_YAML_DATA, "base": "core24"}

    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    lint_config = models.Lint(ignore=[{"gpu": [ignored_pattern]}])
    issues = linters.run_linters(new_dir, lint=lint_config)

    assert len(issues) == expected_count
    for issue in issues:
        assert issue.name == "gpu"


@pytest.mark.parametrize(
    "base,files",
    [
        # core22 with classic confinement (should be skipped)
        pytest.param(
            "core22",
            [("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1")],
            id="core22-libGL",
        ),
        pytest.param(
            "core22",
            [("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1")],
            id="core22-libEGL",
        ),
        pytest.param(
            "core22",
            [
                ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
                ("lib/x86_64-linux-gnu/libvulkan.so.1", "libvulkan.so.1"),
            ],
            id="core22-multiple",
        ),
        # core24 with classic confinement (should be skipped)
        pytest.param(
            "core24",
            [("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1")],
            id="core24-libGL",
        ),
        pytest.param(
            "core24",
            [("lib/x86_64-linux-gnu/libGLESv1_CM.so.1", "libGLESv1_CM.so.1")],
            id="core24-libGLESv1",
        ),
        pytest.param(
            "core24",
            [
                ("lib/x86_64-linux-gnu/libGL.so.1", "libGL.so.1"),
                ("lib/x86_64-linux-gnu/libEGL.so.1", "libEGL.so.1"),
                ("usr/lib/x86_64-linux-gnu/gbm/nvidia_gbm.so", None),
            ],
            id="core24-mixed",
        ),
    ],
)
def test_gpu_linter_classic_confinement(mocker, new_dir, mock_elf_files, base, files):
    """Test that GPU linter is skipped for classic confinement snaps."""
    mock_elf_files(files)
    mocker.patch("snapcraft.linters.linters.LINTERS", {"gpu": GpuLinter})

    yaml_data: dict[str, Any] = {**_YAML_DATA, "base": base, "confinement": "classic"}

    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    # Classic confinement snaps should never trigger GPU linter warnings
    assert len(issues) == 0
