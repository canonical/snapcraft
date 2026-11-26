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

"""Integration tests for the gpu extension's command-chain build script."""

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

from snapcraft.extensions.gpu_extension import _gpu_wrapper_override_build

COMMAND_CHAIN_DIR = (
    Path(__file__).resolve().parents[3] / "extensions" / "gpu" / "command-chain"
)

GPU_INTERFACES = ["graphics-core22", "gpu-2404", "gpu-2604"]


@pytest.fixture
def command_chain_dir(tmp_path):
    """Copy the gpu command-chain sources into a temporary build directory."""
    build_dir = tmp_path / "build"
    shutil.copytree(COMMAND_CHAIN_DIR, build_dir)
    return build_dir


def _run_override_build(gpu_interface: str, build_dir: Path, install_dir: Path) -> None:
    subprocess.run(
        ["bash", "-ec", _gpu_wrapper_override_build(gpu_interface)],
        cwd=build_dir,
        env={**os.environ, "CRAFT_PART_INSTALL": str(install_dir)},
        check=True,
    )


@pytest.mark.parametrize("gpu_interface", GPU_INTERFACES)
def test_override_build_installs_wrapper(command_chain_dir, tmp_path, gpu_interface):
    """The override-build script installs the wrapper for the interface."""
    install_dir = tmp_path / "install"

    _run_override_build(gpu_interface, command_chain_dir, install_dir)

    wrapper = install_dir / "snap" / "command-chain" / f"{gpu_interface}-wrapper"
    assert wrapper.is_file()

    assert stat.S_IMODE(wrapper.stat().st_mode) == 0o755

    content = wrapper.read_text()
    assert "%GPU_INTERFACE%" not in content
    assert (
        f'GPU_WRAPPER="${{SNAP}}/{gpu_interface}/bin/{gpu_interface}-provider-wrapper"'
        in content
    )
    assert f"Usage: {gpu_interface}-wrapper <command>" in content


def test_override_build_is_idempotent(command_chain_dir, tmp_path):
    """The override-build script can run more than once."""
    install_dir = tmp_path / "install"

    for _ in range(2):
        _run_override_build("gpu-2404", command_chain_dir, install_dir)

    wrapper = install_dir / "snap" / "command-chain" / "gpu-2404-wrapper"
    assert wrapper.is_file()
    assert "%GPU_INTERFACE%" not in wrapper.read_text()
