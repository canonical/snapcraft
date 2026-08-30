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

"""Integration tests for gpu extension's Makefile."""

import shutil
import stat
import subprocess
from pathlib import Path

import pytest

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


def test_bare_make_is_a_noop(command_chain_dir):
    """A bare `make` call doesn't install (snapcraft#6328)."""
    result = subprocess.run(
        ["make"],
        cwd=command_chain_dir,
        capture_output=True,
        text=True,
        check=True,
    )

    assert "install" not in result.stdout
    assert not (command_chain_dir / "gpu-2404-wrapper").exists()


@pytest.mark.parametrize("gpu_interface", GPU_INTERFACES)
def test_make_install_writes_to_destdir(command_chain_dir, tmp_path, gpu_interface):
    """`make install DESTDIR=...` installs the wrapper under DESTDIR."""
    install_dir = tmp_path / "install"

    subprocess.run(
        ["make", f"GPU_INTERFACE={gpu_interface}"], cwd=command_chain_dir, check=True
    )
    subprocess.run(
        [
            "make",
            "install",
            f"GPU_INTERFACE={gpu_interface}",
            f"DESTDIR={install_dir}",
        ],
        cwd=command_chain_dir,
        check=True,
    )

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


def test_make_install_is_idempotent(command_chain_dir, tmp_path):
    """Make install can run more than once."""
    install_dir = tmp_path / "install"

    for _ in range(2):
        subprocess.run(["make"], cwd=command_chain_dir, check=True)
        subprocess.run(
            ["make", "install", f"DESTDIR={install_dir}"],
            cwd=command_chain_dir,
            check=True,
        )

    wrapper = install_dir / "snap" / "command-chain" / "gpu-2404-wrapper"
    assert wrapper.is_file()
