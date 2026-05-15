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

"""Integration tests for the library linter using real ELF binaries."""

import shutil
import subprocess
import urllib.request
from pathlib import Path

import pytest

from snapcraft import linters, models
from snapcraft.elf import elf_utils
from snapcraft.meta import snap_yaml

_POWERPC_BASH_URL = "https://old-releases.ubuntu.com/ubuntu/pool/main/b/bash/bash_2.05b-15ubuntu5_powerpc.deb"


def setup_function():
    elf_utils.get_elf_files.cache_clear()


@pytest.fixture
def powerpc_bash(tmp_path):
    """Download and extract the bash binary from a powerpc .deb package.

    Returns the path to the extracted bash executable, which is a 32-bit
    big-endian PowerPC ELF (EM_PPC) — foreign on every supported host arch.
    """
    deb_path = tmp_path / "bash-powerpc.deb"
    extract_dir = tmp_path / "bash-powerpc"

    try:
        urllib.request.urlretrieve(_POWERPC_BASH_URL, deb_path)  # noqa: S310
    except OSError as exc:
        pytest.skip(f"Could not download powerpc bash package: {exc}")

    extract_dir.mkdir()
    result = subprocess.run(
        ["dpkg-deb", "-x", str(deb_path), str(extract_dir)],
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        pytest.skip(f"Could not extract powerpc bash package: {result.stderr.decode()}")

    return extract_dir / "bin" / "bash"


def test_library_linter_skips_foreign_arch_elf(powerpc_bash, new_dir):
    """The library linter must not call load_dependencies() on a foreign-arch ELF.

    A real powerpc (EM_PPC) bash binary is placed in the prime directory.
    On every supported host the linter should silently skip it and report
    no library issues, rather than trying to invoke it via binfmt/QEMU.
    """
    shutil.copy(powerpc_bash, "bash-powerpc")

    yaml_data = {
        "name": "mytest",
        "version": "1.0",
        "base": "core22",
        "summary": "Foreign-arch ELF linter integration test",
        "description": "test",
        "confinement": "strict",
        "parts": {},
    }
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    library_issues = [i for i in issues if i.name == "library"]
    assert library_issues == []
