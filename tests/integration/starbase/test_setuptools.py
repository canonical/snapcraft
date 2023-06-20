# This file is part of starcraft.
#
# Copyright 2023 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License version 3, as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
# SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Starcraft integration tests related to building the package."""
import re
import subprocess
from pathlib import Path
from zipfile import ZipFile


def test_packages(tmp_path, request):
    """Check wheel generation from our pyproject.toml"""
    root_dir = Path(request.config.rootdir)
    out_dir = tmp_path
    subprocess.check_call(["python3", "-m", "build", "--outdir", out_dir, root_dir])
    wheels = list(tmp_path.glob("*.whl"))
    assert len(wheels) == 1
    wheel = wheels[0]

    starcraft_files = []

    dist_files = []
    dist_info_re = re.compile("starcraft-.*.dist-info")

    invalid = []

    with ZipFile(wheel) as wheel_zip:
        names = [Path(p) for p in wheel_zip.namelist()]
        assert len(names) > 1
        for name in names:
            top = name.parts[0]
            if top == "starcraft":
                starcraft_files.append(name)
            elif dist_info_re.match(top):
                dist_files.append(top)
            else:
                invalid = []

    # Only the top-level "starcraft" dir should be present, plus the
    # starcraft-xyz-dist-info/ entries.
    assert starcraft_files, "No 'starcraft' modules were packaged!"
    assert dist_files, "The dist-info directory was not created!"
    assert not invalid, f"Invalid files were packaged: {invalid}"
