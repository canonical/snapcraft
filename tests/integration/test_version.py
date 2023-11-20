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
"""Versioning tests."""
import re
import subprocess

import pytest


def _repo_has_version_tag() -> bool:
    """Returns True if the repo has a git tag usable for versioning."""
    git_describe_command = [
        "git",
        "describe",
        "--always",
        "--long",
        "--match",
        "[0-9]*.[0-9]*.[0-9]*",
        "--exclude",
        "*[^0-9.]*",
    ]
    output = subprocess.run(
        git_describe_command,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.rstrip("\n")

    # match on 'X.Y.Z-<commits since tag>-g<hash>'
    return bool(re.fullmatch(r"\d+\.\d+\.\d+-\d+-g[0-9a-f]+", output))


@pytest.mark.skipif(
    _repo_has_version_tag(),
    reason="Skipping because project was versioned from a tag.",
)
def test_version_without_tags(project_main_module):
    """Validate version format when no valid tag are present."""
    version = project_main_module.__version__

    # match on '0.0.post<total commits>+g<hash>'
    #       or '0.0.post<total commits>+g<hash>.d<%Y%m%d>'
    assert re.fullmatch(r"\d+\.\d+\.post\d+\+g[0-9a-f]+(\.d[0-9]*)?", version)


@pytest.mark.skipif(
    not _repo_has_version_tag(),
    reason="Skipping because project was not versioned from a tag.",
)
def test_version_with_tags(project_main_module):
    """Version should be properly formatted when a valid tag exists."""
    version = project_main_module.__version__

    # match on 'X.Y.Z'
    #       or 'X.Y.Z.d<%Y%m%d>'
    #       or 'X.Y.Z.post<commits since tag>+g<hash>'
    #       or 'X.Y.Z.post<commits since tag>+g<hash>.d<%Y%m%d>'
    assert re.fullmatch(r"\d+\.\d+\.\d+(\.post\d+\+g[0-9a-f]+)?(\.d[0-9]*)?", version)
