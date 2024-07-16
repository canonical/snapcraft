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

"""Snapcraft RemoteBuild service tests."""

import pathlib

import pytest


@pytest.mark.parametrize(
    ("new_exists", "legacy_exists", "expected"),
    [
        (False, False, pathlib.Path("launchpad-credentials")),
        (False, True, pathlib.Path("provider/launchpad/credentials")),
        (True, False, pathlib.Path("launchpad-credentials")),
        (True, True, pathlib.Path("launchpad-credentials")),
    ],
)
def test_credentials_filepaths(
    new_exists, legacy_exists, expected, remote_build_service, mocker, new_dir
):
    """Load legacy credentials only when they exist and the new ones do not."""
    mocker.patch("platformdirs.user_data_path", return_value=new_dir)
    if new_exists:
        (new_dir / "launchpad-credentials").touch()
    if legacy_exists:
        (new_dir / "provider/launchpad/credentials").mkdir(parents=True)
        (new_dir / "provider/launchpad/credentials").touch()

    credentials_filepath = remote_build_service.credentials_filepath

    assert credentials_filepath == new_dir / expected
