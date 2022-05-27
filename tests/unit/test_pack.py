# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import subprocess
from unittest.mock import call

import pytest

from snapcraft import errors, pack


def test_pack_snap(mocker, new_dir):
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None)
    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_compression_none(mocker, new_dir):
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None, compression=None)
    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_compression(mocker, new_dir):
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None, compression="zz")
    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--compression", "zz", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_cwd(mocker, new_dir):
    """Test `snap pack` when it outputs to the current working directory."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=f"{new_dir}/test.snap")
    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--filename", "test.snap", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_existing(mocker, new_dir):
    """Test `snap pack` when it outputs to an existing directory."""
    mock_run = mocker.patch("subprocess.run")
    output_directory = new_dir / "output"
    output_directory.mkdir()
    assert output_directory.is_dir()

    pack.pack_snap(new_dir, output=output_directory / "test.snap")

    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--filename", "test.snap", new_dir, str(output_directory)],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_non_existant(mocker, new_dir):
    """Test `snap pack` when it outputs to a non-existent directory."""
    mock_run = mocker.patch("subprocess.run")
    output_directory = new_dir / "output"
    assert not output_directory.exists()

    pack.pack_snap(new_dir, output=output_directory / "test.snap")

    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            [
                "snap",
                "pack",
                "--filename",
                "test.snap",
                new_dir,
                str(new_dir / "output"),
            ],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_directory_not_specified(mocker, new_dir):
    """Test `snap pack` executes when no output directory is specified."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=str(new_dir))
    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, str(new_dir)],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_existing_no_file_name(mocker, new_dir):
    """Test `snap pack` when it outputs to an existing directory but no file
    name is specified."""
    mock_run = mocker.patch("subprocess.run")
    output_directory = new_dir / "output"
    output_directory.mkdir()
    assert output_directory.is_dir()

    pack.pack_snap(new_dir, output=output_directory)

    assert mock_run.mock_calls == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, str(output_directory)],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_error(mocker, new_dir):
    mocker.patch("subprocess.run", side_effect=subprocess.CalledProcessError(42, "cmd"))
    with pytest.raises(errors.SnapcraftError) as raised:
        pack.pack_snap(new_dir, output=str(new_dir))

    assert str(raised.value) == (
        "Cannot pack snap file: Command 'cmd' returned non-zero exit status 42."
    )
