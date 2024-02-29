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

from unittest.mock import call

import pytest

from snapcraft import errors, pack


def test_pack_snap(mocker, new_dir):
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None)
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_compression_none(mocker, new_dir):
    """No compression uses snap default."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None, compression=None)
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_compression(mocker, new_dir):
    """Compression should be passed to snap pack."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None, compression="zz")
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--compression", "zz", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_cwd(mocker, new_dir):
    """Output to a filename in the current working directory."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=f"{new_dir}/test.snap")
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--filename", "test.snap", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_existing(mocker, new_dir):
    """Output to an existing directory."""
    mock_run = mocker.patch("subprocess.run")
    output_directory = new_dir / "output"
    output_directory.mkdir()
    assert output_directory.is_dir()

    pack.pack_snap(new_dir, output=output_directory / "test.snap")

    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--filename", "test.snap", new_dir, output_directory],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_non_existant(mocker, new_dir):
    """Output to a non-existent directory."""
    mock_run = mocker.patch("subprocess.run")
    output_directory = new_dir / "output"
    assert not output_directory.exists()

    pack.pack_snap(new_dir, output=output_directory / "test.snap")

    assert mock_run.mock_calls[:2] == [
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
                (new_dir / "output"),
            ],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_directory_cwd_no_filename(mocker, new_dir):
    """Output to the current working directory when no filename is specified."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=str(new_dir))
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_output_file_output_directory_existing_no_filename(mocker, new_dir):
    """Outputs to an existing directory when no filename is specified."""
    mock_run = mocker.patch("subprocess.run")
    output_directory = new_dir / "output"
    output_directory.mkdir()
    assert output_directory.is_dir()

    pack.pack_snap(new_dir, output=output_directory)

    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, output_directory],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


@pytest.mark.parametrize(
    "parameters",
    [
        {"name": "hello", "version": "1.0"},
        {"name": "hello", "target_arch": "armhf"},
        {"version": "1.0", "target_arch": "armhf"},
    ],
)
def test_pack_snap_file_name_missing_parameters(mocker, new_dir, parameters):
    """If name, version, and target architecture are not all specified, then use
    snap's default naming convention."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(new_dir, output=None, **parameters)
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_file_name_valid(mocker, new_dir):
    """Passing name, version, and target_arch should produce a valid file name."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(
        new_dir,
        output=None,
        name="hello",
        version="1.0",
        target_arch="armhf",
    )
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--filename", "hello_1.0_armhf.snap", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_use_output_name_over_name_version_arch(mocker, new_dir):
    """Output filename takes priority over name, version, and target_arch parameters."""
    mock_run = mocker.patch("subprocess.run")
    pack.pack_snap(
        new_dir,
        output="test.snap",
        name="hello",
        version="1.0",
        target_arch="armhf",
    )
    assert mock_run.mock_calls[:2] == [
        call(
            ["snap", "pack", "--check-skeleton", new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
        call(
            ["snap", "pack", "--filename", "test.snap", new_dir, new_dir],
            capture_output=True,
            check=True,
            universal_newlines=True,
        ),
    ]


def test_pack_snap_error(mocker, new_dir, fake_process):
    fake_process.register_subprocess(
        ["snap", "pack", "--check-skeleton", str(new_dir)],
        stdout=b"xxxx",
        stderr=b'error: cannot validate snap "pack-error": '
        b'invalid definition of application "pack-error": '
        b"app description field 'command' contains illegal "
        b"\"pack-error foo=bar\" (legal: '^[A-Za-z0-9/. _#:$-]*$')",
        returncode=1,
    )
    with pytest.raises(errors.SnapcraftError) as raised:
        pack.pack_snap(new_dir, output=str(new_dir))

    assert str(raised.value) == (
        """Cannot pack snap: error: cannot validate snap "pack-error": """
        """invalid definition of application "pack-error": """
        """app description field 'command' contains illegal "pack-error foo=bar" """
        """(legal: '^[A-Za-z0-9/. _#:$-]*$')"""
    )
