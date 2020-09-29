# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from unittest import mock

import pytest

from snapcraft.cli import lifecycle


@pytest.mark.parametrize(
    "output,pack_name,pack_dir",
    [
        ("/tmp/output.snap", "output.snap", "/tmp"),
        ("/tmp", None, "/tmp"),
        ("output.snap", "output.snap", None),
    ],
)
@pytest.mark.parametrize(
    "compression", ["xz", "lzo", None],
)
@mock.patch("snapcraft.file_utils.get_host_tool_path", return_value="/bin/snap")
@mock.patch("snapcraft.cli.lifecycle._run_pack", return_value="ignore.snap")
def test_pack(mock_run_pack, mock_host_tool, compression, output, pack_name, pack_dir):
    lifecycle._pack(directory="/my/snap", compression=compression, output=output)

    assert mock_host_tool.mock_calls == [
        mock.call(command_name="snap", package_name="snapd")
    ]

    pack_command = ["/bin/snap", "pack"]

    if compression:
        pack_command.extend(["--compression", compression])

    if pack_name:
        pack_command.extend(["--filename", pack_name])

    pack_command.append("/my/snap")

    if pack_dir:
        pack_command.append(pack_dir)

    assert mock_run_pack.mock_calls == [mock.call(pack_command)]
