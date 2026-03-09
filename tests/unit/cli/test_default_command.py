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

import sys

import pytest

from snapcraft import cli


@pytest.mark.parametrize(
    "args",
    [
        [],
        ["--destructive-mode"],
        ["--use-lxd"],
        ["-o", "name"],
        ["--output", "name"],
        ["--http-proxy", "test-http"],
        ["--https-proxy", "test-https"],
    ],
)
def test_default_command_with_option(mocker, capsys, args):
    expected = "Missing a command. Try 'snapcraft pack' or 'snapcraft help'."
    mocker.patch.object(sys, "argv", ["cmd", *args])

    exit_code = cli.run()

    assert exit_code == 1
    assert expected in capsys.readouterr().err
