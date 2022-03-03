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

import argparse
import sys
from unittest.mock import call

import pytest

from snapcraft import cli


def test_default_command(mocker):
    mocker.patch.object(sys, "argv", ["cmd"])
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output=None,
                destructive_mode=False,
                use_lxd=False,
                provider=None,
            )
        )
    ]


def test_default_command_parameters(mocker):
    mocker.patch.object(
        sys, "argv", ["cmd", "--destructive-mode", "--use-lxd", "--provider=lxd"]
    )
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output=None,
                destructive_mode=True,
                use_lxd=True,
                provider="lxd",
            )
        )
    ]


@pytest.mark.parametrize("option", ["-o", "--output"])
def test_default_command_output(mocker, option):
    mocker.patch.object(sys, "argv", ["cmd", option, "name"])
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output="name",
                destructive_mode=False,
                use_lxd=False,
                provider=None,
            )
        )
    ]
