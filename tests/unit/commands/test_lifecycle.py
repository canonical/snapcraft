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
from unittest.mock import call

import pytest

from snapcraft.commands.lifecycle import (
    BuildCommand,
    PrimeCommand,
    PullCommand,
    StageCommand,
)


@pytest.mark.parametrize(
    "step_name,cmd_class",
    [
        ("pull", PullCommand),
        ("build", BuildCommand),
        ("stage", StageCommand),
        ("prime", PrimeCommand),
    ],
)
def test_lifecycle_command(step_name, cmd_class, mocker):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.run_lifecycle")
    cmd = cmd_class(None)
    cmd.run(argparse.Namespace(parts=["part1", "part2"]))
    assert lifecycle_run_mock.mock_calls == [
        call(step_name, argparse.Namespace(parts=["part1", "part2"]))
    ]
