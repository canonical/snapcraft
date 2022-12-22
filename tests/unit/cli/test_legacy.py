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


@pytest.fixture
def legacy_run(mocker):
    return mocker.patch("snapcraft.commands.legacy.LegacyBaseCommand.run")


def test_promote_command(mocker, legacy_run):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            "promote",
            "name",
            "--from-channel",
            "edge",
            "--to-channel",
            "edge/foo",
        ],
    )

    cli.run()

    assert legacy_run.mock_calls == [
        call(
            argparse.Namespace(
                snap_name="name", from_channel="edge", to_channel="edge/foo", yes=False
            )
        )
    ]


def test_promote_command_yes(mocker, legacy_run):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            "promote",
            "name",
            "--from-channel",
            "edge",
            "--to-channel",
            "edge/foo",
            "--yes",
        ],
    )

    cli.run()

    assert legacy_run.mock_calls == [
        call(
            argparse.Namespace(
                snap_name="name", from_channel="edge", to_channel="edge/foo", yes=True
            )
        )
    ]


def test_list_validation_sets(mocker, legacy_run):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            "list-validation-sets",
        ],
    )

    cli.run()

    assert legacy_run.mock_calls == [call(argparse.Namespace(name=None, sequence=None))]


def test_list_validation_sets_with_options(mocker, legacy_run):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "list-validation-sets", "--name", "set-name", "--sequence", "all"],
    )

    cli.run()

    assert legacy_run.mock_calls == [
        call(argparse.Namespace(name="set-name", sequence="all"))
    ]
