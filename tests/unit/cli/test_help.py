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
from unittest.mock import call

import pytest

from snapcraft import cli


def test_help_command(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "help"])
    mock_dispatcher_run = mocker.patch("craft_cli.dispatcher.Dispatcher.run")
    mock_legacy_run = mocker.patch("snapcraft_legacy.cli.legacy.legacy_run")

    cli.run()

    assert mock_dispatcher_run.mock_calls == []
    assert mock_legacy_run.mock_calls == [call()]


@pytest.mark.parametrize("arg", ["-h", "--help"])
def test_help_option(mocker, arg):
    mocker.patch.object(sys, "argv", ["cmd", arg])
    mock_dispatcher_run = mocker.patch("craft_cli.dispatcher.Dispatcher.run")
    mock_legacy_run = mocker.patch(
        "snapcraft_legacy.cli.legacy.legacy_run", side_effect=SystemExit
    )

    with pytest.raises(SystemExit):
        cli.run()

    assert mock_dispatcher_run.mock_calls == []
    assert mock_legacy_run.mock_calls == [call()]
