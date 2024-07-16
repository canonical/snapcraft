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

from snapcraft import __version__, application


def test_version_command(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "version"])
    app = application.create_app()
    mock_version_cmd = mocker.patch(
        "craft_application.commands.other.VersionCommand.run"
    )
    app.run()
    assert mock_version_cmd.mock_calls == [
        call(argparse.Namespace()),
        call().__bool__(),
    ]


def test_version_argument(mocker, emitter):
    mocker.patch.object(sys, "argv", ["cmd", "--version"])
    app = application.create_app()
    with pytest.raises(SystemExit):
        app.run()
    emitter.assert_message(f"snapcraft {__version__}")


def test_version_argument_with_command(mocker, emitter):
    mocker.patch.object(sys, "argv", ["cmd", "--version", "version"])
    app = application.create_app()
    with pytest.raises(SystemExit):
        app.run()
    emitter.assert_message(f"snapcraft {__version__}")
