# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

import re
from argparse import Namespace
from textwrap import dedent

import pytest

import snapcraft.commands
from snapcraft import errors


@pytest.mark.usefixtures("fake_extension")
def test_command(emitter, fake_app_config):
    cmd = snapcraft.commands.ExtensionsCommand(fake_app_config)

    cmd.run(Namespace())

    emitter.assert_message(
        dedent(
            """\
        Extension name        Supported bases
        --------------------  ----------------------
        dotnet10              core24
        dotnet8               core24
        dotnet9               core24
        env-injector          core24
        fake-extension        core22, core24, core26
        gnome                 core22, core24
        kde-neon              core22, core24
        kde-neon-6            core22, core24
        kde-neon-qt6          core22, core24
        ros2-humble           core22
        ros2-humble-desktop   core22
        ros2-humble-ros-base  core22
        ros2-humble-ros-core  core22
        ros2-jazzy            core24
        ros2-jazzy-desktop    core24
        ros2-jazzy-ros-base   core24
        ros2-jazzy-ros-core   core24"""
        )
    )


def test_list_extensions_error(fake_app_config):
    """Error on removed 'list-extensions' command."""
    cmd = snapcraft.commands.ListExtensionsCommand(fake_app_config)
    expected = re.escape("The 'list-extensions' command was renamed to 'extensions'.")

    with pytest.raises(errors.RemovedCommand, match=expected):
        cmd.run(Namespace())
