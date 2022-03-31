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

from argparse import Namespace
from textwrap import dedent

import pytest

from snapcraft.commands import ExtensionsCommand, ListExtensionsCommand


@pytest.mark.usefixtures("fake_extension")
@pytest.mark.parametrize("command", [ListExtensionsCommand, ExtensionsCommand])
def test_command(emitter, command):
    cmd = command(None)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            """\
        Extension name    Supported bases
        ----------------  -----------------
        fake-extension    core22
        flutter-beta      core18
        flutter-dev       core18
        flutter-master    core18
        flutter-stable    core18
        gnome             core22
        gnome-3-28        core18
        gnome-3-34        core18
        gnome-3-38        core20
        kde-neon          core18, core20
        ros1-noetic       core20
        ros2-foxy         core20"""
        )
    )


@pytest.mark.usefixtures("fake_extension_name_from_legacy")
@pytest.mark.parametrize("command", [ListExtensionsCommand, ExtensionsCommand])
def test_command_extension_dups(emitter, command):
    cmd = command(None)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            """\
        Extension name    Supported bases
        ----------------  -----------------
        flutter-beta      core18
        flutter-dev       core18
        flutter-master    core18
        flutter-stable    core18
        gnome             core22
        gnome-3-28        core18
        gnome-3-34        core18
        gnome-3-38        core20
        kde-neon          core18, core20
        ros1-noetic       core20
        ros2-foxy         core20, core22"""
        )
    )
