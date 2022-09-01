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
import textwrap

import craft_parts.plugins
import pytest

from snapcraft import commands, errors

############
# Fixtures #
############


@pytest.fixture(autouse=True)
def fake_registered_plugins(mocker):
    fake_plugins = mocker.patch(
        "snapcraft.commands.discovery.get_registered_plugins",
        autospec=True,
        return_value={
            "bar": craft_parts.plugins.Plugin,
            "foo": craft_parts.plugins.Plugin,
        },
    )
    return fake_plugins


###################
# Plugins Command #
###################


@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_default(emitter):
    cmd = commands.ListPluginsCommand(None)

    cmd.run(argparse.Namespace(base=None))

    emitter.assert_message(
        textwrap.dedent(
            """\
            Displaying plugins available for 'core22'
            bar
            foo"""
        )
    )


@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_project(emitter, snapcraft_yaml):
    snapcraft_yaml(base="core22")

    cmd = commands.ListPluginsCommand(None)

    cmd.run(argparse.Namespace(base=None))

    emitter.assert_message(
        textwrap.dedent(
            """\
            Displaying plugins available to the current base 'core22' project
            bar
            foo"""
        )
    )


@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_base_option(emitter, snapcraft_yaml):
    snapcraft_yaml(base="core20")

    cmd = commands.ListPluginsCommand(None)

    cmd.run(argparse.Namespace(base="core22"))

    emitter.assert_message(
        textwrap.dedent(
            """\
            Displaying plugins available for 'core22'
            bar
            foo"""
        )
    )


@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_legacy_project(emitter, snapcraft_yaml):
    snapcraft_yaml(base="core20")

    cmd = commands.ListPluginsCommand(None)

    with pytest.raises(errors.LegacyFallback):
        cmd.run(argparse.Namespace(base=None))
