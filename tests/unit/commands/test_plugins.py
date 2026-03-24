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

import argparse
import re
import textwrap

import craft_parts.plugins
import pytest

from snapcraft import commands, const, errors

############
# Fixtures #
############


@pytest.fixture(autouse=True)
def fake_registered_plugins(mocker):
    fake_plugins = mocker.patch(
        "snapcraft.commands.plugins.get_registered_plugins",
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
def test_registered_plugins_default(emitter, fake_app_config):
    """Default to core24."""
    commands.PluginsCommand(fake_app_config).run(argparse.Namespace(base=None))

    emitter.assert_message(
        textwrap.dedent(
            """\
            Displaying plugins available for 'core24'
            bar
            foo"""
        )
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_project(base, emitter, snapcraft_yaml, fake_app_config):
    """Use the project's base."""
    if base in (b for b in const.UnstableBase):
        snapcraft_yaml(base=base, build_base="devel", grade="devel")
    else:
        snapcraft_yaml(base=base)

    commands.PluginsCommand(fake_app_config).run(argparse.Namespace(base=None))

    emitter.assert_message(
        textwrap.dedent(
            f"""\
            Displaying plugins available to the current base {base!r} project
            bar
            foo"""
        )
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_base_option(base, emitter, snapcraft_yaml, fake_app_config):
    """The base cli option should override the project's base."""
    snapcraft_yaml(base="core20")

    commands.PluginsCommand(fake_app_config).run(argparse.Namespace(base=base))

    emitter.assert_message(
        textwrap.dedent(
            f"""\
            Displaying plugins available for {base!r}
            bar
            foo"""
        )
    )


@pytest.mark.parametrize("base", const.ESM_BASES)
@pytest.mark.usefixtures("new_dir")
def test_registered_plugins_esm_base_error(base, snapcraft_yaml, fake_app_config):
    """Error when listing plugins for an outdated base."""
    snapcraft_yaml(base=base)
    expected = re.escape(f"{base!r} is not supported on this version of Snapcraft.")

    with pytest.raises(errors.MaintenanceBase, match=expected):
        commands.PluginsCommand(fake_app_config).run(argparse.Namespace(base=None))


@pytest.mark.usefixtures("new_dir")
def test_list_plugins_error(fake_app_config):
    """Error on removed 'list-plugins' command."""
    expected = re.escape("The 'list-plugins' command was renamed to 'plugins'.")

    with pytest.raises(errors.RemovedCommand, match=expected):
        commands.ListPluginsCommand(fake_app_config).run(argparse.Namespace(base=None))
