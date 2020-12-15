# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2020 Canonical Ltd
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
import importlib
import sys
from textwrap import dedent

import click

import snapcraft
from snapcraft.internal import errors, sources
from snapcraft.project import errors as project_errors

from . import echo
from ._options import get_project

_TOPICS = {"sources": sources, "plugins": snapcraft}


@click.group()
def helpcli():
    """Help commands"""
    pass


@helpcli.command("help")
@click.argument("topic", metavar="<topic>", required=False)
@click.option(
    "--devel", is_flag=True, help="Show more details for snapcraft developers"
)
@click.option(
    "--base",
    help="Show help for specific base",
    type=click.Choice(["core", "core16", "core18", "core20"]),
)
@click.pass_context
def help_command(ctx, topic, devel, base):
    """Obtain help for a certain topic, plugin or command.

    The <topic> can either be a plugin name or one of:

    \b
        - topics
        - plugins
        - sources

    \b
    Examples:
        snapcraft help topics
        snapcraft help plugins
        snapcraft help sources
        snapcraft help go
        snapcraft help go --base core18
        snapcraft help build
    """
    if not topic:
        click.echo(ctx.parent.get_help())
        click.echo(
            dedent(
                """\

            For more help, use:
                snapcraft help topics
                snapcraft help <topic>
                snapcraft help <plugin-name>
                snapcraft help <command-name>
        """
            )
        )
    elif topic in ctx.parent.command.commands:
        ctx.info_name = topic
        click.echo(ctx.parent.command.commands[topic].get_help(ctx))
    elif topic == "topics":
        for key in _TOPICS:
            click.echo(key)
    elif topic in _TOPICS:
        _topic_help(topic, devel)
    else:
        try:
            _module_help(topic, devel, base)
        except ImportError:
            # 10 is the limit which determines ellipsis is needed
            if len(topic) > 10:
                topic = "{}...".format(topic[:10])
            echo.wrapped(
                dedent(
                    """\
    There is no help topic, plugin or command {!r}. Try:

    For topics:

        snapcraft help topics

    For valid plugins:

        snapcraft list-plugins

    Or for general help:

        snapcraft help
    """
                ).format(topic)
            )
            sys.exit(1)


def _topic_help(module_name, devel):
    if devel:
        help(_TOPICS[module_name])
    else:
        click.echo(_TOPICS[module_name].__doc__)


def _module_help(plugin_name: str, devel: bool, base: str):
    module_name = plugin_name.replace("-", "_")

    if base is None:
        try:
            base = get_project()._snap_meta.get_build_base()
        except (errors.ProjectNotFoundError, project_errors.MissingSnapcraftYamlError):
            base = "core20"

    if base == "core20":
        plugin_version = "v2"
    else:
        plugin_version = "v1"

    module = importlib.import_module(
        f"snapcraft.plugins.{plugin_version}.{module_name}"
    )
    if module.__doc__ and devel:
        help(module)
    elif module.__doc__:
        click.echo_via_pager(
            f"Displaying help for the {plugin_name!r} plugin for {base!r}.\n\n"
            + module.__doc__
        )
    else:
        click.echo("The plugin has no documentation")
