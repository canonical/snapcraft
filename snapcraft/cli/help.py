# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
from snapcraft.internal import sources


_TOPICS = {
    'sources': sources,
    'plugins': snapcraft,
}


@click.group()
def helpcli():
    """Help commands"""
    pass


@helpcli.command('help')
@click.argument('topic', metavar='<topic>', required=False)
@click.option('--devel', is_flag=True,
              help='Show more details for snapcraft developers')
@click.pass_context
def help_command(ctx, topic, devel):
    """Obtain help for a certain plugin or topic.

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
    """
    if not topic:
        click.echo(ctx.parent.get_help())
        click.echo(dedent("""\

            To get additional help, run:
                snapcraft help topics
                snapcraft help <topic>
                snapcraft help <plugin-name>
        """))
    elif topic == 'topics':
        for key in _TOPICS:
            click.echo(key)
    elif topic in _TOPICS:
        _topic_help(topic, devel)
    else:
        try:
            _module_help(topic, devel)
        except ImportError:
            click.echo(dedent("""\
    The argument to help is not a plugin name nor topic, to see the
    available topics run:

        snapcraft help topics

    to see the available plugins run:

        snapcraft list-plugins
    """))
            sys.exit(1)


def _topic_help(module_name, devel):
    if devel:
        help(_TOPICS[module_name])
    else:
        click.echo(_TOPICS[module_name].__doc__)


def _module_help(module_name, devel):
    module = importlib.import_module(
        'snapcraft.plugins.{}'.format(module_name.replace('-', '_')))
    if module.__doc__ and devel:
        help(module)
    elif module.__doc__:
        click.echo(module.__doc__)
    else:
        click.echo('The plugin has no documentation')
