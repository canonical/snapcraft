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
import click
from snapcraft.internal import remote_parts


@click.group(context_settings={})
@click.pass_context
def partscli(ctx):
    pass


@partscli.command()
@click.pass_context
def update(ctx):
    """Updates the parts listing from the cloud."""
    remote_parts.update()


@partscli.command()
@click.pass_context
@click.argument('part', metavar='<part>')
def define(ctx, part):
    """Shows the definition for the cloud part.

    \b
    Examples:
        snapcraft define my-part1

    """
    remote_parts.define(part)


@partscli.command()
@click.pass_context
@click.argument('query', nargs=-1, metavar='<query>...')
def search(ctx, query):
    """Searches the remote parts cache for matching parts.

    \b
    Examples:
        snapcraft search go

    """
    remote_parts.search(' '.join(query))
