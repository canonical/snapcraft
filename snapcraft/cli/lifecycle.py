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
import sys

import click

from snapcraft.internal import errors, lifecycle
from ._options import add_build_options, get_project_options
from . import echo
from . import env


def _execute(command, parts, **kwargs):
    project_options = get_project_options(**kwargs)

    if env.is_containerbuild():
        lifecycle.containerbuild(command, project_options, parts)
    else:
        try:
            lifecycle.execute(command, project_options, parts)
        except Exception as e:
            echo.error(e)
            sys.exit(1)
    return project_options


@click.group()
@add_build_options()
@click.pass_context
def lifecyclecli(ctx, **kwargs):
    pass


@lifecyclecli.command()
def init():
    """Initialize a snapcraft project."""
    try:
        snapcraft_yaml_path = lifecycle.init()
    except errors.SnapcraftEnvironmentError as e:
        echo.error(e)
        sys.exit(1)
    echo.info('Created {}.'.format(snapcraft_yaml_path))
    echo.info(
        'Edit the file to your liking or run `snapcraft` to get started')


@lifecyclecli.command()
@click.pass_context
@add_build_options()
@click.argument('parts', nargs=-1, metavar='<part>...', required=False)
def pull(ctx, parts, **kwargs):
    """Download or retrieve artifacts defined for a part.

    \b
    Examples:
        snapcraft pull
        snapcraft pull my-part1 my-part2

    """
    _execute('pull', parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument('parts', nargs=-1, metavar='<part>...', required=False)
def build(parts, **kwargs):
    """Build artifacts defined for a part.

    \b
    Examples:
        snapcraft build
        snapcraft build my-part1 my-part2

    """
    _execute('build', parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument('parts', nargs=-1, metavar='<part>...', required=False)
def stage(parts, **kwargs):
    """Stage the part's built artifacts into the common staging area.

    \b
    Examples:
        snapcraft stage
        snapcraft stage my-part1 my-part2

    """
    _execute('stage', parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument('parts', nargs=-1, metavar='<part>...', required=False)
def prime(parts, **kwargs):
    """Final copy and preparation for the snap.

    \b
    Examples:
        snapcraft prime
        snapcraft prime my-part1 my-part2

    """
    _execute('prime', parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument('directory', required=False)
@click.option('--output', '-o', help='path to the resulting snap.')
def snap(directory, output, **kwargs):
    """Create a snap.

    \b
    Examples:
        snapcraft snap
        snapcraft snap --output renamed-snap.snap

    If you want to snap a directory, you should use the snap-dir command
    instead.
    """
    project_options = get_project_options(**kwargs)
    if env.is_containerbuild():
        lifecycle.containerbuild('snap', project_options, output, directory)
    else:
        try:
            snap_name = lifecycle.snap(
                project_options, directory=directory, output=output)
        except Exception as e:
            echo.error(e)
            sys.exit(1)
        echo.info('Snapped {}'.format(snap_name))


@lifecyclecli.command()
@add_build_options()
@click.argument('parts', nargs=-1, metavar='<part>...', required=False)
@click.option('--step', '-s',
              type=click.Choice(['pull', 'build', 'stage', 'prime', 'strip']),
              help='only clean the specified step and those that '
                   'depend on it.')
def clean(parts, step, **kwargs):
    """Remove content - cleans downloads, builds or install artifacts.

    \b
    Examples:
        snapcraft clean
        snapcraft clean my-part --step build
    """
    project_options = get_project_options(**kwargs)
    if env.is_containerbuild():
        step = step or 'pull'
        lifecycle.containerbuild('clean', project_options,
                                 args=['--step', step, *parts])
    else:
        if step == 'strip':
            echo.warning('DEPRECATED: Use `prime` instead of `strip` '
                         'as the step to clean')
            step = 'prime'
        try:
            lifecycle.clean(project_options, parts, step)
        except errors.SnapcraftEnvironmentError as e:
            echo.error(e)
            sys.exit(1)


@lifecyclecli.command()
@add_build_options()
@click.option('--remote', metavar='<remote>',
              help='Use a specific lxd remote instead of a local container.')
@click.option('--debug', is_flag=True,
              help='Shells into the environment if the build fails.')
def cleanbuild(remote, debug, **kwargs):
    """Create a snap using a clean environment managed by lxd.

    \b
    Examples:
        snapcraft cleanbuild
        snapcraft cleanbuild --output

    The cleanbuild command requires a properly setup lxd environment that
    can connect to external networks. Refer to the "Ubuntu Desktop and
    Ubuntu Server" section on
    https://linuxcontainers.org/lxd/getting-started-cli
    to get started.

    If using a remote, a prior setup is required which is described on:
    https://linuxcontainers.org/lxd/getting-started-cli/#multiple-hosts
    """
    project_options = get_project_options(**kwargs, debug=debug)
    try:
        lifecycle.cleanbuild(project_options, remote)
    except errors.SnapcraftEnvironmentError as e:
        echo.error(e)
        sys.exit(1)


if __name__ == '__main__':
    lifecyclecli.main()
