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
import os

from snapcraft.internal import deprecations, lifecycle, lxd, project_loader
from ._options import add_build_options, get_project_options
from . import echo
from . import env


def _execute(command, parts, **kwargs):
    project_options = get_project_options(**kwargs)
    build_environment = env.BuilderEnvironmentConfig()
    if build_environment.is_host:
        lifecycle.execute(command, project_options, parts)
    else:
        lifecycle.containerbuild(command, project_options, parts)
    return project_options


@click.group()
@add_build_options()
@click.pass_context
def lifecyclecli(ctx, **kwargs):
    pass


@lifecyclecli.command()
def init():
    """Initialize a snapcraft project."""
    snapcraft_yaml_path = lifecycle.init()
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

    If you want to snap a directory, you should use the pack command
    instead.
    """
    if directory:
        deprecations.handle_deprecation_notice('dn6')

    project_options = get_project_options(**kwargs)
    build_environment = env.BuilderEnvironmentConfig()
    if build_environment.is_host:
        snap_name = lifecycle.snap(
            project_options, directory=directory, output=output)
        echo.info('Snapped {}'.format(snap_name))
    else:
        lifecycle.containerbuild('snap', project_options, output, directory)


@lifecyclecli.command()
@click.argument('directory')
@click.option('--output', '-o', help='path to the resulting snap.')
def pack(directory, output, **kwargs):
    """Create a snap from a directory holding a valid snap.

    The layout of <directory> should contain a valid meta/snap.yaml in
    order to be a valid snap.

    \b
    Examples:
        snapcraft pack my-snap-directory
        snapcraft pack my-snap-directory --output renamed-snap.snap

    """
    snap_name = lifecycle.pack(directory, output)
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
    build_environment = env.BuilderEnvironmentConfig()
    if build_environment.is_host:
        step = step or 'pull'
        if step == 'strip':
            echo.warning('DEPRECATED: Use `prime` instead of `strip` '
                         'as the step to clean')
            step = 'prime'
        lifecycle.clean(project_options, parts, step)
    else:
        config = project_loader.load_config(project_options)
        lxd.Project(project_options=project_options,
                    output=None, source=os.path.curdir,
                    metadata=config.get_metadata()).clean(parts, step)


@lifecyclecli.command()
@add_build_options()
@click.option('--remote', metavar='<remote>',
              help='Use a specific lxd remote instead of a local container.')
@click.option('--debug', is_flag=True,
              help='Shells into the environment if the build fails.')
def cleanbuild(remote, debug, **kwargs):
    """Create a snap using a clean environment managed by a build provider.

    \b
    Examples:
        snapcraft cleanbuild

    The cleanbuild command requires a properly setup lxd environment that
    can connect to external networks. Refer to the "Ubuntu Desktop and
    Ubuntu Server" section on
    https://linuxcontainers.org/lxd/getting-started-cli
    to get started.

    If using a remote, a prior setup is required which is described on:
    https://linuxcontainers.org/lxd/getting-started-cli/#multiple-hosts
    """
    # cleanbuild is a special snow flake, while all the other commands
    # would work with the host as the build_provider it makes little
    # sense in this scenario.
    build_environment = env.BuilderEnvironmentConfig(
        default='lxd', additional_providers=['multipass'])
    project_options = get_project_options(**kwargs, debug=debug)
    lifecycle.cleanbuild(project=project_options,
                         echoer=echo,
                         remote=remote,
                         build_environment=build_environment)


if __name__ == '__main__':
    lifecyclecli.main()
