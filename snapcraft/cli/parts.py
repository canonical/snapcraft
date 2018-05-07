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

from snapcraft import formatting_utils
from snapcraft.internal import errors, remote_parts, lifecycle, project_loader
from ._options import get_project_options
from . import env, echo


@click.group(context_settings={})
@click.pass_context
def partscli(ctx):
    pass


@partscli.command()
@click.pass_context
def update(ctx, **kwargs):
    """Updates the parts listing from the cloud."""
    # Update in the container so that it will use the parts at build time
    build_environment = env.BuilderEnvironmentConfig()
    if not build_environment.is_host:
        project_options = get_project_options(**kwargs)
        lifecycle.containerbuild('update', project_options)

    # Parts can be defined and searched from any folder on the host, so
    # regardless of using containers we always update these as well
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


# Implemented as a generator since loading up the state could be heavy
def _part_states_for_step(step, parts_config):
    for part in parts_config.all_parts:
        state = part.get_state(step)
        if state:
            yield (part.name, state)


@partscli.command()
@click.argument('path', metavar='<path>')
def provides(*, path, **kwargs):
    """Show the part that provided <path>.

    <path> can be an absolute path, or relative to the current working
    directory. It must point to a file either in the staging or the priming
    area.
    """
    # First of all, ensure the file actually exists before doing any work
    if not os.path.exists(path):
        raise errors.NoSuchFileError(path)

    project = get_project_options(**kwargs)
    config = project_loader.load_config(project)

    # Convert file path into absolute path
    absolute_file_path = os.path.abspath(path)

    # Which step are we operating on? We'll know by where the file_path is:
    # the staging area, or the priming area?
    if absolute_file_path.startswith(project.stage_dir):
        step = 'stage'
        relative_file_path = os.path.relpath(
            absolute_file_path, start=project.stage_dir)
    elif absolute_file_path.startswith(project.prime_dir):
        step = 'prime'
        relative_file_path = os.path.relpath(
            absolute_file_path, start=project.prime_dir)
    else:
        raise errors.ProvidesInvalidFilePathError(path)

    is_dir = os.path.isdir(absolute_file_path)
    is_file = os.path.isfile(absolute_file_path)

    providing_parts = set()  # type: Set[str]
    for part_name, state in _part_states_for_step(step, config.parts):
        if is_dir and relative_file_path in state.directories:
                providing_parts.add(part_name)
        elif is_file and relative_file_path in state.files:
                providing_parts.add(part_name)

    if not providing_parts:
        raise errors.UntrackedFileError(path)

    echo.info('This path was provided by the following {}:'.format(
        formatting_utils.pluralize(providing_parts, 'part', 'parts')))
    echo.info('\n'.join(providing_parts))
