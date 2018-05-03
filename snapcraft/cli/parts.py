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
import contextlib
from datetime import datetime
import json
import logging
import os
from tabulate import tabulate
from typing import Set

from snapcraft import formatting_utils
from snapcraft.internal import errors, project_loader, remote_parts, lifecycle, steps
from ._options import get_project
from . import echo, env


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
        project = get_project(**kwargs)
        project_config = project_loader.load_config(project)
        lifecycle.containerbuild("update", project_config)

    # Parts can be defined and searched from any folder on the host, so
    # regardless of using containers we always update these as well
    remote_parts.update()


@partscli.command()
@click.pass_context
@click.argument("part", metavar="<part>")
def define(ctx, part):
    """Shows the definition for the cloud part.

    \b
    Examples:
        snapcraft define my-part1

    """
    remote_parts.define(part)


@partscli.command()
@click.pass_context
@click.argument("query", nargs=-1, metavar="<query>...")
def search(ctx, query):
    """Searches the remote parts cache for matching parts.

    \b
    Examples:
        snapcraft search go

    """
    remote_parts.search(" ".join(query))


# Implemented as a generator since loading up the state could be heavy
def _part_states_for_step(step, parts_config):
    for part in parts_config.all_parts:
        state = part.get_state(step)
        if state:
            yield (part.name, state)


@partscli.command()
@click.option(
    "--provides",
    metavar="<path>",
    help=(
        "Show the part that provided <path>. <path> can be relative or absolute, and "
        "must be pointing within the staging or priming area."
    ),
)
@click.option(
    "--latest-step", is_flag=True, help="Show the most recently-completed step"
)
@click.option("--json", "use_json", is_flag=True, help="Print the result in json")
def inspect(*, provides: str, latest_step: bool, use_json: bool, **kwargs):
    """Inspect the current state of the project.

    By default, this command will print a tabulated summary of the current state of the
    project.
    """
    count = 0
    for option in (provides, latest_step):
        if option:
            count += 1

    if count > 1:
        raise errors.SnapcraftEnvironmentError(
            "Only one of --provides or --latest-step may be supplied"
        )

    # Silence the plugin loader logger
    logging.getLogger("snapcraft.internal.pluginhandler._plugin_loader").setLevel(
        logging.ERROR
    )
    project = get_project(**kwargs)
    config = project_loader.load_config(project)

    if provides:
        providing_parts = _provides(provides, config)
        if use_json:
            print(
                json.dumps(
                    {"path": provides, "parts": list(providing_parts)},
                    sort_keys=True,
                    indent=4,
                )
            )
        else:
            echo.info(
                "This path was provided by the following {}:".format(
                    formatting_utils.pluralize(providing_parts, "part", "parts")
                )
            )
            echo.info("\n".join(providing_parts))

    elif latest_step:
        part_name, step, timestamp = _latest_step(config)
        if use_json:
            print(
                json.dumps(
                    {"part": part_name, "step": step.name, "timestamp": timestamp},
                    sort_keys=True,
                    indent=4,
                )
            )
        else:
            echo.info(
                "The latest step that was run is the {!r} step of the {!r} part, "
                "which ran at {}".format(
                    step.name,
                    part_name,
                    datetime.fromtimestamp(timestamp).strftime("%c"),
                )
            )

    else:
        summary = _lifecycle_status(config)
        if use_json:
            summary_dict = dict()
            for part_summary in summary:
                part_name = part_summary.pop("part")
                summary_dict[part_name] = part_summary
            print(json.dumps(summary_dict, sort_keys=True, indent=4))
        else:
            print(tabulate(summary, headers="keys"))


def _provides(path, config) -> Set[str]:
    # First of all, ensure the file actually exists before doing any work
    if not os.path.exists(path):
        raise errors.NoSuchFileError(path)

    # Convert file path into absolute path
    absolute_file_path = os.path.abspath(path)

    # Which step are we operating on? We'll know by where the file_path is:
    # the staging area, or the priming area?
    if absolute_file_path.startswith(config.project.stage_dir):
        step = steps.STAGE
        relative_file_path = os.path.relpath(
            absolute_file_path, start=config.project.stage_dir
        )
    elif absolute_file_path.startswith(config.project.prime_dir):
        step = steps.PRIME
        relative_file_path = os.path.relpath(
            absolute_file_path, start=config.project.prime_dir
        )
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

    return providing_parts


def _latest_step(config):
    latest_part = None
    latest_step = None
    latest_timestamp = 0
    for part in config.all_parts:
        for step in steps.STEPS:
            with contextlib.suppress(errors.StepHasNotRunError):
                timestamp = part.step_timestamp(step)
                if latest_timestamp < timestamp:
                    latest_part = part.name
                    latest_step = step
                    latest_timestamp = timestamp

    return (latest_part, latest_step, latest_timestamp)


def _lifecycle_status(config):
    cache = lifecycle.StatusCache(config)

    summary = []
    for part in config.all_parts:
        part_summary = {"part": part.name}
        for step in steps.STEPS:
            part_summary[step.name] = None
            if cache.has_step_run(part, step):
                part_summary[step.name] = "complete"

            dirty_report = cache.get_dirty_report(part, step)
            if dirty_report:
                part_summary[step.name] = "dirty ({})".format(
                    dirty_report.get_summary()
                )

            outdated_report = cache.get_outdated_report(part, step)
            if outdated_report:
                part_summary[step.name] = "outdated ({})".format(
                    outdated_report.get_summary()
                )
        summary.append(part_summary)
    return summary
