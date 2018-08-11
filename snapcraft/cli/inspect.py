# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
from datetime import datetime
import json
import logging
from tabulate import tabulate

from snapcraft import formatting_utils
from snapcraft.internal import errors, project_loader
from snapcraft.internal.project_loader import inspection
from ._options import get_project
from . import echo


@click.group(context_settings={})
@click.pass_context
def inspectcli(ctx):
    pass


@inspectcli.command()
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
        providing_parts = inspection.provides(
            provides, config.project, config.all_parts
        )
        providing_part_names = sorted([p.name for p in providing_parts])
        if use_json:
            print(
                json.dumps(
                    {"path": provides, "parts": providing_part_names},
                    sort_keys=True,
                    indent=4,
                )
            )
        else:
            echo.info(
                "This path was provided by the following {}:".format(
                    formatting_utils.pluralize(providing_part_names, "part", "parts")
                )
            )
            echo.info("\n".join(providing_part_names))

    elif latest_step:
        part, step, timestamp = inspection.latest_step(config.all_parts)
        directory = part.working_directory_for_step(step)
        if use_json:
            print(
                json.dumps(
                    {
                        "part": part.name,
                        "step": step.name,
                        "directory": directory,
                        "timestamp": timestamp,
                    },
                    sort_keys=True,
                    indent=4,
                )
            )
        else:
            echo.info(
                "The latest step that was run is the {!r} step of the {!r} part, "
                "which ran at {}. The working directory for this step is {!r}".format(
                    step.name,
                    part.name,
                    datetime.fromtimestamp(timestamp).strftime("%c"),
                    directory,
                )
            )

    else:
        summary = inspection.lifecycle_status(config)
        if use_json:
            summary_dict = dict()
            for part_summary in summary:
                part_name = part_summary.pop("part")
                summary_dict[part_name] = part_summary
            print(json.dumps(summary_dict, sort_keys=True, indent=4))
        else:
            print(tabulate(summary, headers="keys"))
