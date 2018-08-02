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
import os
import yaml
import sys

from ._options import get_project
from snapcraft.internal import common, project_loader


@click.group()
def templatecli(**kwargs):
    pass


@templatecli.command()
def templates(**kwargs):
    """List available templates."""

    template_names = os.listdir(common.get_templatesdir())
    if not template_names:
        click.echo("No templates supported")
        return

    # Wrap the output depending on terminal size
    width = common.get_terminal_width()
    for line in common.format_output_in_columns(
        sorted(template_names), max_width=width
    ):
        click.echo(line)


@templatecli.command()
@click.argument("name")
def template(name, **kwargs):
    """Show contents of template."""

    # Dump the template file itself, thus preserving all comments, etc.
    with open(project_loader.template_yaml_path(name), "r") as f:
        click.echo(f.read())


@templatecli.command("expand-templates")
def expand_templates(**kwargs):
    """Display snapcraft.yaml with all templates applied."""

    project = get_project(**kwargs)
    yaml_with_templates = project_loader.apply_templates(
        project.info.get_raw_snapcraft()
    )

    # Loading the config applied all the templates, so just dump it back out
    yaml.safe_dump(yaml_with_templates, stream=sys.stdout, default_flow_style=False)
