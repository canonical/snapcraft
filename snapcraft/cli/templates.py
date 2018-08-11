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
import collections
import os
import yaml
import sys
import tabulate

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

    templates = []
    for template_name in sorted(template_names):
        template = project_loader.load_template(template_name)
        template_info = collections.OrderedDict()
        template_info["Template name"] = template_name
        template_info["Supported bases"] = ", ".join(sorted(template.keys()))
        templates.append(template_info)

    click.echo(tabulate.tabulate(templates, headers="keys"))


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
