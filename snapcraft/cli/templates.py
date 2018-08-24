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
import pkgutil
import yaml
import sys
import textwrap
import tabulate

from ._options import get_project
from snapcraft.internal import lifecycle


@click.group()
def templatecli(**kwargs):
    pass


@templatecli.command()
def templates(**kwargs):
    """List available templates."""
    from snapcraft.internal import project_loader

    template_names = []
    for _, modname, _ in pkgutil.iter_modules(project_loader._templates.__path__):
        # Only add non-private modules/packages to the template list
        if not modname.startswith("_"):
            template_names.append(modname)

    if not template_names:
        click.echo("No templates supported")
        return

    templates = []
    for template_name in sorted(template_names):
        template = project_loader.find_template(template_name)
        template_info = collections.OrderedDict()
        template_info["Template name"] = template_name
        template_info["Supported bases"] = ", ".join(sorted(template.supported_bases()))
        templates.append(template_info)

    click.echo(tabulate.tabulate(templates, headers="keys"))


@templatecli.command()
@click.argument("name")
def template(name, **kwargs):
    """Show contents of template."""
    from snapcraft.internal import project_loader

    dummy_data = lifecycle.get_init_data()
    template_instance = project_loader.find_template(name)(dummy_data)

    app_snippet = template_instance.app_snippet()
    part_snippet = template_instance.part_snippet()
    parts = template_instance.parts()

    intro = "The {} template".format(name)
    if app_snippet:
        click.echo("{} adds the following to apps that use it:".format(intro))
        click.echo(
            textwrap.indent(
                yaml.safe_dump(app_snippet, default_flow_style=False), "    "
            )
        )
        intro = "It"

    if part_snippet:
        click.echo("{} adds the following to all parts:".format(intro))
        click.echo(
            textwrap.indent(
                yaml.safe_dump(part_snippet, default_flow_style=False), "    "
            )
        )
        intro = "It"

    if parts:
        click.echo("{} adds the following part definitions:".format(intro))
        click.echo(
            textwrap.indent(yaml.safe_dump(parts, default_flow_style=False), "    ")
        )


@templatecli.command("expand-templates")
def expand_templates(**kwargs):
    """Display snapcraft.yaml with all templates applied."""
    from snapcraft.internal import project_loader

    project = get_project(**kwargs)
    yaml_with_templates = project_loader.apply_templates(
        project.info.get_raw_snapcraft()
    )

    # Loading the config applied all the templates, so just dump it back out
    yaml.safe_dump(yaml_with_templates, stream=sys.stdout, default_flow_style=False)
