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
def extensioncli(**kwargs):
    pass


@extensioncli.command("list-extensions")
def list_extensions(**kwargs):
    """List available extensions.

    This command has an alias of `extensions`.
    """
    from snapcraft.internal import project_loader

    extension_names = []
    for _, modname, _ in pkgutil.iter_modules(project_loader._extensions.__path__):
        # Only add non-private modules/packages to the extension list
        if not modname.startswith("_"):
            extension_names.append(modname)

    if not extension_names:
        click.echo("No extensions supported")
        return

    extensions = []
    for extension_name in sorted(extension_names):
        extension = project_loader.find_extension(extension_name)
        extension_info = collections.OrderedDict()
        extension_info["Extension name"] = extension_name
        extension_info["Supported bases"] = ", ".join(sorted(extension.supported_bases))
        extensions.append(extension_info)

    click.echo(tabulate.tabulate(extensions, headers="keys"))


@extensioncli.command()
@click.argument("name")
def extension(name, **kwargs):
    """Show contents of extension."""
    from snapcraft.internal import project_loader

    dummy_data = lifecycle.get_init_data()
    extension_instance = project_loader.find_extension(name)(dummy_data)

    app_snippet = extension_instance.app_snippet
    part_snippet = extension_instance.part_snippet
    parts = extension_instance.parts

    intro = "The {} extension".format(name)
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


@extensioncli.command("expand-extensions")
def expand_extensions(**kwargs):
    """Display snapcraft.yaml with all extensions applied."""
    from snapcraft.internal import project_loader

    project = get_project(**kwargs)
    yaml_with_extensions = project_loader.apply_extensions(
        project.info.get_raw_snapcraft()
    )

    # Loading the config applied all the extensions, so just dump it back out
    yaml.safe_dump(yaml_with_extensions, stream=sys.stdout, default_flow_style=False)
