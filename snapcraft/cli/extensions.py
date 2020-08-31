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

import collections
import inspect
import os
import sys

import click
import tabulate

from snapcraft import yaml_utils
from snapcraft.internal import project_loader

from ._options import get_project


@click.group()
def extensioncli(**kwargs):
    pass


@extensioncli.command("list-extensions")
def list_extensions(**kwargs):
    """List available extensions.

    This command has an alias of `extensions`.
    """

    extension_names = project_loader.supported_extension_names()

    if not extension_names:
        click.echo("No extensions supported")
        return

    extensions = []
    for extension_name in sorted(extension_names):
        extension = project_loader.find_extension(extension_name)
        extension_info = collections.OrderedDict()
        extension_info["Extension name"] = extension_name.replace("_", "-")
        extension_info["Supported bases"] = ", ".join(
            sorted(extension.get_supported_bases())
        )
        extensions.append(extension_info)

    click.echo(tabulate.tabulate(extensions, headers="keys"))


@extensioncli.command()
@click.pass_context
@click.argument("name")
def extension(ctx, name, **kwargs):
    """Show contents of extension."""

    extension_cls = project_loader.find_extension(name)

    # Not using inspect.getdoc here since it'll fall back to the base class
    docstring = extension_cls.__doc__
    if not docstring:
        raise project_loader.errors.ExtensionMissingDocumentationError(name)

    formatter = ctx.make_formatter()
    formatter.write_text(inspect.cleandoc(docstring))
    click.echo(formatter.getvalue().rstrip("\n"))


@extensioncli.command("expand-extensions")
@click.option(
    "--enable-experimental-extensions",
    is_flag=True,
    envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS",
)
def expand_extensions(**kwargs):
    """Display snapcraft.yaml with all extensions applied."""
    if kwargs.get("enable_experimental_extensions"):
        os.environ["SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS"] = "True"

    project = get_project(**kwargs)
    yaml_with_extensions = project_loader.apply_extensions(
        project.info.get_raw_snapcraft()
    )

    # Loading the config applied all the extensions, so just dump it back out
    yaml_utils.dump(yaml_with_extensions, stream=sys.stdout)
