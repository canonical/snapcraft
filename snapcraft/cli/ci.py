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
import importlib

import click

from ._options import get_project


_SUPPORTED_CI_SYSTEMS = ("travis",)


@click.group()
def cicli():
    pass


@cicli.command("enable-ci")
@click.argument(
    "ci-system", metavar="<ci-system>", type=click.Choice(_SUPPORTED_CI_SYSTEMS)
)
@click.option(
    "--refresh",
    is_flag=True,
    help=("Refresh the macaroon required to be able to push and release to channels"),
)
def enableci(ci_system, refresh):
    """Enable continuous-integration systems to build and release snaps."""
    module = importlib.import_module("snapcraft.integrations.{}".format(ci_system))

    project = get_project()
    if refresh:
        module.refresh(project)
    else:
        if click.confirm(module.__doc__):
            module.enable(project)
