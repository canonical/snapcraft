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

import snapcraft

SNAPCRAFT_VERSION_TEMPLATE = "snapcraft, version %(version)s"


@click.group()
def versioncli():
    """Version commands"""
    pass


@versioncli.command("version")
def version():
    """Obtain snapcraft's version number.

    Examples:
        snapcraft version
        snapcraft --version
    """
    click.echo(SNAPCRAFT_VERSION_TEMPLATE % {"version": snapcraft.__version__})
