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
import functools
import logging
import os
import sys
from distutils import util

import click

import snapcraft
from snapcraft import project, yaml_utils
from snapcraft.internal import common, log
from .assertions import assertionscli
from .containers import containerscli
from .discovery import discoverycli
from .lifecycle import lifecyclecli
from .store import storecli
from .inspect import inspectcli
from .help import helpcli
from .extensions import extensioncli
from .version import versioncli, SNAPCRAFT_VERSION_TEMPLATE
from .ci import cicli
from ._command_group import SnapcraftGroup
from ._options import add_build_options
from ._errors import exception_handler


command_groups = [
    storecli,
    cicli,
    assertionscli,
    containerscli,
    discoverycli,
    helpcli,
    lifecyclecli,
    extensioncli,
    versioncli,
    inspectcli,
]


def _is_legacy_reexec() -> bool:
    if not os.path.isdir(common.get_legacy_snapcraft_dir()):
        return False

    try:
        # Early bootstrapping does not allow us to use the existing utilities we
        # have to manage this check.
        if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
            base_dir = os.path.expanduser(os.path.join("~", "project"))
        else:
            base_dir = None
        snapcraft_yaml_path = project.get_snapcraft_yaml(base_dir=base_dir)
        with open(snapcraft_yaml_path, "r") as f:
            data = yaml_utils.load(f)
        return data.get("base") is None
    except Exception:
        # If there are issues loading/parsing the YAML, just pass off to the current version
        # where the error should be properly handled
        return False


def _legacy_reexec() -> None:
    legacy_python = os.path.join(
        common.get_legacy_snapcraft_dir(), "usr", "bin", "python3"
    )
    legacy_snapcraft = os.path.join(
        common.get_legacy_snapcraft_dir(), "bin", "snapcraft"
    )

    os.execv(legacy_python, [legacy_python, legacy_snapcraft] + sys.argv[1:])


@click.group(cls=SnapcraftGroup, invoke_without_command=True)
@click.version_option(
    message=SNAPCRAFT_VERSION_TEMPLATE, version=snapcraft.__version__  # type: ignore
)
@click.pass_context
@add_build_options(hidden=True)
@click.option("--debug", "-d", is_flag=True)
def run(ctx, debug, catch_exceptions=False, **kwargs):
    """Snapcraft is a delightful packaging tool."""

    if _is_legacy_reexec():
        _legacy_reexec()

    # Debugging snapcraft itself is not tied to debugging a snapcraft project.
    try:
        is_snapcraft_developer_debug = util.strtobool(
            os.getenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "n")
        )
    except ValueError:
        is_snapcraft_developer_debug = False
    if is_snapcraft_developer_debug:
        log_level = logging.DEBUG
        click.echo(
            "Starting snapcraft {} from {}.".format(
                snapcraft.__version__, os.path.dirname(__file__)
            )
        )
    else:
        log_level = logging.INFO

    # Setup global exception handler (to be called for unhandled exceptions)
    sys.excepthook = functools.partial(
        exception_handler, debug=is_snapcraft_developer_debug
    )

    # In an ideal world, this logger setup would be replaced
    log.configure(log_level=log_level)
    # The default command
    if not ctx.invoked_subcommand:
        ctx.forward(lifecyclecli.commands["snap"])


# This would be much easier if they were subcommands
for command_group in command_groups:
    for command in command_group.commands:
        run.add_command(command_group.commands[command])
