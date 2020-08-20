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
import pathlib
import sys

import click

import snapcraft
from snapcraft.internal import log

from ._command_group import SnapcraftGroup
from ._errors import exception_handler
from ._options import add_provider_options
from .assertions import assertionscli
from .containers import containerscli
from .discovery import discoverycli
from .extensions import extensioncli
from .help import helpcli
from .legacy import legacycli
from .lifecycle import lifecyclecli
from .remote import remotecli
from .store import storecli
from .version import SNAPCRAFT_VERSION_TEMPLATE, versioncli

logger = logging.getLogger(__name__)

command_groups = [
    storecli,
    assertionscli,
    containerscli,
    discoverycli,
    helpcli,
    legacycli,
    lifecyclecli,
    extensioncli,
    versioncli,
    remotecli,
]


def configure_requests_ca() -> None:
    """Force `requests` library to use host certificates, if found.

    Otherwise it will use the ones provided by `certifi`, which may be
    out of date or missing user-installed certificates.

    The bundle paths below should cover the majority of the Linux
    ecosystem, but OS X and Windows are currently unhandled which results
    in default behavior (use the certifi certificates).
    """
    ca_bundle_paths = [
        p
        for p in [
            pathlib.Path("/etc/ssl/certs/ca-certificates.crt"),
            pathlib.Path("/etc/pki/tls/certs/ca-bundle.crt"),
        ]
        if p.exists()
    ]

    if "REQUESTS_CA_BUNDLE" not in os.environ and ca_bundle_paths:
        os.environ["REQUESTS_CA_BUNDLE"] = str(ca_bundle_paths[0])

    if "REQUESTS_CA_BUNDLE" in os.environ:
        logger.debug(
            f"configure_requests_ca: using {os.environ['REQUESTS_CA_BUNDLE']!r} for certificate verification"
        )


@click.group(
    cls=SnapcraftGroup,
    invoke_without_command=True,
    context_settings=dict(help_option_names=["-h", "--help"]),
)
@click.version_option(
    message=SNAPCRAFT_VERSION_TEMPLATE, version=snapcraft.__version__  # type: ignore
)
@click.pass_context
@add_provider_options(hidden=True)
@click.option("--debug", "-d", is_flag=True)
def run(ctx, debug, catch_exceptions=False, **kwargs):
    """Snapcraft is a delightful packaging tool."""

    is_snapcraft_developer_debug = kwargs["enable_developer_debug"]
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

    # Configure `requests` library to use system certificates.
    configure_requests_ca()

    # Payload information about argv
    ctx.obj = dict(argv=sys.argv)

    # The default command
    if not ctx.invoked_subcommand:
        snap_command = lifecyclecli.commands["snap"]
        # Fill in the context with default values for the snap command.
        if "directory" not in ctx.params:
            ctx.params["directory"] = None
        if "output" not in ctx.params:
            ctx.params["output"] = None
        snap_command.invoke(ctx)


# This would be much easier if they were subcommands
for command_group in command_groups:
    for command in command_group.commands:
        run.add_command(command_group.commands[command])
