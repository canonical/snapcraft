# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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
"""Callbacks used to modify craft-application commands."""
import argparse
import os
from typing import Any

import craft_cli
from craft_application import commands

from snapcraft import utils


def fill_lifecycle_parser(
    command: commands.ExtensibleCommand,  # pylint: disable=unused-argument
    parser: argparse.ArgumentParser,
) -> None:
    """Fill snapcraft-specific parameters for lifecycle commands."""
    parser.add_argument(
        "--use-lxd",
        action="store_true",
        help="Use LXD to build",
    )
    parser.add_argument(
        "--enable-manifest",
        action="store_true",
        default=utils.strtobool(os.getenv("SNAPCRAFT_BUILD_INFO", "n")),
        help="Generate snap manifest",
    )
    parser.add_argument(
        "--manifest-image-information",
        type=str,
        metavar="image-info",
        default=os.getenv("SNAPCRAFT_IMAGE_INFO"),
        help="Set snap manifest image-info",
    )
    parser.add_argument(
        "--bind-ssh",
        action="store_true",
        help="Bind ~/.ssh directory to local build instances",
    )
    if build_for := os.getenv("SNAPCRAFT_BUILD_FOR"):
        os.environ["CRAFT_BUILD_FOR"] = build_for
    parser.add_argument(
        "--http-proxy",
        type=str,
        default=os.getenv("http_proxy"),
        help="Set http proxy",
    )
    parser.add_argument(
        "--https-proxy",
        type=str,
        default=os.getenv("https_proxy"),
        help="Set https proxy",
    )
    parser.add_argument(
        "--ua-token",
        type=str,
        metavar="ua-token",
        default=os.getenv("SNAPCRAFT_UA_TOKEN"),
        help="Configure build environment with ESM using specified UA token",
    )
    parser.add_argument(
        "--enable-experimental-ua-services",
        action="store_true",
        help="Allow selection of UA services to enable.",
    )
    parser.add_argument(
        "--enable-experimental-plugins",
        action="store_true",
        default=os.getenv("SNAPCRAFT_ENABLE_EXPERIMENTAL_PLUGINS", "") != "",
        help="Allow using experimental (unstable) plugins.",
    )


def lifecycle_prologue(
    command: commands.base.ExtensibleCommand,  # pylint: disable=unused-argument
    parsed_args: argparse.Namespace,
    **kwargs: Any,  # noqa: ANN401  # pylint: disable=unused-argument
) -> None:
    """General prologue for any lifecycle command in Snapcraft."""
    if parsed_args.destructive_mode and parsed_args.use_lxd:
        raise craft_cli.ArgumentParsingError(
            "--destructive-mode cannot be used with --use-lxd"
        )
