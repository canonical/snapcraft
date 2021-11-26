# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd.
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

"""Utilities for snapcraft."""

import distutils.util
import logging
import os
import pathlib
import sys
from collections import namedtuple
from typing import Optional

logger = logging.getLogger(__name__)

OSPlatform = namedtuple("OSPlatform", "system release machine")


def is_managed_mode():
    """Check if snapcraft is running in a managed environment."""
    managed_flag = os.getenv("SNAPCRAFT_MANAGED_MODE", "n")
    return distutils.util.strtobool(managed_flag) == 1


def get_managed_environment_home_path():
    """Path for home when running in managed environment."""
    return pathlib.Path("/root")


def get_managed_environment_project_path():
    """Path for project when running in managed environment."""
    return get_managed_environment_home_path() / "project"


def get_managed_environment_log_path():
    """Path for log when running in managed environment."""
    return pathlib.Path("/tmp/snapcraft.log")


def get_managed_environment_snap_channel() -> Optional[str]:
    """User-specified channel to use when installing Snapcraft snap from Snap Store.

    :returns: Channel string if specified, else None.
    """
    return os.getenv("SNAPCRAFT_INSTALL_SNAP_CHANNEL")


def confirm_with_user(prompt, default=False) -> bool:
    """Query user for yes/no answer.

    If stdin is not a tty, the default value is returned.

    If user returns an empty answer, the default value is returned.
    returns default value.

    :returns: True if answer starts with [yY], False if answer starts with [nN],
        otherwise the default.
    """
    if is_managed_mode():
        raise RuntimeError("confirmation not yet supported in managed-mode")

    if not sys.stdin.isatty():
        return default

    choices = " [Y/n]: " if default else " [y/N]: "

    reply = str(input(prompt + choices)).lower().strip()
    if reply and reply[0] == "y":
        return True

    if reply and reply[0] == "n":
        return False

    return default
