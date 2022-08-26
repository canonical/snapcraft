# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2022 Canonical Ltd.
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

"""Ubuntu Advantage token management."""

import contextlib
import json
import subprocess
from typing import Any, Dict, Iterator, List, Optional

from craft_cli import emit
from craft_parts.packages import Repository

from snapcraft import errors


class UATokenAttachError(errors.SnapcraftError):
    """Failure attaching UA token."""

    def __init__(self) -> None:
        super().__init__(
            "Error attaching UA token.",
            resolution="Make sure the UA token is correct and valid.",
        )


class UATokenDetachError(errors.SnapcraftError):
    """Failure attaching UA token."""

    def __init__(self) -> None:
        super().__init__("Error detaching UA token.")


class UAEnableServicesError(errors.SnapcraftError):
    """Failure attaching UA token."""

    def __init__(self) -> None:
        super().__init__(
            "Error enabling UA services.",
            resolution=("Make sure the requested services are valid and available."),
        )


def _install_ua_tools() -> None:
    """Ensure UA tools are installed."""
    if not Repository.is_package_installed("ubuntu-advantage-tools"):
        Repository.install_packages(
            ["ubuntu-advantage-tools"], refresh_package_cache=True
        )


def _attach(ua_token: str) -> None:
    """Attach UA token (ua will update package lists)."""
    try:
        with emit.open_stream("Attach UA token") as stream:
            subprocess.check_call(
                ["ua", "attach", ua_token], stdout=stream, stderr=stream
            )
    except subprocess.CalledProcessError as error:
        raise UATokenAttachError from error


def _enable_services(services: List[str]) -> None:
    """Enable the specified UA services."""
    try:
        with emit.open_stream("Enable UA services") as stream:
            subprocess.check_call(
                ["ua", "enable", *services, "--beta"], stdout=stream, stderr=stream
            )
    except subprocess.CalledProcessError as error:
        raise UAEnableServicesError from error


def _detach() -> None:
    """Detach UA token (ua will update package lists)."""
    try:
        with emit.open_stream("Detach UA token") as stream:
            subprocess.check_call(
                ["ua", "detach", "--assume-yes"], stdout=stream, stderr=stream
            )
    except subprocess.CalledProcessError as error:
        raise UATokenDetachError from error


def _is_attached() -> bool:
    return _status()["attached"]


def _status() -> Dict[str, Any]:
    stdout = subprocess.check_output(["ua", "status", "--format", "json"])
    return json.loads(stdout)


@contextlib.contextmanager
def ua_manager(
    ua_token: Optional[str], *, services: Optional[List[str]]
) -> Iterator[None]:
    """Attach and detach UA token as required.

    Uses try/finally to ensure that token is detached on error.

    If ua_token is None, neither attach / detach will occur and the context
    manager is effectively a noop.

    If ua_token is specified and the system is already attached, log a warning
    that we are refusing to configure specified token, but otherwise resume
    normally.

    :param ua_token: Optional ua_token.
    :param services: Optional ua services to enable.
    """
    ua_needs_detach = False

    if ua_token is not None:
        # Make sure the tools are installed first.
        _install_ua_tools()

        if _is_attached():
            emit.progress(
                "Refusing to attach specified UA token as system is already attached.",
                permanent=True,
            )
        else:
            ua_needs_detach = True
            emit.progress("Attaching specified UA token...")
            _attach(ua_token)
            emit.progress("Attached specified UA token", permanent=True)

    try:
        if services:
            emit.progress("Enabling UA services...")
            _enable_services(services)
            emit.progress(f"Enabled UA services: {' '.join(services)}")

        yield
    finally:
        if ua_needs_detach:
            emit.progress("Detaching specified UA token...")
            _detach()
            emit.progress("Detached specified UA token", permanent=True)
