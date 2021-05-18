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

import contextlib
import json
import logging
import subprocess
from typing import Any, Dict, Iterator, Optional

from snapcraft.internal import repo

logger = logging.getLogger(__name__)


def _install_ua_tools() -> None:
    """Ensure UA tools are installed."""
    repo.Repo.install_build_packages(package_names=["ubuntu-advantage-tools"])


def _attach(ua_token: str) -> None:
    """Attach UA token (ua will update package lists)."""
    subprocess.check_call(["sudo", "ua", "attach", ua_token])


def _detach() -> None:
    """Detach UA token (ua will update package lists)."""
    subprocess.check_call(["sudo", "ua", "detach", "--assume-yes"])


def _is_attached() -> bool:
    return _status()["attached"]


def _status() -> Dict[str, Any]:
    stdout = subprocess.check_output(["sudo", "ua", "status", "--format", "json"])
    return json.loads(stdout)


@contextlib.contextmanager
def ua_manager(ua_token: Optional[str]) -> Iterator[None]:
    """Attach and detach UA token as required.

    Uses try/finally to ensure that token is detached on error.

    If ua_token is None, neither attach / detach will occur and the context
    manager is effectively a noop.

    If ua_token is specified and the system is already attached, log a warning
    that we are refusing to configure specified token, but otherwise resume
    normally.

    :param ua_token: Optional ua_token.
    """
    ua_needs_detach = False

    if ua_token is not None:
        # Make sure the tools are installed first.
        _install_ua_tools()

        if _is_attached():
            logger.warning(
                "Refusing to attach specified UA token as system is already attached. "
            )
        else:
            ua_needs_detach = True
            logger.info("Attaching specified UA token...")
            _attach(ua_token)

    try:
        yield
    finally:
        if ua_needs_detach:
            logger.info("Detaching specified UA token...")
            _detach()
