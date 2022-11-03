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
from typing import Any, Dict, Iterator, List, Optional

from snapcraft_legacy.internal import repo

logger = logging.getLogger(__name__)


def _install_ua_tools() -> None:
    """Ensure UA tools are installed."""
    repo.Repo.install_build_packages(package_names=["ubuntu-advantage-tools"])


def _attach(ua_token: str) -> None:
    """Attach UA token (ua will update package lists)."""
    subprocess.check_call(["sudo", "ua", "attach", ua_token])


def _enable_services(services: List[str]) -> None:
    """Enable the specified UA services.

    If a service is already enabled, it will not be re-enabled because UA will raise
    an error.
    """
    service_data = _status().get("services")
    enabled_services = []
    disabled_services = []

    if service_data:
        # sort services into enabled and disabled
        for service in services:
            if _is_service_enabled(service, service_data):
                enabled_services.append(service)
            else:
                disabled_services.append(service)
    else:
        # assume all services are disabled if they are not in the status data
        disabled_services = services

    if enabled_services:
        logger.debug(
            "Not re-enabling services %s because the services are already enabled.",
            enabled_services,
        )

    if disabled_services:
        logger.debug("Enabling services %s.", disabled_services)
        subprocess.check_call(
            ["sudo", "ua", "enable", *disabled_services, "--beta", "--assume-yes"]
        )


def _detach() -> None:
    """Detach UA token (ua will update package lists)."""
    subprocess.check_call(["sudo", "ua", "detach", "--assume-yes"])


def _is_attached() -> bool:
    return _status()["attached"]


def _is_service_enabled(service_name: str, service_data: List[Dict[str, str]]) -> bool:
    """Check if a service is enabled.

    :param service_name: name of service to check
    :param service_data: list of dictionaries containing data for each service

    :return: True if the service is enabled. False is the service is disabled or not
    listed in the list of service data.
    """
    for service in service_data:
        if service.get("name") == service_name:
            # possible statuses are 'enabled', 'disabled', and 'n/a'
            return service.get("status") == "enabled"

    logger.debug("Could not find service %r.", service_name)
    return False


def _status() -> Dict[str, Any]:
    stdout = subprocess.check_output(
        ["sudo", "ua", "status", "--all", "--format", "json"]
    )
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
            logger.warning(
                "Refusing to attach specified UA token as system is already attached. "
            )
        else:
            ua_needs_detach = True
            logger.info("Attaching specified UA token...")
            _attach(ua_token)

    try:
        if services:
            logger.info("Enabling specified UA services...")
            _enable_services(services)

        yield
    finally:
        if ua_needs_detach:
            logger.info("Detaching specified UA token...")
            _detach()
