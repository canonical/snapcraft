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

import json
import subprocess

import pytest

from snapcraft_legacy.internal.repo import ua_manager


@pytest.fixture()
def mock_status_data():
    """Returns mock data from calling `ua status --all --format=json`.

    Contains two disabled services (disabled-service-1 and disabled-service-2) and one enabled
    service (enabled-service).
    """
    return json.dumps(
        {
            "_doc": "Content...",
            "attached": False,
            "services": [
                {
                    "available": "yes",
                    "blocked_by": [],
                    "description": "Common Criteria EAL2 Provisioning Packages",
                    "description_override": None,
                    "entitled": "yes",
                    "name": "disabled-service-1",
                    "status": "disabled",
                    "status_details": "",
                },
                {
                    "available": "yes",
                    "blocked_by": [],
                    "description": "Expanded Security Maintenance for Applications",
                    "description_override": None,
                    "entitled": "yes",
                    "name": "disabled-service-2",
                    "status": "n/a",
                    "status_details": "Ubuntu Pro: ESM Apps is not configured",
                },
                {
                    "available": "yes",
                    "blocked_by": [],
                    "description": "Expanded Security Maintenance for Infrastructure",
                    "description_override": None,
                    "entitled": "yes",
                    "name": "enabled-service",
                    "status": "enabled",
                    "status_details": "Ubuntu Pro: ESM Infra is active",
                },
            ],
        }
    )


@pytest.fixture(autouse=True)
def _setup_fixture(mocker):
    mocker.patch("snapcraft_legacy.internal.repo.Repo.refresh_build_packages")
    mocker.patch("snapcraft_legacy.internal.repo.Repo._install_packages")


def test_ua_manager(fake_process):
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager("test-ua-token", services=None):
        pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--all", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_already_attached(fake_process):
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": true, "_doc": "Content..."}',
    )

    with ua_manager.ua_manager("ua-token", services=None):
        pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--all", "--format", "json"]
    ]


def test_ua_manager_enable_services(fake_process, mock_status_data):
    """Test enabling of services:
    - Disabled services are enabled.
    - Services not in the status data are enabled.
    - Enabled services are not re-enabled.
    """
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"], stdout=mock_status_data
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"], stdout=mock_status_data
    )
    fake_process.register_subprocess(
        [
            "sudo",
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "service-not-in-status-data",
            "--beta",
            "--assume-yes",
        ]
    )
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(
        "test-ua-token",
        services=[
            "disabled-service-1",
            "disabled-service-2",
            "service-not-in-status-data",
        ],
    ):
        pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--all", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "status", "--all", "--format", "json"],
        [
            "sudo",
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "service-not-in-status-data",
            "--beta",
            "--assume-yes",
        ],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_enable_services_no_services(fake_process):
    """Assume services are disabled if the services node is not in the status data."""
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(
        [
            "sudo",
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "--beta",
            "--assume-yes",
        ]
    )
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(
        "test-ua-token", services=["disabled-service-1", "disabled-service-2"]
    ):
        pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--all", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "status", "--all", "--format", "json"],
        [
            "sudo",
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "--beta",
            "--assume-yes",
        ],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_enable_services_error(fake_process):
    """Raise a UAEnableServicesError when the `ua enable` command fails."""
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(
        ["sudo", "ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"], returncode=1
    )
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with pytest.raises(subprocess.CalledProcessError) as raised:
        with ua_manager.ua_manager("test-ua-token", services=["svc1", "svc2"]):
            pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--all", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "status", "--all", "--format", "json"],
        ["sudo", "ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]
    assert raised.value.returncode == 1


def test_ua_manager_detaches_on_error(fake_process):
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with pytest.raises(RuntimeError):
        with ua_manager.ua_manager("test-ua-token", services=None):
            raise RuntimeError("whoopsie")

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--all", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]
