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

import json

import pytest

from snapcraft import ua_manager


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
    mocker.patch(
        "craft_parts.packages.Repository.is_package_installed", return_value=True
    )


def test_ua_manager(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager("test-ua-token", services=None):
        pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_already_attached(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": true, "_doc": "Content..."}',
    )

    with ua_manager.ua_manager("ua-token", services=None):
        pass

    assert list(fake_process.calls) == [["ua", "status", "--all", "--format", "json"]]


def test_ua_manager_attach_error(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"], returncode=23)

    with pytest.raises(ua_manager.UATokenAttachError) as raised:
        with ua_manager.ua_manager("test-ua-token", services=None):
            pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
    ]
    assert str(raised.value) == "Error attaching UA token."


def test_ua_manager_enable_services(fake_process, mock_status_data):
    """Test enabling of services:
    - Disabled services are enabled.
    - Services not in the status data are enabled.
    - Enabled services are not re-enabled.
    """
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"], stdout=mock_status_data
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"], stdout=mock_status_data
    )
    fake_process.register_subprocess(
        [
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "service-not-in-status-data",
            "--beta",
            "--assume-yes",
        ]
    )
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

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
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "status", "--all", "--format", "json"],
        [
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "service-not-in-status-data",
            "--beta",
            "--assume-yes",
        ],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_enable_services_no_services(fake_process):
    """Assume services are disabled if the services node is not in the status data."""
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(
        [
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "--beta",
            "--assume-yes",
        ]
    )
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(
        "test-ua-token", services=["disabled-service-1", "disabled-service-2"]
    ):
        pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "status", "--all", "--format", "json"],
        [
            "ua",
            "enable",
            "disabled-service-1",
            "disabled-service-2",
            "--beta",
            "--assume-yes",
        ],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_enable_services_error(fake_process):
    """Raise a UAEnableServicesError when the `ua enable` command fails."""
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(
        ["ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"], returncode=1
    )
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with pytest.raises(ua_manager.UAEnableServicesError) as raised:
        with ua_manager.ua_manager("test-ua-token", services=["svc1", "svc2"]):
            pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"],
        ["ua", "detach", "--assume-yes"],
    ]
    assert str(raised.value) == "Error enabling UA services."


def test_ua_manager_detaches_on_error(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with pytest.raises(RuntimeError):
        with ua_manager.ua_manager("test-ua-token", services=None):
            raise RuntimeError("whoopsie")

    assert list(fake_process.calls) == [
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_detach_error(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--all", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"], returncode=34)

    with pytest.raises(ua_manager.UATokenDetachError) as raised:
        with ua_manager.ua_manager("test-ua-token", services=None):
            pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--all", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "detach", "--assume-yes"],
    ]
    assert str(raised.value) == "Error detaching UA token."
