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

import subprocess

import pytest

from snapcraft_legacy.internal.repo import ua_manager


@pytest.fixture(autouse=True)
def _setup_fixture(mocker):
    mocker.patch("snapcraft_legacy.internal.repo.Repo.refresh_build_packages")
    mocker.patch("snapcraft_legacy.internal.repo.Repo._install_packages")


def test_ua_manager(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(ua_token, services=None):
        pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_already_attached(fake_process):
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--format", "json"],
        stdout='{"attached": true, "_doc": "Content..."}',
    )

    with ua_manager.ua_manager("ua-token", services=None):
        pass

    assert list(fake_process.calls) == [["sudo", "ua", "status", "--format", "json"]]


def test_ua_manager_enable_services(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["sudo", "ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"]
    )
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(ua_token, services=["svc1", "svc2"]):
        pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_enable_services_error(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["sudo", "ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"], returncode=1
    )
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with pytest.raises(subprocess.CalledProcessError) as raised:
        with ua_manager.ua_manager(ua_token, services=["svc1", "svc2"]):
            pass

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "enable", "svc1", "svc2", "--beta", "--assume-yes"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]
    assert raised.value.returncode == 1


def test_ua_manager_detaches_on_error(fake_process):
    fake_process.register_subprocess(
        ["sudo", "ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["sudo", "ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["sudo", "ua", "detach", "--assume-yes"])

    with pytest.raises(RuntimeError):
        with ua_manager.ua_manager("test-ua-token", services=None):
            raise RuntimeError("whoopsie")

    assert list(fake_process.calls) == [
        ["sudo", "ua", "status", "--format", "json"],
        ["sudo", "ua", "attach", "test-ua-token"],
        ["sudo", "ua", "detach", "--assume-yes"],
    ]
