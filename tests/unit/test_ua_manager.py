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

import pytest

from snapcraft import ua_manager


@pytest.fixture(autouse=True)
def _setup_fixture(mocker):
    mocker.patch(
        "craft_parts.packages.Repository.is_package_installed", return_value=True
    )


def test_ua_manager(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(ua_token, services=None):
        pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_already_attached(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": true, "_doc": "Content..."}',
    )

    with ua_manager.ua_manager("ua-token", services=None):
        pass

    assert list(fake_process.calls) == [["ua", "status", "--format", "json"]]


def test_ua_manager_attach_error(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"], returncode=23)

    with pytest.raises(ua_manager.UATokenAttachError) as raised:
        with ua_manager.ua_manager(ua_token, services=None):
            pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
    ]
    assert str(raised.value) == "Error attaching UA token."


def test_ua_manager_enable_services(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "enable", "svc1", "svc2", "--beta"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with ua_manager.ua_manager(ua_token, services=["svc1", "svc2"]):
        pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "enable", "svc1", "svc2", "--beta"],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_enable_services_error(fake_process):
    ua_token = "test-ua-token"

    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(
        ["ua", "enable", "svc1", "svc2", "--beta"], returncode=1
    )
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with pytest.raises(ua_manager.UAEnableServicesError) as raised:
        with ua_manager.ua_manager(ua_token, services=["svc1", "svc2"]):
            pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "enable", "svc1", "svc2", "--beta"],
        ["ua", "detach", "--assume-yes"],
    ]
    assert str(raised.value) == "Error enabling UA services."


def test_ua_manager_detaches_on_error(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"])

    with pytest.raises(RuntimeError):
        with ua_manager.ua_manager("test-ua-token", services=None):
            raise RuntimeError("whoopsie")

    assert list(fake_process.calls) == [
        ["ua", "status", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "detach", "--assume-yes"],
    ]


def test_ua_manager_detach_error(fake_process):
    fake_process.register_subprocess(
        ["ua", "status", "--format", "json"],
        stdout='{"attached": false, "_doc": "Content..."}',
    )
    fake_process.register_subprocess(["ua", "attach", "test-ua-token"])
    fake_process.register_subprocess(["ua", "detach", "--assume-yes"], returncode=34)

    with pytest.raises(ua_manager.UATokenDetachError) as raised:
        with ua_manager.ua_manager("test-ua-token", services=None):
            pass

    assert list(fake_process.calls) == [
        ["ua", "status", "--format", "json"],
        ["ua", "attach", "test-ua-token"],
        ["ua", "detach", "--assume-yes"],
    ]
    assert str(raised.value) == "Error detaching UA token."
