# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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

import os
import subprocess
import sys
from typing import Any, Dict
from unittest import mock
from unittest.mock import call

from testtools.matchers import Equals, FileContains, FileExists

from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.internal.build_providers import _base_provider, errors
from snapcraft.internal.build_providers._lxd import LXD
from snapcraft.internal.repo.errors import SnapdConnectionError
from tests.unit.build_providers import BaseProviderBaseTest


if sys.platform == "linux":
    import pylxd


class GetEnv(_base_provider.Provider):
    def _get_env_command(self):
        return ["env", "SNAPCRAFT_HAS_TTY=False"]


class LXDTestImpl(LXD, GetEnv):
    pass


class FakeContainer:
    @property
    def status(self) -> str:
        return self._status

    def __init__(self, config: Dict[str, Any], wait: bool) -> None:
        self._status = "STOPPED"
        self.config = config
        self.devices = dict()  # type: Dict[str, Any]

        self.delete_mock = mock.Mock()
        self.files_delete_mock = mock.Mock()
        self.files_get_mock = mock.Mock()
        self.files_put_mock = mock.Mock()
        self.restart_mock = mock.Mock()
        self.save_mock = mock.Mock()
        self.start_mock = mock.Mock()
        self.stop_mock = mock.Mock()
        self.sync_mock = mock.Mock()

        class FakeContainerFiles:
            delete_available = True

            @staticmethod
            def get(file_name) -> bytes:
                self.files_get_mock(file_name=file_name)
                return b"fake-pull"

            @staticmethod
            def put(destination: str, contents: bytes) -> None:
                self.files_put_mock(destination=destination, contents=contents)

            @staticmethod
            def delete(file_name) -> None:
                self.files_delete_mock(file_name=file_name)

        self.files = FakeContainerFiles

    def _reset_mocks(self) -> None:
        self.delete_mock.reset_mock()
        self.files_delete_mock.reset_mock()
        self.files_get_mock.reset_mock()
        self.files_put_mock.reset_mock()
        self.restart_mock.reset_mock()
        self.save_mock.reset_mock()
        self.start_mock.reset_mock()
        self.stop_mock.reset_mock()
        self.sync_mock.reset_mock()

    def restart(self, wait: bool) -> None:
        self.restart_mock(wait=wait)

    def save(self, wait: bool) -> None:
        self.save_mock(wait=wait)

    def state(self):
        class State:
            network = dict(
                eth0=dict(addresses=[dict(address="192.168.0.1", family="inet")])
            )

        return State()

    def sync(self) -> None:
        self.sync_mock()

    def start(self, wait: bool) -> None:
        self.start_mock(wait=wait)
        self._status = "RUNNING"

    def stop(self, wait: bool) -> None:
        self.stop_mock(wait=wait)
        self._status = "STOPPED"

    def delete(self, wait: bool) -> None:
        self.delete_mock(wait=wait)


class FakeContainers:
    def __init__(self):
        self._containers = dict()  # type: Dict[str, FakeContainer]
        self.create_mock = mock.Mock()
        self.get_mock = mock.Mock()

    def exists(self, container_name: str) -> bool:
        return container_name in self._containers

    def get(self, container_name: str) -> FakeContainer:
        self.get_mock(container_name)
        return self._containers[container_name]

    def create(self, config: Dict[str, Any], wait: bool) -> FakeContainer:
        self.create_mock(config=config, wait=wait)
        self._containers[config["name"]] = FakeContainer(config, wait)
        return self._containers[config["name"]]


class FakePyLXDClient:
    def __init__(self) -> None:
        self.containers = FakeContainers()
        self.host_info = {
            "environment": {"kernel_features": {"seccomp_listener": "true"}}
        }


class LXDBaseTest(BaseProviderBaseTest):
    def setUp(self):
        super().setUp()

        self.fake_pylxd_client = FakePyLXDClient()
        patcher = mock.patch("pylxd.Client", return_value=self.fake_pylxd_client)
        self.pylxd_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider.Provider.clean_project",
            return_value=True,
        )

        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_call", spec=subprocess.check_call)
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        def check_output_effect(command: str, *arch, **kwargs) -> bytes:
            if "is-system-running" in command:
                raise subprocess.CalledProcessError(
                    returncode=1,
                    cmd=["systemctl", "is-system-running"],
                    output="degraded\n".encode(),
                )

            if "getent" in command:
                output = "2001:67c:1562::20 snapcraft.io"
            else:
                output = ""

            return output.encode()

        patcher = mock.patch(
            "subprocess.check_output",
            spec=subprocess.check_output,
            side_effect=check_output_effect,
        )
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)


class LXDInitTest(LXDBaseTest):
    def test_create(self):
        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)

        instance.create()

        self.fake_pylxd_client.containers.create_mock.assert_called_once_with(
            config={
                "name": "snapcraft-project-name",
                "raw.idmap": f"both {os.getuid()} 0",
                "security.syscalls.intercept.mknod": "true",
                "source": {
                    "mode": "pull",
                    "type": "image",
                    "server": "https://cloud-images.ubuntu.com/buildd/daily",
                    "protocol": "simplestreams",
                    "alias": "16.04",
                },
            },
            wait=True,
        )

        container = self.fake_pylxd_client.containers.get(self.instance_name)
        container.start_mock.assert_called_once_with(wait=True)
        self.assertThat(container.save_mock.call_count, Equals(2))
        self.assertThat(self.check_output_mock.call_count, Equals(3))
        self.check_output_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "is-system-running",
                    ]
                )
            ]
            + [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "getent",
                        "hosts",
                        "snapcraft.io",
                    ]
                )
            ]
            * 2
        )
        self.check_call_mock.assert_has_calls(
            [
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L3Jvb3QvLmJhc2hyYw==",
                        "/root/.bashrc",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/root/.bashrc",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0600",
                        "/root/.bashrc",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2Jpbi9fc25hcGNyYWZ0X3Byb21wdA==",
                        "/bin/_snapcraft_prompt",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/bin/_snapcraft_prompt",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0755",
                        "/bin/_snapcraft_prompt",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0",
                        "/etc/apt/sources.list",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/etc/apt/sources.list",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0644",
                        "/etc/apt/sources.list",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0LmQvZGVmYXVsdC5zb3VyY2Vz",
                        "/etc/apt/sources.list.d/default.sources",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/etc/apt/sources.list.d/default.sources",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0644",
                        "/etc/apt/sources.list.d/default.sources",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0LmQvZGVmYXVsdC1zZWN1cml0eS5zb3VyY2Vz",
                        "/etc/apt/sources.list.d/default-security.sources",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/etc/apt/sources.list.d/default-security.sources",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0644",
                        "/etc/apt/sources.list.d/default-security.sources",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2V0Yy9hcHQvYXB0LmNvbmYuZC8wMC1zbmFwY3JhZnQ=",
                        "/etc/apt/apt.conf.d/00-snapcraft",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/etc/apt/apt.conf.d/00-snapcraft",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0644",
                        "/etc/apt/apt.conf.d/00-snapcraft",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2V0Yy9zeXN0ZW1kL25ldHdvcmsvMTAtZXRoMC5uZXR3b3Jr",
                        "/etc/systemd/network/10-eth0.network",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/etc/systemd/network/10-eth0.network",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0644",
                        "/etc/systemd/network/10-eth0.network",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "mv",
                        "/var/tmp/L2V0Yy9ob3N0bmFtZQ==",
                        "/etc/hostname",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chown",
                        "root:root",
                        "/etc/hostname",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "chmod",
                        "0644",
                        "/etc/hostname",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "ln",
                        "-sf",
                        "/run/systemd/resolve/resolv.conf",
                        "/etc/resolv.conf",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "enable",
                        "systemd-resolved",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "enable",
                        "systemd-networkd",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "restart",
                        "systemd-resolved",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "restart",
                        "systemd-networkd",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "apt-get",
                        "update",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "apt-get",
                        "install",
                        "dirmngr",
                        "udev",
                        "fuse",
                        "--yes",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "enable",
                        "systemd-udevd",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "start",
                        "systemd-udevd",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "apt-get",
                        "install",
                        "snapd",
                        "sudo",
                        "--yes",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "start",
                        "snapd",
                    ]
                ),
                call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "apt-get",
                        "update",
                    ]
                ),
            ]
        )

    def test_clean_project(self):
        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)
        instance.create()

        instance.clean_project()

        container = self.fake_pylxd_client.containers.get(self.instance_name)
        container.stop_mock.assert_called_once_with(wait=True)
        container.delete_mock.assert_called_once_with(wait=True)
        self.assertThat(instance.clean_project(), Equals(True))

    def test_clean_project_new_instance(self):
        pre_instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)
        pre_instance.create()
        pre_instance._stop()

        self.fake_pylxd_client.containers.get(self.instance_name)._reset_mocks()

        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)
        instance.clean_project()

        container = self.fake_pylxd_client.containers.get(self.instance_name)
        container.stop_mock.assert_not_called()
        container.delete_mock.assert_called_once_with(wait=True)
        self.assertThat(instance.clean_project(), Equals(True))

    def test_clean_when_missing(self):
        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)
        self.fake_pylxd_client.containers.get_mock.side_effect = pylxd.exceptions.NotFound(
            "not found"
        )

        instance.clean_project()

    def test_destroy_when_not_created(self):
        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)
        # This call should not fail
        instance.destroy()

    def test_create_for_type_base(self):
        self.project._snap_meta.name = "core18"
        self.project._snap_meta.type = "base"
        self.project._snap_meta.base = None

        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)

        instance.create()

        self.fake_pylxd_client.containers.create_mock.assert_called_once_with(
            config={
                "name": "snapcraft-core18",
                "raw.idmap": f"both {os.getuid()} 0",
                "security.syscalls.intercept.mknod": "true",
                "source": {
                    "mode": "pull",
                    "type": "image",
                    "server": "https://cloud-images.ubuntu.com/buildd/daily",
                    "protocol": "simplestreams",
                    "alias": "18.04",
                },
            },
            wait=True,
        )

    def test_create_invalid_base(self):
        self.project._snap_meta.base = "core19"

        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)

        self.assertRaises(errors.ProviderInvalidBaseError, instance.create)

    def test_create_without_syscall_intercept(self):
        self.fake_pylxd_client.host_info["environment"]["kernel_features"][
            "seccomp_listener"
        ] = "false"

        instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)

        instance.create()

        self.fake_pylxd_client.containers.create_mock.assert_called_once_with(
            config={
                "name": "snapcraft-project-name",
                "raw.idmap": f"both {os.getuid()} 0",
                "source": {
                    "mode": "pull",
                    "type": "image",
                    "server": "https://cloud-images.ubuntu.com/buildd/daily",
                    "protocol": "simplestreams",
                    "alias": "16.04",
                },
            },
            wait=True,
        )


class LXDLaunchedTest(LXDBaseTest):
    def setUp(self):
        super().setUp()

        self.instance = LXDTestImpl(project=self.project, echoer=self.echoer_mock)
        self.instance.create()
        self.fake_container = self.fake_pylxd_client.containers.get(self.instance_name)
        # reset for the tests to only be concerned about what they are testing
        self.fake_container._reset_mocks()
        self.fake_pylxd_client.containers.create_mock.reset_mock()
        self.check_call_mock.reset_mock()

    def test_destroy(self):
        self.instance.destroy()

        self.fake_container.stop_mock.assert_called_once_with(wait=True)

    def test_destroy_and_launch(self):
        self.instance.destroy()

        self.fake_container.stop_mock.assert_called_once_with(wait=True)

        self.instance.create()

        self.fake_pylxd_client.containers.create_mock.assert_not_called()
        self.fake_container.start_mock.assert_called_once_with(wait=True)

    def test_pull_file(self):
        self.instance.pull_file("src.txt", "dest.txt")

        self.fake_container.files_get_mock.assert_called_once_with(file_name="src.txt")
        self.fake_container.files_delete_mock.assert_not_called()
        self.assertThat("dest.txt", FileExists())
        self.assertThat("dest.txt", FileContains("fake-pull"))

    def test_pull_file_and_delete(self):
        self.instance.pull_file("src.txt", "dest.txt", delete=True)

        self.fake_container.files_get_mock.assert_called_once_with(file_name="src.txt")
        self.fake_container.files_delete_mock.assert_called_once_with(
            file_name="src.txt"
        )
        self.assertThat("dest.txt", FileExists())
        self.assertThat("dest.txt", FileContains("fake-pull"))

    def test_push_file(self):
        with open("src.txt", "w") as f:
            f.write("fake-put")

        self.instance._push_file(source="src.txt", destination="dest.txt")

        self.fake_container.files_put_mock.assert_called_once_with(
            destination="dest.txt", contents=b"fake-put"
        )

    def test_shell(self):
        self.instance.shell()

        self.check_call_mock.assert_called_once_with(
            [
                "/snap/bin/lxc",
                "exec",
                "snapcraft-project-name",
                "--",
                "env",
                "SNAPCRAFT_HAS_TTY=False",
                "/bin/bash",
            ]
        )

    def test_mount_project(self):
        self.instance.mount_project()

        self.assertThat(
            self.fake_container.devices,
            Equals(
                {
                    "snapcraft-project": dict(
                        path="/root/project", source=self.path, type="disk"
                    )
                }
            ),
        )
        self.assertThat(self.fake_container.sync_mock.call_count, Equals(1))
        self.fake_container.save_mock.assert_called_once_with(wait=True)
        self.assertThat(self.check_output_mock.call_count, Equals(3))
        self.check_output_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "is-system-running",
                    ]
                )
            ]
            + [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "getent",
                        "hosts",
                        "snapcraft.io",
                    ]
                )
            ]
            * 2
        )

    def test_mount_prime_directory(self):
        self.check_output_mock.return_value = b"/root"

        self.instance._mount_prime_directory()

        self.assertThat(
            self.fake_container.devices,
            Equals(
                {
                    "snapcraft-project-prime": dict(
                        path="/root/prime",
                        source=os.path.join(self.path, "prime"),
                        type="disk",
                    )
                }
            ),
        )
        self.assertThat(self.fake_container.sync_mock.call_count, Equals(1))
        self.fake_container.save_mock.assert_called_once_with(wait=True)
        self.assertThat(self.check_output_mock.call_count, Equals(3))
        self.check_output_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "systemctl",
                        "is-system-running",
                    ]
                )
            ]
            + [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        "snapcraft-project-name",
                        "--",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "getent",
                        "hosts",
                        "snapcraft.io",
                    ]
                )
            ]
            * 2
        )

    def test_run(self):
        self.instance._run(["ls", "/root/project"])

        self.check_call_mock.assert_called_once_with(
            [
                "/snap/bin/lxc",
                "exec",
                "snapcraft-project-name",
                "--",
                "env",
                "SNAPCRAFT_HAS_TTY=False",
                "ls",
                "/root/project",
            ]
        )


class EnsureLXDTest(LXDBaseTest):
    def test_linux(self):
        self.fake_snapd.snaps_result = [
            dict(name="lxd", channel="stable", revision="10")
        ]

        # Thou shall not fail
        with mock.patch(
            "snapcraft.internal.repo.Repo.is_package_installed", return_value=False
        ):
            LXD.ensure_provider()

    def test_linux_with_snap_and_deb_installed(self):
        self.fake_snapd.snaps_result = [
            dict(name="lxd", channel="stable", revision="10")
        ]

        # Thou shall not fail
        with mock.patch(
            "snapcraft.internal.repo.Repo.is_package_installed", return_value=True
        ):
            raised = self.assertRaises(SnapcraftEnvironmentError, LXD.ensure_provider)

        self.assertThat(
            str(raised),
            Equals(
                "The 'LXD' provider does not support having the 'lxd' "
                "or 'lxd-client' deb packages installed. "
                "To completely migrate to the LXD snap run 'lxd.migrate' "
                "and try again."
            ),
        )

    def test_lxd_snap_not_installed(self):
        raised = self.assertRaises(errors.ProviderNotFound, LXD.ensure_provider)

        self.assertThat(raised.prompt_installable, Equals(True))
        self.assertThat(
            raised.error_message,
            Equals("The LXD snap is required to continue: snap install lxd"),
        )

    def test_snap_support_missing(self):
        with mock.patch(
            "snapcraft.internal.repo.snaps.SnapPackage.is_snap_installed",
            side_effect=SnapdConnectionError(snap_name="lxd", url="fake"),
        ):
            raised = self.assertRaises(errors.ProviderNotFound, LXD.ensure_provider)

        self.assertThat(raised.prompt_installable, Equals(False))
        self.assertThat(
            raised.error_message,
            Equals(
                "snap support is required to continue: "
                "https://docs.snapcraft.io/installing-snapd/6735"
            ),
        )

    def test_non_linux(self):
        with mock.patch("sys.platform", return_value="darwin"):
            raised = self.assertRaises(errors.ProviderNotFound, LXD.ensure_provider)

        self.assertThat(raised.prompt_installable, Equals(False))
        self.assertThat(
            raised.error_message, Equals("LXD is not supported on this platform")
        )


class SetupLXDTest(LXDBaseTest):
    def setUp(self):
        super().setUp()

        self.fake_snapd.snaps_result = [
            dict(name="lxd", channel="stable", revision="10")
        ]
        self.fake_snapd.find_result = [
            dict(
                lxd=dict(
                    channel="stable",
                    type="app",
                    channels={"latest/stable": dict(confinement="strict")},
                )
            )
        ]

    def test_install(self):
        LXD.setup_provider(echoer=self.echoer_mock)

        self.assertThat(self.check_output_mock.call_count, Equals(2))
        self.check_output_mock.assert_has_calls(
            [
                mock.call(["/snap/bin/lxd", "waitready", "--timeout=30"]),
                mock.call(["/snap/bin/lxd", "init", "--auto"]),
            ]
        )

    def test_lxd_wait_fails(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(
            cmd=["wait-fake"], returncode=1
        )

        raised = self.assertRaises(
            SnapcraftEnvironmentError, LXD.setup_provider, echoer=self.echoer_mock
        )

        self.assertThat(
            str(raised), Equals("Timeout reached waiting for LXD to start.")
        )

    def test_lxd_init_fails(self):
        self.check_output_mock.side_effect = [
            b"",
            subprocess.CalledProcessError(cmd=["init-fake"], returncode=1),
        ]

        raised = self.assertRaises(
            SnapcraftEnvironmentError, LXD.setup_provider, echoer=self.echoer_mock
        )

        self.assertThat(
            str(raised),
            Equals(
                "Failed to initialize LXD. "
                "Try manually initializing before trying again: lxd init --auto."
            ),
        )
