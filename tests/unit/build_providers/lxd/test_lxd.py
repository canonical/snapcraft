# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
from typing import Any, Dict
from unittest import mock

from testtools.matchers import Equals, FileContains, FileExists

from tests.unit.build_providers import BaseProviderBaseTest
from snapcraft.internal.build_providers._lxd import LXD


class FakeContainer:
    @property
    def status(self) -> str:
        return self._status

    def __init__(self, config: Dict[str, Any], wait: bool) -> None:
        self._status = "STOPPED"
        self.config = config
        self.devices = dict()  # type: Dict[str, Any]

        self.save_mock = mock.Mock()
        self.start_mock = mock.Mock()
        self.stop_mock = mock.Mock()
        self.delete_mock = mock.Mock()
        self.sync_mock = mock.Mock()
        self.files_get_mock = mock.Mock()
        self.files_put_mock = mock.Mock()
        self.files_delete_mock = mock.Mock()

        class FakeContainerFiles:
            delete_available = True

            def get(file_name) -> bytes:
                self.files_get_mock(file_name=file_name)
                return b"fake-pull"

            def put(destination: str, contents: bytes) -> None:
                self.files_put_mock(destination=destination, contents=contents)

            def delete(file_name) -> None:
                self.files_delete_mock(file_name=file_name)

        self.files = FakeContainerFiles

    def _reset_mocks(self) -> None:
        self.save_mock.reset_mock()
        self.start_mock.reset_mock()
        self.stop_mock.reset_mock()
        self.delete_mock.reset_mock()
        self.sync_mock.reset_mock()
        self.files_get_mock.reset_mock()
        self.files_put_mock.reset_mock()
        self.files_delete_mock.reset_mock()

    def save(self, wait: bool) -> None:
        self.save_mock(wait=wait)

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

    def exists(self, container_name: str) -> bool:
        return container_name in self._containers

    def get(self, container_name: str) -> FakeContainer:
        return self._containers[container_name]

    def create(self, config: Dict[str, Any], wait: bool) -> FakeContainer:
        self.create_mock(config=config, wait=wait)
        self._containers[config["name"]] = FakeContainer(config, wait)
        return self._containers[config["name"]]


class FakePyLXDClient:
    def __init__(self) -> None:
        self.containers = FakeContainers()


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

        patcher = mock.patch("subprocess.check_output", spec=subprocess.check_output)
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)


class LXDInitTest(LXDBaseTest):
    def test_create(self):
        instance = LXD(project=self.project, echoer=self.echoer_mock)
        with mock.patch.object(
            LXD, "_get_cloud_user_data_string", return_value="fake-cloud"
        ):
            instance.create()

        self.fake_pylxd_client.containers.create_mock.assert_called_once_with(
            config={
                "name": "snapcraft-project-name",
                "environment.SNAPCRAFT_BUILD_ENVIRONMENT": "managed-host",
                "raw.idmap": "both 1000 0",
                "source": {
                    "mode": "pull",
                    "type": "image",
                    "server": "https://cloud-images.ubuntu.com/minimal/releases/",
                    "protocol": "simplestreams",
                    "alias": "16.04",
                },
                "environment.SNAPCRAFT_HAS_TTY": "False",
                "user.user-data": "fake-cloud",
            },
            wait=True,
        )

        container = self.fake_pylxd_client.containers.get(self.instance_name)
        container.start_mock.assert_called_once_with(wait=True)
        self.assertThat(container.save_mock.call_count, Equals(2))
        self.assertThat(container.sync_mock.call_count, Equals(3))
        self.assertThat(self.check_call_mock.call_count, Equals(2))
        self.check_call_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        self.instance_name,
                        "--",
                        "cloud-init",
                        "status",
                        "--wait",
                    ]
                ),
                mock.call(
                    [
                        "/snap/bin/lxc",
                        "exec",
                        self.instance_name,
                        "--",
                        "snapcraft",
                        "refresh",
                    ]
                ),
            ]
        )

    def test_clean_project(self):
        instance = LXD(project=self.project, echoer=self.echoer_mock)
        instance.create()

        instance.clean_project()

        container = self.fake_pylxd_client.containers.get(self.instance_name)
        container.stop_mock.assert_called_once_with(wait=True)
        container.delete_mock.assert_called_once_with(wait=True)
        self.assertThat(instance.clean_project(), Equals(True))

    def test_clean_project_new_instance(self):
        pre_instance = LXD(project=self.project, echoer=self.echoer_mock)
        pre_instance.create()
        pre_instance._stop()

        self.fake_pylxd_client.containers.get(self.instance_name)._reset_mocks()

        instance = LXD(project=self.project, echoer=self.echoer_mock)
        instance.clean_project()

        container = self.fake_pylxd_client.containers.get(self.instance_name)
        container.stop_mock.assert_not_called()
        container.delete_mock.assert_called_once_with(wait=True)
        self.assertThat(instance.clean_project(), Equals(True))


class LXDLaunchedTest(LXDBaseTest):
    def setUp(self):
        super().setUp()

        self.instance = LXD(project=self.project, echoer=self.echoer_mock)
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
            ["/snap/bin/lxc", "exec", "snapcraft-project-name", "--", "/bin/bash"]
        )

    def test_mount_project(self):
        self.check_output_mock.return_value = b"/root"

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
        self.assertThat(self.fake_container.sync_mock.call_count, Equals(2))
        self.fake_container.save_mock.assert_called_once_with(wait=True)
        self.check_output_mock.assert_called_once_with(
            ["/snap/bin/lxc", "exec", self.instance_name, "--", "printenv", "HOME"]
        )

    def test_run(self):
        self.instance._run(["ls", "/root/project"])

        self.check_call_mock.assert_called_once_with(
            [
                "/snap/bin/lxc",
                "exec",
                "snapcraft-project-name",
                "--",
                "ls",
                "/root/project",
            ]
        )
