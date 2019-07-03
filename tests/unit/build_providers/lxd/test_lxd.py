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

import os
import subprocess
from typing import Any, Dict
from unittest import mock

from testtools.matchers import Equals, FileContains, FileExists

from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.internal.build_providers import errors
from snapcraft.internal.build_providers._lxd import LXD
from snapcraft.internal.repo.errors import SnapdConnectionError
from tests.unit.build_providers import BaseProviderBaseTest


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

    def test_destroy_when_not_created(self):
        instance = LXD(project=self.project, echoer=self.echoer_mock)
        # This call should not fail
        instance.destroy()

    def test_create_for_type_base(self):
        self.project.info.name = "core18"
        self.project.info.type = "base"
        self.project.info.base = None

        instance = LXD(project=self.project, echoer=self.echoer_mock)
        with mock.patch.object(
            LXD, "_get_cloud_user_data_string", return_value="fake-cloud"
        ):
            instance.create()

        self.fake_pylxd_client.containers.create_mock.assert_called_once_with(
            config={
                "name": "snapcraft-core18",
                "environment.SNAPCRAFT_BUILD_ENVIRONMENT": "managed-host",
                "raw.idmap": "both 1000 0",
                "source": {
                    "mode": "pull",
                    "type": "image",
                    "server": "https://cloud-images.ubuntu.com/minimal/releases/",
                    "protocol": "simplestreams",
                    "alias": "18.04",
                },
                "environment.SNAPCRAFT_HAS_TTY": "False",
                "user.user-data": "fake-cloud",
            },
            wait=True,
        )


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
            dict(lxd=dict(channels={"latest/stable": dict(confinement="strict")}))
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
