# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from unittest import mock

from snapcraft.project import Project

from tests import fixture_setup, unit
from snapcraft.internal.build_providers._base_provider import Provider


class ProviderImpl(Provider):
    def __init__(self, *, project, echoer, is_ephemeral=False):
        super().__init__(project=project, echoer=echoer, is_ephemeral=is_ephemeral)

        self.run_mock = mock.Mock()
        self.launch_mock = mock.Mock()
        self.start_mock = mock.Mock()
        self.mount_mock = mock.Mock()
        self.unmount_mock = mock.Mock()
        self.push_file_mock = mock.Mock()
        self.create_mock = mock.Mock()
        self.destroy_mock = mock.Mock()
        self.mount_project_mock = mock.Mock()
        self.shell_mock = mock.Mock()

    def _run(self, command, hide_output=False):
        self.run_mock(command)

    def _launch(self) -> None:
        self.launch_mock()

    def _start(self) -> None:
        self.start_mock()

    def _mount(self, *, mountpoint: str, dev_or_path: str) -> None:
        self.mount_mock(mountpoint=mountpoint, dev_or_path=dev_or_path)

    def _unmount(self, *, mountpoint: str) -> None:
        self.unmount_mock(mountpoint=mountpoint)

    def _mount_snaps_directory(self) -> None:
        self._mount(mountpoint=self._SNAPS_MOUNTPOINT, dev_or_path="snaps-dev")

    def _unmount_snaps_directory(self) -> None:
        self._unmount(mountpoint=self._SNAPS_MOUNTPOINT)

    def _push_file(self, *, source: str, destination: str) -> None:
        self.push_file_mock(source=source, destination=destination)

    def _get_provider_name(self) -> str:
        return "stub-provider"

    def _umount(self):
        raise NotImplementedError("test stub not implemented")

    def build_project(self) -> None:
        raise NotImplementedError("test stub not implemented")

    def create(self):
        self.create_mock("create")

    def destroy(self):
        self.destroy_mock("destroy")

    def mount_project(self):
        self.mount_project_mock("mount-project")

    def provision_project(self):
        raise NotImplementedError("test stub not implemented")

    def retrieve_snap(self):
        raise NotImplementedError("test stub not implemented")

    def retrieve_file(self, name: str, delete: bool = False):
        raise NotImplementedError("test stub not implemented")

    def shell(self):
        self.shell_mock("shell")


def get_project(base: str = "core16") -> Project:
    with open("snapcraft.yaml", "w") as snapcraft_file:
        print("name: project-name", file=snapcraft_file)
        print("base: {}".format(base), file=snapcraft_file)

    return Project(snapcraft_yaml_file_path="snapcraft.yaml")


class BaseProviderBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        self.instance_name = "snapcraft-project-name"

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider.SnapInjector"
        )
        self.snap_injector_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.project = get_project()

        self.echoer_mock = mock.Mock()


class BaseProviderWithBasesBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        self.instance_name = "snapcraft-project-name"

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider.SnapInjector"
        )
        self.snap_injector_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.echoer_mock = mock.Mock()


class MacBaseProviderWithBasesBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.instance_name = "snapcraft-project-name"

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider.SnapInjector"
        )
        self.snap_injector_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.project = get_project()

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider._get_platform",
            return_value="darwin",
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        self.echoer_mock = mock.Mock()
