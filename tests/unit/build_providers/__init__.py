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

from tests import unit
from snapcraft.internal.build_providers._base_provider import Provider


class ProviderImpl(Provider):
    def __init__(self, *, project, echoer, is_ephemeral=False):
        super().__init__(project=project, echoer=echoer, is_ephemeral=is_ephemeral)

        self.run_mock = mock.Mock()
        self.launch_mock = mock.Mock()
        self.mount_mock = mock.Mock()
        self.unmount_mock = mock.Mock()
        self.push_file_mock = mock.Mock()

    def _run(self, command, hide_output=False):
        self.run_mock(command)

    def _launch(self) -> None:
        self.launch_mock()

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


def get_project() -> Project:
    with open("snapcraft.yaml", "w") as snapcraft_file:
        print("name: project-name", file=snapcraft_file)

    return Project(snapcraft_yaml_file_path="snapcraft.yaml")


class BaseProviderBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.instance_name = "ridicoulus-hours"
        patcher = mock.patch("petname.Generate", return_value=self.instance_name)
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider.SnapInjector"
        )
        self.snap_injector_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.project = get_project()

        self.echoer_mock = mock.Mock()
