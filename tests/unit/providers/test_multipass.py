# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

from pathlib import Path
from unittest.mock import call

import pytest

from snapcraft import providers


@pytest.fixture
def multipass_instance_mock(mocker):
    yield mocker.patch(
        "craft_providers.multipass._launch.MultipassInstance", autospec=True
    )


class TestMultipassLaunchedEnvironment:
    """Multipass provider instance setup and launching."""

    def test_mount_project(self, mocker, new_dir, multipass_instance_mock):
        mocker.patch(
            "snapcraft.providers._multipass.multipass.launch",
            return_value=multipass_instance_mock,
        )

        prov = providers.MultipassProvider()

        with prov.launched_environment(
            project_name="test",
            project_path=new_dir,
            base="core22",
            bind_ssh=False,
            build_on="test",
            build_to="test",
        ):
            assert multipass_instance_mock.mount.mock_calls == [
                call(host_source=new_dir, target=Path("/root/project")),
            ]

    def test_bind_ssh(self, mocker, new_dir, multipass_instance_mock):
        mocker.patch(
            "snapcraft.providers._multipass.multipass.launch",
            return_value=multipass_instance_mock,
        )

        prov = providers.MultipassProvider()

        with prov.launched_environment(
            project_name="test",
            project_path=new_dir,
            base="core22",
            bind_ssh=True,
            build_on="test",
            build_to="test",
        ):
            assert multipass_instance_mock.mount.mock_calls == [
                call(host_source=new_dir, target=Path("/root/project")),
                call(host_source=Path.home() / ".ssh", target=Path("/root/.ssh")),
            ]
