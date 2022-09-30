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
from unittest.mock import call, patch

import pytest

from snapcraft import providers


@pytest.fixture
def multipass_instance_mock(mocker):
    yield mocker.patch(
        "craft_providers.multipass._launch.MultipassInstance", autospec=True
    )


@pytest.fixture()
def mock_multipass_is_installed():
    with patch(
        "craft_providers.multipass.is_installed", return_value=True
    ) as mock_is_installed:
        yield mock_is_installed


@pytest.fixture()
def mock_multipass_delete():
    with patch("craft_providers.multipass.MultipassInstance.delete") as mock_delete:
        yield mock_delete


@pytest.fixture()
def mock_multipass_exists():
    with patch(
        "craft_providers.multipass.MultipassInstance.exists", return_value=True
    ) as mock_exists:
        yield mock_exists


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
            build_for="test",
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
            build_for="test",
        ):
            assert multipass_instance_mock.mount.mock_calls == [
                call(host_source=new_dir, target=Path("/root/project")),
                call(host_source=Path.home() / ".ssh", target=Path("/root/.ssh")),
            ]

    def test_get_command_environment(self, mocker, new_dir, multipass_instance_mock):
        """Verify launched_environment calls get_command_environment."""
        mocker.patch(
            "snapcraft.providers._multipass.multipass.launch",
            return_value=multipass_instance_mock,
        )
        mock_get_command_environment = mocker.patch(
            "snapcraft.providers._multipass.get_command_environment",
        )

        prov = providers.MultipassProvider()

        with prov.launched_environment(
            project_name="test",
            project_path=new_dir,
            base="core22",
            bind_ssh=True,
            build_on="test",
            build_for="test",
            http_proxy="test-http",
            https_proxy="test-https",
        ):
            assert mock_get_command_environment.mock_calls == [
                call(http_proxy="test-http", https_proxy="test-https")
            ]
