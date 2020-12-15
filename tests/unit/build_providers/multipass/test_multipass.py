# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2020 Canonical Ltd
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
from textwrap import dedent
from unittest import mock
from unittest.mock import call

import fixtures
import pytest
from testtools.matchers import Equals

from snapcraft.internal import steps
from snapcraft.internal.build_providers import _base_provider, errors
from snapcraft.internal.build_providers._multipass import Multipass, MultipassCommand
from snapcraft.internal.errors import SnapcraftEnvironmentError
from tests.unit.build_providers import BaseProviderBaseTest, get_project

_DEFAULT_INSTANCE_INFO = dedent(
    """\
    {
        "errors": [
        ],
        "info": {
            "snapcraft-project-name": {
                "disks": {
                    "sda1": {
                        "total": "5136297984",
                        "used": "1076072448"
                    }
                },
                "image_hash": "bc990872f070249450fc187d668bf65e2f87ffa3d7cf6e2934cb84b62af368e1",
                "image_release": "16.04 LTS",
                "ipv4": [
                    "10.74.70.141"
                ],
                "load": [
                    0,
                    0,
                    0
                ],
                "memory": {
                    "total": 1040318464,
                    "used": 42754048
                },
                "mounts": {
                },
                "release": "Ubuntu 16.04.4 LTS",
                "state": "RUNNING"
            }
        }
    }
    """
)  # noqa: E501


@pytest.fixture()
def multipass_cmd():
    """Fake MultipassCommand implementation."""

    def execute_effect(*, command, instance_name, hide_output):
        if hide_output:
            return None
        return b""

    patcher = mock.patch(
        "snapcraft.internal.build_providers._multipass." "_multipass.MultipassCommand",
        spec=MultipassCommand,
    )
    multipass_cmd_mock = patcher.start()

    multipass_cmd_mock().execute.side_effect = execute_effect

    # default data returned for info so launch is triggered
    multipass_cmd_mock().info.side_effect = [
        errors.ProviderInfoError(
            provider_name="multipass", exit_code=1, stderr=b"error"
        ),
        _DEFAULT_INSTANCE_INFO.encode(),
        _DEFAULT_INSTANCE_INFO.encode(),
    ]
    multipass_cmd_mock().exists.return_value = False

    yield multipass_cmd_mock

    patcher.stop()


class GetEnv(_base_provider.Provider):
    def _get_env_command(self):
        return ["env", "SNAPCRAFT_HAS_TTY=False"]


class MultipassTestImpl(Multipass, GetEnv):
    pass


class MultipassTest(BaseProviderBaseTest):
    def setUp(self):
        super().setUp()

        patcher = mock.patch(
            "snapcraft.internal.build_providers._multipass."
            "_multipass.MultipassCommand",
            spec=MultipassCommand,
        )
        self.multipass_cmd_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.internal.build_providers._base_provider.Provider.clean_project",
            return_value=True,
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("builtins.open", mock.mock_open())
        self.open_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # default data returned for info so launch is triggered
        self.multipass_cmd_mock().info.side_effect = [
            errors.ProviderInfoError(
                provider_name="multipass", exit_code=1, stderr=b"error"
            ),
            _DEFAULT_INSTANCE_INFO.encode(),
            _DEFAULT_INSTANCE_INFO.encode(),
        ]

    def test_ephemeral_instance_with_contextmanager(self):
        with MultipassTestImpl(
            project=self.project, echoer=self.echoer_mock, is_ephemeral=True
        ) as instance:
            instance.execute_step(steps.PULL)
            instance.execute_step(steps.BUILD)

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus="2",
            mem="2G",
            disk="256G",
            image="snapcraft:core16",
        )
        # Given SnapInjector is mocked, we only need to verify the commands
        # called from the Multipass class.
        self.multipass_cmd_mock().execute.assert_has_calls(
            [
                mock.call(
                    instance_name=self.instance_name,
                    hide_output=False,
                    command=[
                        "sudo",
                        "-H",
                        "-i",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "snapcraft",
                        "pull",
                    ],
                ),
                mock.call(
                    instance_name=self.instance_name,
                    hide_output=False,
                    command=[
                        "sudo",
                        "-H",
                        "-i",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "snapcraft",
                        "build",
                    ],
                ),
            ]
        )
        self.assertThat(self.multipass_cmd_mock().info.call_count, Equals(3))
        self.multipass_cmd_mock().info.assert_has_calls(
            [
                mock.call(instance_name=self.instance_name, output_format="json"),
                mock.call(instance_name=self.instance_name, output_format="json"),
            ]
        )
        self.multipass_cmd_mock().stop.assert_called_once_with(
            instance_name=self.instance_name, time=10
        )
        self.multipass_cmd_mock().delete.assert_called_once_with(
            instance_name=self.instance_name, purge=True
        )

    def test_launch_with_cpus_from_environment(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_CPU", "64")
        )

        instance = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)
        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus="64",
            mem="2G",
            disk="256G",
            image="snapcraft:core16",
        )

    def test_launch_for_type_base(self):
        self.project._snap_meta.name = "core18"
        self.project._snap_meta.type = "base"
        self.project._snap_meta.base = None

        instance = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)
        self.useFixture(
            fixtures.MockPatchObject(
                instance,
                "_get_instance_info",
                side_effect=[
                    errors.ProviderInfoError(
                        provider_name="multipass", exit_code=1, stderr=b"error"
                    ),
                    {},
                ],
            )
        )
        self.useFixture(fixtures.MockPatchObject(instance, "_mount_project",))

        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name="snapcraft-core18",
            cpus="2",
            mem="2G",
            disk="256G",
            image="snapcraft:core18",
        )

    def test_launch_with_ram_from_environment(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_MEMORY", "4G")
        )

        instance = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)
        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus="2",
            mem="4G",
            disk="256G",
            image="snapcraft:core16",
        )

    def test_launch_with_disk_from_environment(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_DISK", "400G")
        )

        instance = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)
        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus="2",
            mem="2G",
            disk="400G",
            image="snapcraft:core16",
        )

    def test_push_file(self):
        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass._push_file(source="src.txt", destination="dest.txt")

        self.multipass_cmd_mock().push_file.assert_called_once_with(
            destination="{}:dest.txt".format(self.instance_name),
            source=self.open_mock.return_value.__enter__(),
        )

        self.open_mock.assert_called_once_with("src.txt", "rb")
        self.open_mock.return_value.__exit__.assert_called()
        self.multipass_cmd_mock().info.assert_not_called()
        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_pull_file(self):
        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass.pull_file("src.txt", "dest.txt")

        self.multipass_cmd_mock().execute.assert_called_once_with(
            command=[
                "sudo",
                "-H",
                "-i",
                "env",
                "SNAPCRAFT_HAS_TTY=False",
                "test",
                "-f",
                "src.txt",
            ],
            hide_output=False,
            instance_name="snapcraft-project-name",
        )

        self.multipass_cmd_mock().pull_file.assert_called_once_with(
            destination=self.open_mock.return_value.__enter__(),
            source="{}:src.txt".format(self.instance_name),
        )

        self.open_mock.assert_called_once_with("dest.txt", "wb")
        self.open_mock.return_value.__exit__.assert_called()
        self.multipass_cmd_mock().info.assert_not_called()
        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_pull_and_delete_file(self):
        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass.pull_file("src.txt", "dest.txt", delete=True)

        self.multipass_cmd_mock().pull_file.assert_called_once_with(
            destination=self.open_mock.return_value.__enter__(),
            source="{}:src.txt".format(self.instance_name),
        )

        self.multipass_cmd_mock().execute.assert_has_calls(
            [
                mock.call(
                    command=[
                        "sudo",
                        "-H",
                        "-i",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "test",
                        "-f",
                        "src.txt",
                    ],
                    hide_output=False,
                    instance_name="snapcraft-project-name",
                ),
                mock.call(
                    command=[
                        "sudo",
                        "-H",
                        "-i",
                        "env",
                        "SNAPCRAFT_HAS_TTY=False",
                        "rm",
                        "src.txt",
                    ],
                    hide_output=False,
                    instance_name="snapcraft-project-name",
                ),
            ]
        )
        self.open_mock.assert_called_once_with("dest.txt", "wb")
        self.open_mock.return_value.__exit__.assert_called()
        self.multipass_cmd_mock().info.assert_not_called()
        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_instance_does_not_exist_on_destroy(self):
        # An error is raised if the queried image does not exist
        self.multipass_cmd_mock().info.side_effect = errors.ProviderInfoError(
            provider_name=self.instance_name, exit_code=2, stderr=b"error"
        )

        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass.destroy()

        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_destroy_instance_with_stop_delay(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_STOP_TIME", "60")
        )

        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass.create()
        multipass.destroy()

        self.multipass_cmd_mock().stop.assert_called_once_with(
            instance_name=self.instance_name, time=60
        )
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_destroy_instance_with_stop_delay_0(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_STOP_TIME", "0")
        )

        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass.create()
        multipass.destroy()

        self.multipass_cmd_mock().stop.assert_called_once_with(
            instance_name=self.instance_name
        )
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_destroy_instance_with_stop_delay_invalid(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_STOP_TIME", "A")
        )

        multipass = MultipassTestImpl(project=self.project, echoer=self.echoer_mock)

        multipass.create()

        self.assertRaises(SnapcraftEnvironmentError, multipass.destroy)

        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()


class TestMultipassWithBases:

    scenarios = (
        ("linux", dict(base="core16", expected_image="snapcraft:core16")),
        ("linux", dict(base="core18", expected_image="snapcraft:core18")),
    )

    def test_lifecycle(
        self, xdg_dirs, in_snap, snap_injector, multipass_cmd, base, expected_image
    ):
        with MultipassTestImpl(
            project=get_project(base=base), echoer=mock.Mock(), is_ephemeral=False
        ) as instance:
            instance.execute_step(steps.PULL)

        multipass_cmd().launch.assert_called_once_with(
            instance_name="snapcraft-project-name",
            cpus="2",
            mem="2G",
            disk="256G",
            image=expected_image,
        )

        multipass_cmd().execute.assert_called()

        for args, kwargs in multipass_cmd().execute.call_args_list:
            assert kwargs["command"][0:3] == [
                "sudo",
                "-H",
                "-i",
            ]
            assert kwargs["hide_output"] is False
            assert kwargs["instance_name"] == "snapcraft-project-name"

        assert multipass_cmd().mount.mock_calls == [
            mock.call(
                source=mock.ANY,
                target="snapcraft-project-name:/root/project",
                uid_map={str(os.getuid()): "0"},
                gid_map={str(os.getgid()): "0"},
            )
        ]
        multipass_cmd().umount.assert_not_called()

        assert multipass_cmd().info.call_count == 3
        multipass_cmd().info.assert_has_calls(
            [
                mock.call(instance_name="snapcraft-project-name", output_format="json"),
                mock.call(instance_name="snapcraft-project-name", output_format="json"),
            ]
        )
        assert multipass_cmd().push_file.mock_calls == [
            call(
                destination="snapcraft-project-name:/var/tmp/L3Jvb3QvLmJhc2hyYw==",
                source=mock.ANY,
            ),
            call(
                destination="snapcraft-project-name:/var/tmp/L2Jpbi9fc25hcGNyYWZ0X3Byb21wdA==",
                source=mock.ANY,
            ),
            call(
                destination="snapcraft-project-name:/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0",
                source=mock.ANY,
            ),
            call(
                destination="snapcraft-project-name:/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0LmQvZGVmYXVsdC5zb3VyY2Vz",
                source=mock.ANY,
            ),
            call(
                destination="snapcraft-project-name:/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0LmQvZGVmYXVsdC1zZWN1cml0eS5zb3VyY2Vz",
                source=mock.ANY,
            ),
            call(
                destination="snapcraft-project-name:/var/tmp/L2V0Yy9hcHQvYXB0LmNvbmYuZC8wMC1zbmFwY3JhZnQ=",
                source=mock.ANY,
            ),
        ]

        multipass_cmd().stop.assert_called_once_with(
            instance_name="snapcraft-project-name", time=10
        )
        multipass_cmd().delete.assert_not_called()

    def test_mount_prime_directory(
        self, xdg_dirs, in_snap, snap_injector, multipass_cmd, base, expected_image
    ):

        with MultipassTestImpl(
            project=get_project(base), echoer=mock.Mock(), is_ephemeral=False
        ) as instance:
            instance._mount_prime_directory()

        multipass_cmd().mount.assert_any_call(
            source=mock.ANY,
            target="snapcraft-project-name:/root/prime",
            uid_map={str(os.getuid()): "0"},
            gid_map={str(os.getgid()): "0"},
        )
