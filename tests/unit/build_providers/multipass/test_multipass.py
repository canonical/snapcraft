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

from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import Equals

from tests.unit.build_providers import (
    BaseProviderBaseTest,
    BaseProviderWithBasesBaseTest,
    get_project,
)
from snapcraft.internal import steps
from snapcraft.internal.build_providers import errors
from snapcraft.internal.build_providers._multipass import Multipass, MultipassCommand


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

        # default data returned for info so launch is triggered
        self.multipass_cmd_mock().info.side_effect = [
            errors.ProviderInfoError(
                provider_name="multipass", exit_code=1, stderr=b"error"
            ),
            _DEFAULT_INSTANCE_INFO.encode(),
        ]

    def test_ephemeral_instance_with_contextmanager(self):
        with Multipass(
            project=self.project, echoer=self.echoer_mock, is_ephemeral=True
        ) as instance:
            instance.provision_project("source.tar")
            instance.build_project()
            instance.retrieve_snap()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus=mock.ANY,
            mem="2G",
            disk="256G",
            image="16.04",
            cloud_init=mock.ANY,
        )
        # Given SnapInjector is mocked, we only need to verify the commands
        # called from the Multipass class.
        self.multipass_cmd_mock().execute.assert_has_calls(
            [
                mock.call(
                    instance_name=self.instance_name, command=["mkdir", "~/project"]
                ),
                mock.call(
                    instance_name=self.instance_name,
                    command=["tar", "-xvf", "source.tar", "-C", "~/project"],
                ),
                mock.call(
                    instance_name=self.instance_name,
                    command=[
                        "snapcraft",
                        "snap",
                        "--output",
                        "project-name_{}.snap".format(self.project.deb_arch),
                    ],
                ),
            ]
        )
        self.assertThat(self.multipass_cmd_mock().info.call_count, Equals(2))
        self.multipass_cmd_mock().info.assert_has_calls(
            [
                mock.call(instance_name=self.instance_name, output_format="json"),
                mock.call(instance_name=self.instance_name, output_format="json"),
            ]
        )

        self.multipass_cmd_mock().copy_files.assert_has_calls(
            [
                mock.call(
                    destination="{}:source.tar".format(self.instance_name),
                    source="source.tar",
                ),
                mock.call(
                    destination="project-name_{}.snap".format(self.project.deb_arch),
                    source="{}:~/project/project-name_{}.snap".format(
                        self.instance_name, self.project.deb_arch
                    ),
                ),
            ]
        )
        self.multipass_cmd_mock().stop.assert_called_once_with(
            instance_name=self.instance_name
        )
        self.multipass_cmd_mock().delete.assert_called_once_with(
            instance_name=self.instance_name, purge=True
        )

    def test_launch_with_cpus_from_environment(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_CPU", "64")
        )

        instance = Multipass(project=self.project, echoer=self.echoer_mock)
        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus="64",
            mem="2G",
            disk="256G",
            image="16.04",
            cloud_init=mock.ANY,
        )

    def test_launch_with_ram_from_environment(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_MEMORY", "4G")
        )

        instance = Multipass(project=self.project, echoer=self.echoer_mock)
        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus=mock.ANY,
            mem="4G",
            disk="256G",
            image="16.04",
            cloud_init=mock.ANY,
        )

    def test_launch_with_disk_from_environment(self):
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT_DISK", "400G")
        )

        instance = Multipass(project=self.project, echoer=self.echoer_mock)
        instance.create()

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus=mock.ANY,
            mem="2G",
            disk="400G",
            image="16.04",
            cloud_init=mock.ANY,
        )

    def test_provision_project(self):
        multipass = Multipass(project=self.project, echoer=self.echoer_mock)

        # In the real world, MultipassCommand would return an error when
        # calling this on an instance that does not exist.
        multipass.provision_project("source.tar")

        self.multipass_cmd_mock().execute.assert_has_calls(
            [
                mock.call(
                    instance_name=self.instance_name, command=["mkdir", "~/project"]
                ),
                mock.call(
                    instance_name=self.instance_name,
                    command=["tar", "-xvf", "source.tar", "-C", "~/project"],
                ),
            ]
        )
        self.multipass_cmd_mock().copy_files.assert_called_once_with(
            destination="{}:source.tar".format(self.instance_name), source="source.tar"
        )

        self.multipass_cmd_mock().launch.assert_not_called()
        self.multipass_cmd_mock().info.assert_not_called()
        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_build_project(self):
        multipass = Multipass(project=self.project, echoer=self.echoer_mock)

        # In the real world, MultipassCommand would return an error when
        # calling this on an instance that does not exist.
        multipass.build_project()

        self.multipass_cmd_mock().execute.assert_called_once_with(
            instance_name=self.instance_name,
            command=[
                "snapcraft",
                "snap",
                "--output",
                "project-name_{}.snap".format(self.project.deb_arch),
            ],
        )

        self.multipass_cmd_mock().copy_files.assert_not_called()
        self.multipass_cmd_mock().launch.assert_not_called()
        self.multipass_cmd_mock().info.assert_not_called()
        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_retrieve_snap(self):
        multipass = Multipass(project=self.project, echoer=self.echoer_mock)

        # In the real world, MultipassCommand would return an error when
        # calling this on an instance that does not exist.
        multipass.retrieve_snap()

        self.multipass_cmd_mock().copy_files.assert_called_once_with(
            destination="project-name_{}.snap".format(self.project.deb_arch),
            source="{}:~/project/project-name_{}.snap".format(
                self.instance_name, self.project.deb_arch
            ),
        )

        self.multipass_cmd_mock().execute.assert_not_called()
        self.multipass_cmd_mock().launch.assert_not_called()
        self.multipass_cmd_mock().info.assert_not_called()
        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()

    def test_instance_does_not_exist_on_destroy(self):
        # An error is raised if the queried image does not exist
        self.multipass_cmd_mock().info.side_effect = errors.ProviderInfoError(
            provider_name=self.instance_name, exit_code=2, stderr=b"error"
        )

        multipass = Multipass(project=self.project, echoer=self.echoer_mock)

        multipass.destroy()

        self.multipass_cmd_mock().stop.assert_not_called()
        self.multipass_cmd_mock().delete.assert_not_called()


class MultipassWithBasesTest(BaseProviderWithBasesBaseTest):

    scenarios = (
        ("linux", dict(platform="linux", base="core16", expected_image="16.04")),
        ("linux", dict(platform="linux", base="core18", expected_image="18.04")),
        ("linux no base", dict(platform="linux", base=None, expected_image="16.04")),
        ("darwin", dict(platform="darwin", base="core18", expected_image="18.04")),
        ("darwin", dict(platform="darwin", base="core16", expected_image="16.04")),
    )

    def setUp(self):
        super().setUp()

        patcher = mock.patch(
            "snapcraft.internal.build_providers._multipass._multipass._get_platform",
            return_value=self.platform,
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            "snapcraft.internal.build_providers._multipass."
            "_multipass.MultipassCommand",
            spec=MultipassCommand,
        )
        self.multipass_cmd_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.project = get_project(base=self.base)

        def execute_effect(*, command, instance_name, hide_output):
            if command == ["printenv", "HOME"]:
                return "/home/multipass".encode()
            elif hide_output:
                return None
            else:
                return b""

        self.multipass_cmd_mock().execute.side_effect = execute_effect

        # default data returned for info so launch is triggered
        self.multipass_cmd_mock().info.side_effect = [
            errors.ProviderInfoError(
                provider_name="multipass", exit_code=1, stderr=b"error"
            ),
            _DEFAULT_INSTANCE_INFO.encode(),
        ]

    def test_lifecycle(self):
        with Multipass(
            project=self.project, echoer=self.echoer_mock, is_ephemeral=False
        ) as instance:
            instance.mount_project()
            instance.execute_step(steps.PULL)

        self.multipass_cmd_mock().launch.assert_called_once_with(
            instance_name=self.instance_name,
            cpus=mock.ANY,
            mem="2G",
            disk="256G",
            image=self.expected_image,
            cloud_init=mock.ANY,
        )
        self.multipass_cmd_mock().execute.assert_has_calls(
            [
                mock.call(
                    instance_name=self.instance_name,
                    hide_output=True,
                    command=["printenv", "HOME"],
                ),
                mock.call(
                    instance_name=self.instance_name,
                    hide_output=False,
                    command=["snapcraft", "pull"],
                ),
            ]
        )
        self.multipass_cmd_mock().mount.assert_called_once_with(
            source=mock.ANY,
            target="{}:{}".format(self.instance_name, "/home/multipass/project"),
        )
        self.multipass_cmd_mock().umount.assert_not_called()
        self.assertThat(self.multipass_cmd_mock().info.call_count, Equals(2))
        self.multipass_cmd_mock().info.assert_has_calls(
            [
                mock.call(instance_name=self.instance_name, output_format="json"),
                mock.call(instance_name=self.instance_name, output_format="json"),
            ]
        )
        self.multipass_cmd_mock().copy_files.assert_not_called()
        self.multipass_cmd_mock().stop.assert_called_once_with(
            instance_name=self.instance_name
        )
        self.multipass_cmd_mock().delete.assert_not_called()


class MultipassUnsupportedPlatform(BaseProviderWithBasesBaseTest):
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
            "snapcraft.internal.build_providers._multipass._multipass._get_platform",
            return_value="darwin",
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        # default data returned for info so launch is triggered
        self.multipass_cmd_mock().info.side_effect = [
            errors.ProviderInfoError(
                provider_name="multipass", exit_code=1, stderr=b"error"
            ),
            _DEFAULT_INSTANCE_INFO.encode(),
        ]

    def test_plaftorm_and_base_unsupported(self):
        project = get_project(base="core17")

        multipass = Multipass(project=project, echoer=self.echoer_mock)

        self.assertRaises(errors.UnsupportedHostError, multipass.create)
