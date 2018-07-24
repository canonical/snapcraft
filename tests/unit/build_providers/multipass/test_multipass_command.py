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

import subprocess
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal.build_providers import errors
from snapcraft.internal.build_providers._multipass import MultipassCommand
from tests import unit


class MultipassCommandBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("signal.signal")
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("shutil.which", return_value=["multipass"])
        self.which_mock = patcher.start()
        self.addCleanup(patcher.stop)


class MultipassCommandGeneralTest(MultipassCommandBaseTest):
    def test_provider_name(self):
        self.assertThat(MultipassCommand.provider_name, Equals("multipass"))

    def test_multipass_command_missing_raises(self):
        self.which_mock.return_value = []

        self.assertRaises(errors.ProviderCommandNotFound, MultipassCommand)


class MultipassCommandPassthroughBaseTest(MultipassCommandBaseTest):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("subprocess.check_call")
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.multipass_command = MultipassCommand()
        self.instance_name = "stub-instance"


class MultipassCommandLaunchTest(MultipassCommandPassthroughBaseTest):
    def test_launch(self):
        self.multipass_command.launch(instance_name=self.instance_name, image="16.04")

        self.check_call_mock.assert_called_once_with(
            ["multipass", "launch", "16.04", "--name", self.instance_name]
        )
        self.check_output_mock.assert_not_called()

    def test_launch_with_remote(self):
        self.multipass_command.launch(
            instance_name=self.instance_name, image="18.04", remote="daily"
        )

        self.check_call_mock.assert_called_once_with(
            ["multipass", "launch", "daily:18.04", "--name", self.instance_name]
        )
        self.check_output_mock.assert_not_called()

    def test_launch_fails(self):
        # multipass can fail due to several reasons and will display the error
        # right above this exception message, it looks something like:
        #
        #   Creating a build environment named 'nonfilterable-mayola'
        #   failed to launch: failed to obtain exit status for remote process
        #   An error occurred when trying to launch the instance with 'multipass'.  # noqa: E501
        cmd = ["multipass", "launch", "18.04", "--name", self.instance_name]
        self.check_call_mock.side_effect = subprocess.CalledProcessError(1, cmd)

        self.assertRaises(
            errors.ProviderLaunchError,
            self.multipass_command.launch,
            instance_name=self.instance_name,
            image="18.04",
        )
        self.check_call_mock.assert_called_once_with(cmd)
        self.check_output_mock.assert_not_called()


class MultipassCommandStopTest(MultipassCommandPassthroughBaseTest):
    def test_stop(self):
        self.multipass_command.stop(instance_name=self.instance_name)

        self.check_call_mock.assert_called_once_with(
            ["multipass", "stop", self.instance_name]
        )
        self.check_output_mock.assert_not_called()

    def test_stop_fails(self):
        # multipass can fail due to several reasons and will display the error
        # right above this exception message.
        cmd = ["multipass", "stop", self.instance_name]
        self.check_call_mock.side_effect = subprocess.CalledProcessError(1, cmd)

        self.assertRaises(
            errors.ProviderStopError,
            self.multipass_command.stop,
            instance_name=self.instance_name,
        )
        self.check_call_mock.assert_called_once_with(cmd)
        self.check_output_mock.assert_not_called()


class MultipassCommandDeleteTest(MultipassCommandPassthroughBaseTest):
    def test_delete_implicit_purge(self):
        self.multipass_command.delete(instance_name=self.instance_name)

        self.check_call_mock.assert_called_once_with(
            ["multipass", "delete", self.instance_name, "--purge"]
        )
        self.check_output_mock.assert_not_called()

    def test_delete_no_purge(self):
        self.multipass_command.delete(instance_name=self.instance_name, purge=False)

        self.check_call_mock.assert_called_once_with(
            ["multipass", "delete", self.instance_name]
        )
        self.check_output_mock.assert_not_called()

    def test_delete_fails(self):
        # multipass can fail due to several reasons and will display the error
        # right above this exception message.
        cmd = ["multipass", "delete", self.instance_name, "--purge"]
        self.check_call_mock.side_effect = subprocess.CalledProcessError(1, cmd)

        self.assertRaises(
            errors.ProviderDeleteError,
            self.multipass_command.delete,
            instance_name=self.instance_name,
        )
        self.check_call_mock.assert_called_once_with(cmd)
        self.check_output_mock.assert_not_called()


class MultipassCommandMountTest(MultipassCommandPassthroughBaseTest):
    def test_mount(self):
        source = "mountpath"
        target = "{}/mountpoint".format(self.instance_name)
        self.multipass_command.mount(source=source, target=target)

        self.check_call_mock.assert_called_once_with(
            ["multipass", "mount", source, target]
        )
        self.check_output_mock.assert_not_called()

    def test_mount_fails(self):
        source = "mountpath"
        target = "{}/mountpoint".format(self.instance_name)
        cmd = ["multipass", "mount", source, target]
        self.check_call_mock.side_effect = subprocess.CalledProcessError(1, cmd)

        self.assertRaises(
            errors.ProviderMountError,
            self.multipass_command.mount,
            source=source,
            target=target,
        )
        self.check_call_mock.assert_called_once_with(cmd)
        self.check_output_mock.assert_not_called()


class MultipassCommandCopyFilesTest(MultipassCommandPassthroughBaseTest):
    def test_copy_files(self):
        source = "source-file"
        destination = "{}/destination-file".format(self.instance_name)
        self.multipass_command.copy_files(source=source, destination=destination)

        self.check_call_mock.assert_called_once_with(
            ["multipass", "copy-files", source, destination]
        )
        self.check_output_mock.assert_not_called()

    def test_copy_files_fails(self):
        # multipass can fail due to several reasons and will display the error
        # right above this exception message.
        source = "source-file"
        destination = "destination-file"
        cmd = ["multipass", "copy-files", source, destination]
        self.check_call_mock.side_effect = subprocess.CalledProcessError(1, cmd)

        self.assertRaises(
            errors.ProviderFileCopyError,
            self.multipass_command.copy_files,
            source=source,
            destination=destination,
        )
        self.check_call_mock.assert_called_once_with(cmd)
        self.check_output_mock.assert_not_called()


class MultipassCommandInfoTest(MultipassCommandPassthroughBaseTest):
    def test_info(self):
        self.multipass_command.info(instance_name=self.instance_name)

        self.check_output_mock.assert_called_once_with(
            ["multipass", "info", self.instance_name]
        )
        self.check_call_mock.assert_not_called()

    def test_info_with_format(self):
        self.multipass_command.info(
            instance_name=self.instance_name, output_format="json"
        )

        self.check_output_mock.assert_called_once_with(
            ["multipass", "info", self.instance_name, "--format", "json"]
        )
        self.check_call_mock.assert_not_called()

    def test_info_fails(self):
        # multipass can fail due to several reasons and will display the error
        # right above this exception message.
        cmd = ["multipass", "info", self.instance_name]
        self.check_output_mock.side_effect = subprocess.CalledProcessError(1, cmd)

        self.assertRaises(
            errors.ProviderInfoError,
            self.multipass_command.info,
            instance_name=self.instance_name,
        )
        self.check_output_mock.assert_called_once_with(cmd)
        self.check_call_mock.assert_not_called()
