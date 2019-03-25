# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.internal.build_providers import errors
from tests import unit


class ErrorFormattingTest(unit.TestCase):

    scenarios = [
        (
            "ProviderNotSupportedError",
            dict(
                exception=errors.ProviderNotSupportedError,
                kwargs=dict(provider="docker"),
                expected_message=(
                    "The 'docker' provider is not supported, please choose a "
                    "different one and try again."
                ),
            ),
        ),
        (
            "ProviderNotFound",
            dict(
                exception=errors.ProviderNotFound,
                kwargs=dict(
                    provider="multipass",
                    prompt_installable=False,
                    error_message="not installed",
                ),
                expected_message=(
                    "You need 'multipass' set-up to build snaps: not installed."
                ),
            ),
        ),
        (
            "ProviderLaunchError (exit code)",
            dict(
                exception=errors.ProviderLaunchError,
                kwargs=dict(provider_name="multipass", exit_code=1),
                expected_message=(
                    "An error occurred with the instance when trying to launch with "
                    "'multipass': returned exit code 1.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderLaunchError (error message)",
            dict(
                exception=errors.ProviderLaunchError,
                kwargs=dict(
                    provider_name="multipass", error_message="failed to launch"
                ),
                expected_message=(
                    "An error occurred with the instance when trying to launch with "
                    "'multipass': failed to launch.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderLaunchError (exit code and error message)",
            dict(
                exception=errors.ProviderLaunchError,
                kwargs=dict(
                    provider_name="multipass",
                    exit_code=1,
                    error_message="failed to launch",
                ),
                expected_message=(
                    "An error occurred with the instance when trying to launch with "
                    "'multipass': returned exit code 1: failed to launch.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderStopError (exit code)",
            dict(
                exception=errors.ProviderStopError,
                kwargs=dict(provider_name="multipass", exit_code=1),
                expected_message=(
                    "An error occurred with the instance when trying to stop with "
                    "'multipass': returned exit code 1.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderStopError (error message)",
            dict(
                exception=errors.ProviderStopError,
                kwargs=dict(provider_name="multipass", error_message="failed to stop"),
                expected_message=(
                    "An error occurred with the instance when trying to stop with "
                    "'multipass': failed to stop.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderStopError (exit code and error message)",
            dict(
                exception=errors.ProviderStopError,
                kwargs=dict(
                    provider_name="multipass",
                    exit_code=1,
                    error_message="failed to stop",
                ),
                expected_message=(
                    "An error occurred with the instance when trying to stop with "
                    "'multipass': returned exit code 1: failed to stop.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderDeleteError (exit code)",
            dict(
                exception=errors.ProviderDeleteError,
                kwargs=dict(provider_name="multipass", exit_code=1),
                expected_message=(
                    "An error occurred with the instance when trying to delete with "
                    "'multipass': returned exit code 1.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderDeleteError (error message)",
            dict(
                exception=errors.ProviderDeleteError,
                kwargs=dict(
                    provider_name="multipass", error_message="failed to delete"
                ),
                expected_message=(
                    "An error occurred with the instance when trying to delete with "
                    "'multipass': failed to delete.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderDeleteError (exit code and error message)",
            dict(
                exception=errors.ProviderDeleteError,
                kwargs=dict(
                    provider_name="multipass",
                    exit_code=1,
                    error_message="failed to delete",
                ),
                expected_message=(
                    "An error occurred with the instance when trying to delete with "
                    "'multipass': returned exit code 1: failed to delete.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderShellError (exit code)",
            dict(
                exception=errors.ProviderShellError,
                kwargs=dict(provider_name="multipass", exit_code=1),
                expected_message=(
                    "An error occurred with the instance when trying to shell with "
                    "'multipass': returned exit code 1.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderShellError (error message)",
            dict(
                exception=errors.ProviderShellError,
                kwargs=dict(provider_name="multipass", error_message="failed to shell"),
                expected_message=(
                    "An error occurred with the instance when trying to shell with "
                    "'multipass': failed to shell.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderShellError (exit code and error message)",
            dict(
                exception=errors.ProviderShellError,
                kwargs=dict(
                    provider_name="multipass",
                    exit_code=1,
                    error_message="failed to shell",
                ),
                expected_message=(
                    "An error occurred with the instance when trying to shell with "
                    "'multipass': returned exit code 1: failed to shell.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderMountError (exit code)",
            dict(
                exception=errors.ProviderMountError,
                kwargs=dict(provider_name="multipass", exit_code=1),
                expected_message=(
                    "An error occurred with the instance when trying to mount with "
                    "'multipass': returned exit code 1.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderMountError (error message)",
            dict(
                exception=errors.ProviderMountError,
                kwargs=dict(provider_name="multipass", error_message="failed to mount"),
                expected_message=(
                    "An error occurred with the instance when trying to mount with "
                    "'multipass': failed to mount.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderMountError (exit code and error message)",
            dict(
                exception=errors.ProviderMountError,
                kwargs=dict(
                    provider_name="multipass",
                    exit_code=1,
                    error_message="failed to mount",
                ),
                expected_message=(
                    "An error occurred with the instance when trying to mount with "
                    "'multipass': returned exit code 1: failed to mount.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderFileCopyError (exit code)",
            dict(
                exception=errors.ProviderFileCopyError,
                kwargs=dict(provider_name="multipass", exit_code=1),
                expected_message=(
                    "An error occurred with the instance when trying to copy files with "
                    "'multipass': returned exit code 1.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderFileCopyError (error message)",
            dict(
                exception=errors.ProviderFileCopyError,
                kwargs=dict(
                    provider_name="multipass", error_message="failed to copy files"
                ),
                expected_message=(
                    "An error occurred with the instance when trying to copy files with "
                    "'multipass': failed to copy files.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderFileCopyError (exit code and error message)",
            dict(
                exception=errors.ProviderFileCopyError,
                kwargs=dict(
                    provider_name="multipass",
                    exit_code=1,
                    error_message="failed to copy files",
                ),
                expected_message=(
                    "An error occurred with the instance when trying to copy files with "
                    "'multipass': returned exit code 1: failed to copy files.\n"
                    "Ensure that 'multipass' is setup correctly and try again."
                ),
            ),
        ),
        (
            "ProviderExecError",
            dict(
                exception=errors.ProviderExecError,
                kwargs=dict(
                    provider_name="multipass",
                    command=["snap", "install", "snapcraft", "--classic"],
                    exit_code=1,
                ),
                expected_message=(
                    "An error occurred when trying to execute "
                    "'snap install snapcraft --classic' with 'multipass': "
                    "returned exit code 1."
                ),
            ),
        ),
        (
            "ProviderInfoError",
            dict(
                exception=errors.ProviderInfoError,
                kwargs=dict(provider_name="multipass", exit_code=1, stderr=b"error"),
                expected_message=(
                    "An error occurred when using 'multipass' to query the status "
                    "of the instance: returned exit code 1: error."
                ),
            ),
        ),
        (
            "ProviderInstanceNotFoundError",
            dict(
                exception=errors.ProviderInstanceNotFoundError,
                kwargs=dict(instance_name="test-build"),
                expected_message=("Cannot find an instance named 'test-build'."),
            ),
        ),
        (
            "ProviderInfoDataKeyError",
            dict(
                exception=errors.ProviderInfoDataKeyError,
                kwargs=dict(
                    provider_name="multipass",
                    missing_key="instance-name",
                    data=dict(info=dict(another_instance="data")),
                ),
                expected_message=(
                    "The data returned by 'multipass' was not expected. It is "
                    "missing a required key "
                    "'instance-name' in {'info': {'another_instance': 'data'}}."
                ),
            ),
        ),
        (
            "ProviderBadDataError",
            dict(
                exception=errors.ProviderBadDataError,
                kwargs=dict(provider_name="multipass", data="bad-json"),
                expected_message=(
                    "The data returned by 'multipass' was not expected or in the "
                    "wrong format: 'bad-json'."
                ),
            ),
        ),
        (
            "ProviderCommunicationError",
            dict(
                exception=errors.ProviderCommunicationError,
                kwargs=dict(provider_name="multipass"),
                expected_message=(
                    "An error occurred when trying to communicate with the "
                    "'multipass' provider."
                ),
            ),
        ),
    ]

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )
