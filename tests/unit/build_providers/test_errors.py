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

from testtools.matchers import Equals

from snapcraft.internal.build_providers import errors
from tests import unit


class ErrorFormattingTest(unit.TestCase):

    scenarios = [
        ('ProviderNotSupportedError', dict(
            exception=errors.ProviderNotSupportedError,
            kwargs=dict(provider='docker'),
            expected_message=(
                "The 'docker' provider is not supported, please choose a "
                "different one and try again."))),
        ('ProviderCommandNotFound', dict(
            exception=errors.ProviderCommandNotFound,
            kwargs=dict(command='multipass'),
            expected_message=(
                "'multipass' command not found: this command is necessary to "
                "build in this environment.\n"
                "Install 'multipass' or if already installed, ensure it is "
                "on the system PATH, and try again."))),
        ('ProviderLaunchError', dict(
            exception=errors.ProviderLaunchError,
            kwargs=dict(provider_name='multipass', exit_code=1),
            expected_message=(
                "An error occurred when trying to launch the instance "
                "with 'multipass': returned exit code 1.\n"
                "Ensure that 'multipass' is setup correctly and try "
                "again."))),
        ('ProviderStopError', dict(
            exception=errors.ProviderStopError,
            kwargs=dict(provider_name='multipass', exit_code=1),
            expected_message=(
                "An error occurred when trying to stop the instance "
                "with 'multipass': returned exit code 1.\n"
                "Ensure that 'multipass' is setup correctly and try "
                "again."))),
        ('ProviderDeleteError', dict(
            exception=errors.ProviderDeleteError,
            kwargs=dict(provider_name='multipass', exit_code=1),
            expected_message=(
                "An error occurred when trying to delete the instance "
                "with 'multipass': returned exit code 1.\n"
                "Ensure that 'multipass' is setup correctly and try "
                "again."))),
        ('ProviderExecError', dict(
            exception=errors.ProviderExecError,
            kwargs=dict(provider_name='multipass',
                        command=['snap', 'install', 'snapcraft', '--classic'],
                        exit_code=1),
            expected_message=(
                "An error occurred when trying to execute "
                "'snap install snapcraft --classic' with 'multipass': "
                "returned exit code 1."))),
        ('ProviderFileCopyError', dict(
            exception=errors.ProviderFileCopyError,
            kwargs=dict(provider_name='multipass', exit_code=1),
            expected_message=(
                "An error occurred when trying to copy files using "
                "'multipass': returned exit code 1."))),
        ('ProviderInfoError', dict(
            exception=errors.ProviderInfoError,
            kwargs=dict(provider_name='multipass', exit_code=1),
            expected_message=(
                "An error occurred when using 'multipass' to query the status "
                "of the instance: returned exit code 1."))),
        ('ProviderInfoDataKeyError', dict(
            exception=errors.ProviderInfoDataKeyError,
            kwargs=dict(provider_name='multipass',
                        missing_key='instance-name',
                        data=dict(info=dict(another_instance='data'))),
            expected_message=(
                "The data returned by 'multipass' was not expected. It is "
                "missing a required key "
                "'instance-name' in {'info': {'another_instance': 'data'}}."
            ))),
        ('ProviderBadDataError', dict(
            exception=errors.ProviderBadDataError,
            kwargs=dict(provider_name='multipass',
                        data='bad-json'),
            expected_message=(
                "The data returned by 'multipass' was not expected or in the "
                "wrong format: 'bad-json'."
            ))),
    ]

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)),
            Equals(self.expected_message))
