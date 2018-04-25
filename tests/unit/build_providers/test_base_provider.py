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

from testtools.matchers import Equals

from . import BaseProviderBaseTest
from snapcraft.internal.build_providers._base_provider import Provider


class ProviderImpl(Provider):

    def __init__(self, *, project, echoer):
        super().__init__(project=project, echoer=echoer)

        self.launch_mock = mock.Mock()
        self.run_mock = mock.Mock()

    def _run(self, command):
        self.run_mock(command)

    def _launch(self):
        self.launch_mock()


class BaseProviderTest(BaseProviderBaseTest):

    def test_initialize(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(provider.project, Equals(self.project))
        self.assertThat(provider.instance_name,
                        Equals(self.instance_name))
        self.assertThat(provider.project_dir, Equals('project-name'))
        self.assertThat(provider.snap_filename, Equals(
            'project-name_{}.snap'.format(self.project.deb_arch)))

    def test_initialize_snap_filename_with_version(self):
        self.project.info.version = 'test-version'

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(provider.snap_filename, Equals(
            'project-name_test-version_{}.snap'.format(self.project.deb_arch)))

    def test_launch_instance(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.launch_instance()

        provider.launch_mock.assert_any_call()
        provider.run_mock.assert_not_called()
        self.echoer_mock.info.assert_called_once_with(
            'Creating a build environment named '
            '{!r}'.format(self.instance_name))

    def test_setup_snapcraft(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.setup_snapcraft()

        provider.launch_mock.assert_not_called()
        provider.run_mock.assert_called_once_with(
            ['sudo', 'snap', 'install', 'snapcraft', '--classic'])
        self.echoer_mock.info.assert_called_once_with(
            'Setting up snapcraft in {!r}'.format(self.instance_name))
