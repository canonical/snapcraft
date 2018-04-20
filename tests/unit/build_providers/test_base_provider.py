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

from snapcraft.project import Project, ProjectInfo
from snapcraft.internal.build_providers._base_provider import BaseProvider
from tests import unit


class BaseProviderTest(unit.TestCase):

    def setUp(self):
        super().setUp()

        self.instance_name = 'ridicoulus-hours'
        patcher = mock.patch('petname.Generate',
                             return_value=self.instance_name)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.project = Project()
        self.project.info = ProjectInfo(dict(name='project-name'))

        self.echoer_mock = mock.Mock()
        self.executor_mock = mock.Mock()

    def test_initialize(self):
        base_provider = BaseProvider(project=self.project,
                                     executor=self.executor_mock,
                                     echoer=self.echoer_mock)

        self.assertThat(base_provider.project, Equals(self.project))
        self.assertThat(base_provider.instance_name,
                        Equals(self.instance_name))
        self.assertThat(base_provider.project_dir, Equals('project-name'))
        self.assertThat(base_provider.snap_filename, Equals(
            'project-name_{}.snap'.format(self.project.deb_arch)))

    def test_initialize_snap_filename_with_version(self):
        self.project.info.version = 'test-version'

        base_provider = BaseProvider(project=self.project,
                                     executor=self.executor_mock,
                                     echoer=self.echoer_mock)

        self.assertThat(base_provider.snap_filename, Equals(
            'project-name_test-version_{}.snap'.format(self.project.deb_arch)))

    def test_launch_instance(self):
        launch_mock = mock.Mock()

        base_provider = BaseProvider(project=self.project,
                                     executor=self.executor_mock,
                                     echoer=self.echoer_mock)
        base_provider.launch_instance(method=launch_mock,
                                      command=['multipass', 'launch'])

        launch_mock.assert_called_once_with(command=['multipass', 'launch'],
                                            instance_name=self.instance_name)
        self.echoer_mock.info.assert_called_once_with(
            'Creating a build environment named '
            '{!r}'.format(self.instance_name))

    def test_setup_snapcraft(self):
        base_provider = BaseProvider(project=self.project,
                                     executor=self.executor_mock,
                                     echoer=self.echoer_mock)
        base_provider.setup_snapcraft()

        self.executor_mock.assert_called_once_with(
            command=['sudo', 'snap', 'install', 'snapcraft', '--classic'],
            instance_name=self.instance_name,
        )
        self.echoer_mock.info.assert_called_once_with(
            'Setting up snapcraft in {!r}'.format(self.instance_name))
