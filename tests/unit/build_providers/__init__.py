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
from snapcraft.project._project_info import ProjectInfo

from tests import unit


class BaseProviderBaseTest(unit.TestCase):

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
