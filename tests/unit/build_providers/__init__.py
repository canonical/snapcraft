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

from snapcraft.project import Project

from tests import unit


class BaseProviderBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.instance_name = "ridicoulus-hours"
        patcher = mock.patch("petname.Generate", return_value=self.instance_name)
        patcher.start()
        self.addCleanup(patcher.stop)

        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: project-name
        """
            )
        )
        self.project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)

        self.echoer_mock = mock.Mock()

        patcher = mock.patch("snapcraft.internal.repo.snaps.get_assertion")
        self.get_assertion_mock = patcher.start()
        self.addCleanup(patcher.stop)
