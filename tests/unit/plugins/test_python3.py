# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import snapcraft
from snapcraft.plugins import python3
from tests import unit


class Python3PluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            source = "."
            requirements = ""
            constraints = ""
            python_packages = []
            process_dependency_links = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch("subprocess.check_call")
        self.mock_call = patcher.start()
        self.addCleanup(patcher.stop)

    def test_check_version(self):
        plugin = python3.Python3Plugin("test-part", self.options, self.project_options)
        self.assertThat(plugin.options.python_version, Equals("python3"))
