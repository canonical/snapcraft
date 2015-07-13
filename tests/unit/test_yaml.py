# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
import tempfile
import unittest

from snapcraft.yaml import Config


class TestYaml(unittest.TestCase):

    def makeSnapcraftYaml(self, content):
        tempdirObj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdirObj.cleanup)
        os.chdir(tempdirObj.name)
        with open("snapcraft.yaml", "w") as fp:
            fp.write(content)

    @unittest.mock.patch('snapcraft.yaml.Config.loadPlugin')
    def test_config_loads_plugins(self, mock_loadPlugin):
        self.makeSnapcraftYaml("""parts:
  ubuntu:
    package: fswebcam
""")
        Config()
        mock_loadPlugin.assert_called_with("ubuntu", "ubuntu", {
            "package": "fswebcam",
        })

    def test_config_raises_on_missing_snapcraft_yaml(self):
        # no snapcraft.yaml
        with self.assertRaises(SystemExit):
            Config()
