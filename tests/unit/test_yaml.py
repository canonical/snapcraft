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
from unittest.mock import (
    call,
    Mock,
    patch,
)

import snapcraft.common
from snapcraft.yaml import Config

from snapcraft.bin_snapcraft import setup_dirs
setup_dirs()


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

    @patch("snapcraft.common.log")
    def test_config_raises_on_missing_snapcraft_yaml(self, mock_log):
        # no snapcraft.yaml
        with self.assertRaises(SystemExit):
            Config()
        mock_log.assert_called_with("""Could not find snapcraft.yaml.  Are you sure you're in the right directory?
To start a new project, use 'snapcraft init'""")

    @patch("snapcraft.common.log")
    def test_config_loop(self, mock_log):
        self.makeSnapcraftYaml("""parts:
  p1:
    plugin: ubuntu
    after: [p2]
  p2:
    plugin: ubuntu
    after: [p1]
""")
        with self.assertRaises(SystemExit):
            Config()
        mock_log.assert_called_with("Circular dependency chain!")

    @patch("snapcraft.common.run")
    def test_assemble_uses_config_data(self, mock_run):
        self.makeSnapcraftYaml("snap:\n meta: meta-dir")

        mock_args = Mock()
        snapcraft.cmds.assemble(mock_args)

        mock_run.assert_has_calls([
            call("cp -arv meta-dir %s" % snapcraft.common.snapdir),
            call("snappy build %s" % snapcraft.common.snapdir),
        ])
