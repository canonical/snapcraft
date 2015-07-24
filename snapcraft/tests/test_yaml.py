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

import logging
import os
import tempfile
import unittest

import fixtures

from snapcraft import (
    common,
    dirs
)
from snapcraft.yaml import Config
from snapcraft.tests import TestCase


class TestYaml(TestCase):

    def make_snapcraft_yaml(self, content):
        tempdirObj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdirObj.cleanup)
        os.chdir(tempdirObj.name)
        with open("snapcraft.yaml", "w") as fp:
            fp.write(content)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_config_loads_plugins(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""parts:
  ubuntu:
    package: fswebcam
""")
        Config()
        mock_loadPlugin.assert_called_with("ubuntu", "ubuntu", {
            "package": "fswebcam",
        })

    def test_config_raises_on_missing_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        # no snapcraft.yaml
        with self.assertRaises(SystemExit) as raised:
            Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            "Could not find snapcraft.yaml.  Are you sure you're in the right directory?\n"
            "To start a new project, use 'snapcraft init'\n",
            fake_logger.output)

    def test_config_loop(self):
        dirs.setup_dirs()

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""parts:
  p1:
    plugin: ubuntu
    after: [p2]
  p2:
    plugin: ubuntu
    after: [p1]
""")
        with self.assertRaises(SystemExit) as raised:
            Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual('Circular dependency chain!\n', fake_logger.output)
