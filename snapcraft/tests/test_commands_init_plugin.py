# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import py_compile

from snapcraft.main import main
from snapcraft import tests

import fixtures


class InitPluginTestCase(tests.TestCase):

    def test_init_plugin_invalid_name(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        with self.assertRaises(SystemExit) as raised:
            main(['init-plugin', 'invalid-plugin'])

        self.assertEqual(1, raised.exception.code)
        self.assertEqual(
          fake_logger.output,
          "Plugin name must be valid Python identifier\n")

    def test_init_plugin_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        main(['init-plugin', 'testplugin'])

        plugin_path = os.path.join(self.parts_dir,
                                   'plugins',
                                   'x-testplugin.py')

        self.assertTrue(os.path.exists(plugin_path),
                        'Expected "parts/plugins/x-testplugin.py" to exist')

        self.assertEqual(
            fake_logger.output,
            "Created new plugin at {}\n".format(plugin_path))

    def test_plugin_compiles(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        main(['init-plugin', 'testplugin'])
        result = py_compile.compile(
                     os.path.join(self.parts_dir,
                                  'plugins',
                                  'x-testplugin.py'))

        self.assertRegex(result,
                         '{}/x-testplugin.*\.pyc'
                         .format(
                                 os.path.join(self.parts_dir,
                                              'plugins',
                                              '__pycache__')))

    def test_plugin_no_overwrite_existing(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        main(['init-plugin', 'testplugin'])

        with self.assertRaises(SystemExit) as raised:
            main(['init-plugin', 'testplugin'])

        self.assertEqual(1, raised.exception.code)

        plugin_path = os.path.join(self.parts_dir,
                                   'plugins',
                                   'x-testplugin.py')

        self.assertIn(
            "{} already exists. Not overwriting\n".format(plugin_path),
            fake_logger.output)

    def test_preexisting_file_fails(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        os.makedirs(self.parts_dir)
        plugin_dir = os.path.join(self.parts_dir, 'plugins')
        open(plugin_dir, 'w').close()

        with self.assertRaises(SystemExit) as raised:
            main(['init-plugin', 'testplugin'])

        self.assertEqual(1, raised.exception.code)
        self.assertEqual(fake_logger.output,
                         "{} is a file, can't be used as a "
                         "destination\n".format(plugin_dir))
