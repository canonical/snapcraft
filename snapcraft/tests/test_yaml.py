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

import jsonschema
import logging
import os
import tempfile
import unittest
import unittest.mock

import fixtures

import snapcraft.common
import snapcraft.yaml
from snapcraft import dirs
from snapcraft.tests import TestCase


class TestYaml(TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch('os.path.exists')
        mock_wrap_exe = patcher.start()
        mock_wrap_exe.return_value = True
        self.addCleanup(patcher.stop)

    def make_snapcraft_yaml(self, content):
        tempdirObj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdirObj.cleanup)
        os.chdir(tempdirObj.name)
        with open("snapcraft.yaml", "w") as fp:
            fp.write(content)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_config_loads_plugins(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
vendor: me <me@me.com>
summary: test
description: test
icon: my-icon.png

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        snapcraft.yaml.Config()
        mock_loadPlugin.assert_called_with("part1", "go", {
            "stage-packages": ["fswebcam"],
        })

    def test_config_raises_on_missing_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        # no snapcraft.yaml
        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            "Could not find snapcraft.yaml.  Are you sure you're in the right directory?\n"
            "To start a new project, use 'snapcraft init'\n",
            fake_logger.output)

    def test_config_loop(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
vendor: me <me@me.com>
summary: test
description: test
icon: my-icon.png

parts:
  p1:
    plugin: go
    after: [p2]
  p2:
    plugin: go
    after: [p1]
""")
        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual('Circular dependency chain!\n', fake_logger.output)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_name(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""
version: "1"
vendor: me <me@me.com>
summary: test
description: nothing
icon: my-icon.png

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'Issues while validating snapcraft.yaml: \'name\' is a required property\n',
            fake_logger.output)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_name_as_number(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: 1
version: "1"
vendor: me <me@me.com>
summary: test
description: nothing
icon: my-icon.png

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'Issues while validating snapcraft.yaml: 1 is not of type \'string\'\n',
            fake_logger.output)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_name_chars(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: myapp@me_1.0
version: "1"
vendor: me <me@me.com>
summary: test
description: nothing
icon: my-icon.png

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'Issues while validating snapcraft.yaml: \'myapp@me_1.0\' does not match \'^[a-z0-9][a-z0-9+-]*$\'\n',
            fake_logger.output)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_description(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
vendor: me <me@me.com>
summary: test
icon: my-icon.png

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SystemExit) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'Issues while validating snapcraft.yaml: \'description\' is a required property\n',
            fake_logger.output)


class TestValidation(TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch('os.path.exists')
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = True
        self.addCleanup(patcher.stop)

        self.data = {
            'name': 'my-package-1',
            'version': '1.0-snapcraft1~ppa1',
            'vendor': 'Me <me@me.com>',
            'summary': 'my summary less that 79 chars',
            'description': 'description which can be pretty long',
            'icon': 'my-icon.png',
            'parts': {
                'part1': {
                    'plugin': 'project',
                },
            },
        }

    def test_required_properties(self):
        for key in self.data:
            data = self.data.copy()
            with self.subTest(key=key):
                del data[key]

                with self.assertRaises(jsonschema.ValidationError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = '\'{}\' is a required property'.format(key)
                self.assertEqual(raised.exception.message, expected_message, msg=data)

    def test_invalid_names(self):
        invalid_names = [
            'package@awesome',
            'something.another',
            '_hideme',
        ]

        for name in invalid_names:
            data = self.data.copy()
            with self.subTest(key=name):
                data['name'] = name

                with self.assertRaises(jsonschema.ValidationError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = '\'{}\' does not match \'^[a-z0-9][a-z0-9+-]*$\''.format(name)
                self.assertEqual(raised.exception.message, expected_message, msg=data)

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        with self.assertRaises(jsonschema.ValidationError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = '\'{}\' is too long'.format(self.data['summary'])
        self.assertEqual(raised.exception.message, expected_message, msg=self.data)

    def test_valid_types(self):
        self.data['type'] = 'app'
        snapcraft.yaml._validate_snapcraft_yaml(self.data)

        self.data['type'] = 'framework'
        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_invalid_types(self):
        invalid_types = [
            'apps',
            'kernel',
            'platform',
            'oem',
            'os',
        ]

        for t in invalid_types:
            data = self.data.copy()
            with self.subTest(key=t):
                data['type'] = t

                with self.assertRaises(jsonschema.ValidationError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = '\'{}\' is not one of [\'app\', \'framework\']'.format(t)
                self.assertEqual(raised.exception.message, expected_message, msg=data)

    def test_valid_services(self):
        self.data['services'] = [
            {
                'name': 'service1',
                'start': 'binary1 start',
            },
            {
                'name': 'service2',
                'start': 'binary2',
                'stop': 'binary2 --stop',
            },
            {
                'name': 'service3',
            }
        ]

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_services_required_properties(self):
        self.data['services'] = [
            {
                'start': 'binary1 start',
            }
        ]

        with self.assertRaises(jsonschema.ValidationError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = '\'name\' is a required property'
        self.assertEqual(raised.exception.message, expected_message, msg=self.data)

    def test_schema_file_not_found(self):
        mock_the_open = unittest.mock.mock_open()
        mock_the_open.side_effect = FileNotFoundError()

        with unittest.mock.patch('snapcraft.yaml.open', mock_the_open, create=True):
            with self.assertRaises(snapcraft.yaml.SchemaNotFoundError) as raised:
                snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_path = os.path.join(snapcraft.common.get_schemadir(), 'snapcraft.yaml')
        mock_the_open.assert_called_once_with(expected_path)
        expected_message = 'Schema is missing, could not validate snapcraft.yaml, check installation'
        self.assertEqual(raised.exception.message, expected_message)

    def test_icon_missing(self):
        self.mock_path_exists.return_value = False

        with self.assertRaises(jsonschema.ValidationError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = '\'my-icon.png\' is not a \'icon-path\''
        self.assertEqual(raised.exception.message, expected_message, msg=self.data)
