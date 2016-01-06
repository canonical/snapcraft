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
import unittest
import unittest.mock

import fixtures

import snapcraft.common
import snapcraft.yaml
from snapcraft import (
    dirs,
    tests,
)


class TestYaml(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch('os.path.exists')
        mock_wrap_exe = patcher.start()
        mock_wrap_exe.return_value = True
        self.addCleanup(patcher.stop)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    @unittest.mock.patch('snapcraft.wiki.Wiki.get_part')
    def test_config_loads_plugins(self, mock_get_part, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        snapcraft.yaml.Config()
        mock_loadPlugin.assert_called_with('part1', 'go', {
            'stage-packages': ['fswebcam'],
            'stage': [], 'snap': [],
        })

        self.assertFalse(mock_get_part.called)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    @unittest.mock.patch('snapcraft.wiki.Wiki.get_part')
    def test_config_loads_with_different_encodings(
            self, mock_get_part, mock_loadPlugin):
        content = """name: test
version: "1"
summary: test
description: ñoño test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
"""
        for enc in ['utf-8', 'utf-8-sig', 'utf-16']:
            with self.subTest(key=enc):
                self.make_snapcraft_yaml(content, encoding=enc)
                snapcraft.yaml.Config()

                mock_loadPlugin.assert_called_with('part1', 'go', {
                    'stage-packages': ['fswebcam'],
                    'stage': [], 'snap': [],
                })

                self.assertFalse(mock_get_part.called)

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    @unittest.mock.patch('snapcraft.wiki.Wiki.compose')
    def test_config_loads_part_from_wiki(self, mock_compose, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    stage-packages: [fswebcam]
""")
        mock_compose.return_value = {
            'plugin': 'go',
            'source': 'http://source.tar.gz',
        }

        snapcraft.yaml.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'source': 'http://source.tar.gz', 'stage': [], 'snap': []})

    @unittest.mock.patch('snapcraft.pluginhandler.load_plugin')
    @unittest.mock.patch('snapcraft.wiki.Wiki.get_part')
    def test_config_with_wiki_part_after(self, mock_get_part, mock_load):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    after:
      - part2wiki
    plugin: go
    stage-packages: [fswebcam]
""")

        def load_effect(*args, **kwargs):
            mock_part = unittest.mock.Mock()
            mock_part.code.build_packages = []
            mock_part.deps = []
            mock_part.name = args[0]

            return mock_part

        mock_load.side_effect = load_effect
        mock_get_part.return_value = {
            'plugin': 'go',
            'source': 'http://somesource'
        }

        snapcraft.yaml.Config()

        call1 = unittest.mock.call('part1', 'go', {
            'stage-packages': ['fswebcam'], 'stage': [], 'snap': []})
        call2 = unittest.mock.call('part2wiki', 'go', {
            'source': 'http://somesource'})

        mock_load.assert_has_calls([call1, call2])
        self.assertTrue(mock_get_part.called)

    def test_config_raises_on_missing_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        # no snapcraft.yaml
        with self.assertRaises(
                snapcraft.yaml.SnapcraftYamlFileError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.file, 'snapcraft.yaml')

    def test_config_loop(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  p1:
    plugin: tar-content
    source: .
    after: [p2]
  p2:
    plugin: tar-content
    source: .
    after: [p1]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftLogicError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(
            raised.exception.message,
            'circular dependency chain found in parts definition')

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_name(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""
version: "1"
summary: test
description: nothing

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.message,
                         '\'name\' is a required property')

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_name_as_number(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: 1
version: "1"
summary: test
description: nothing

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.message,
                         '1 is not of type \'string\'')

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_name_chars(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: myapp@me_1.0
version: "1"
summary: test
description: nothing

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(
            raised.exception.message,
            '\'myapp@me_1.0\' does not match \'^[a-z0-9][a-z0-9+-]*$\'')

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_description(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(
            raised.exception.message,
            '\'description\' is a required property')

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_tab_in_yaml(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
\tsummary: test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(
            raised.exception.message,
            'found character \'\\t\' that cannot start any token '
            'on line 2 of snapcraft.yaml')

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_config_expands_filesets(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
    filesets:
      wget:
        - /usr/lib/wget.so
        - /usr/bin/wget
      build-wget:
        - /usr/lib/wget.a
    stage:
      - $wget
      - $build-wget
    snap:
      - $wget
      - /usr/share/my-icon.png
""")
        snapcraft.yaml.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'snap': ['/usr/lib/wget.so', '/usr/bin/wget',
                     '/usr/share/my-icon.png'],
            'stage-packages': ['fswebcam'],
            'stage': ['/usr/lib/wget.so', '/usr/bin/wget', '/usr/lib/wget.a'],
        })


class TestYamlEnvironment(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")

    def test_config_runtime_environment(self):
        config = snapcraft.yaml.Config()
        environment = config.runtime_env('foo')
        self.assertTrue('PATH="foo/bin:foo/usr/bin:$PATH"' in environment)

        # Ensure that LD_LIBRARY_PATH is present and it contains only the
        # basics.
        paths = []
        for variable in environment:
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        arch = snapcraft.common.get_arch_triplet()
        for expected in ['foo/lib', 'foo/usr/lib', 'foo/lib/{}'.format(arch),
                         'foo/usr/lib/{}'.format(arch)]:
            self.assertTrue(expected in paths,
                            'Expected LD_LIBRARY_PATH to include "{}"'.format(
                                expected))

    def test_config_runtime_environment_ld(self):
        # Place a few ld.so.conf files in supported locations. We expect the
        # contents of these to make it into the LD_LIBRARY_PATH.
        os.makedirs('foo/usr/lib/my_arch/mesa/')
        with open('foo/usr/lib/my_arch/mesa/ld.so.conf', 'w') as f:
            f.write('/mesa')

        os.makedirs('foo/usr/lib/my_arch/mesa-egl/')
        with open('foo/usr/lib/my_arch/mesa-egl/ld.so.conf', 'w') as f:
            f.write('# Standalone comment\n')
            f.write('/mesa-egl')

        config = snapcraft.yaml.Config()
        environment = config.runtime_env('foo')

        # Ensure that the LD_LIBRARY_PATH includes all the above paths
        paths = []
        for variable in environment:
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        for expected in ['foo/mesa', 'foo/mesa-egl']:
            self.assertTrue(expected in paths,
                            'Expected LD_LIBRARY_PATH to include "{}"'.format(
                                expected))


class TestValidation(tests.TestCase):

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
            'summary': 'my summary less that 79 chars',
            'description': 'description which can be pretty long',
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

                with self.assertRaises(
                        snapcraft.yaml.SnapcraftSchemaError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = '\'{}\' is a required property'.format(key)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

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

                with self.assertRaises(
                        snapcraft.yaml.SnapcraftSchemaError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = (
                    '\'{}\' does not match \'^[a-z0-9][a-z0-9+-]*$\'').format(
                    name)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = '\'{}\' is too long'.format(self.data['summary'])
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

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

                with self.assertRaises(
                        snapcraft.yaml.SnapcraftSchemaError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = ('\'{}\' is not one of ' +
                                    '[\'app\', \'framework\']').format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_valid_services(self):
        self.data['services'] = {
            'service1': {'start': 'binary1 start'},
            'service2': {
                'start': 'binary2',
                'stop': 'binary2 --stop',
            },
        }

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_invalid_binary_names(self):
        invalid_names = {
            'qwe#rty': {'exec': '1'},
            'qwe_rty': {'exec': '1'},
            'que rty': {'exec': '1'},
            'que  rty': {'exec': '1'},
        }

        for t in invalid_names:
            data = self.data.copy()
            with self.subTest(key=t):
                data['binaries'] = {t: invalid_names[t]}

                with self.assertRaises(
                        snapcraft.yaml.SnapcraftSchemaError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = ('Additional properties are not allowed '
                                    '(\'{}\' was unexpected)').format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_invalid_service_names(self):
        invalid_names = {
            'qwe#rty': {'start': '1'},
            'qwe_rty': {'start': '1'},
            'que_rty': {'start': '1'},
            'quer  ty': {'start': '1'},
        }

        for t in invalid_names:
            data = self.data.copy()
            with self.subTest(key=t):
                data['services'] = {t: invalid_names[t]}

                with self.assertRaises(
                        snapcraft.yaml.SnapcraftSchemaError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = ('Additional properties are not allowed '
                                    '(\'{}\' was unexpected)').format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_services_required_properties(self):
        self.data['services'] = {'service1': {}}

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = '\'start\' is a required property'
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_schema_file_not_found(self):
        mock_the_open = unittest.mock.mock_open()
        mock_the_open.side_effect = FileNotFoundError()

        with unittest.mock.patch('snapcraft.yaml.open', mock_the_open,
                                 create=True):
            with self.assertRaises(
                    snapcraft.yaml.SnapcraftSchemaError) as raised:
                snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_path = os.path.join(snapcraft.common.get_schemadir(),
                                     'snapcraft.yaml')
        mock_the_open.assert_called_once_with(expected_path)
        expected_message = ('snapcraft validation file is missing from '
                            'installation path')
        self.assertEqual(raised.exception.message, expected_message)

    def test_icon_missing_is_valid_yaml(self):
        self.mock_path_exists.return_value = False

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_invalid_part_name_plugin_raises_exception(self):
        self.data['parts']['plugins'] = {'type': 'go'}

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = 'Additional properties are not allowed ' \
                           '(\'plugins\' was unexpected)'
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)


class TestFilesets(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.properties = {
            'filesets': {
                '1': ['1', '2', '3'],
                '2': [],
            }
        }

    def test_expand_var(self):
        self.properties['stage'] = ['$1']

        fs = snapcraft.yaml._expand_filesets_for('stage', self.properties)
        self.assertEqual(fs, ['1', '2', '3'])

    def test_no_expansion(self):
        self.properties['stage'] = ['1']

        fs = snapcraft.yaml._expand_filesets_for('stage', self.properties)
        self.assertEqual(fs, ['1'])

    def test_invalid_expansion(self):
        self.properties['stage'] = ['$3']

        with self.assertRaises(snapcraft.yaml.SnapcraftLogicError) as raised:
            snapcraft.yaml._expand_filesets_for('stage', self.properties)

        self.assertEqual(
            raised.exception.message,
            '\'$3\' referred to in the \'stage\' fileset but it is not '
            'in filesets')
