# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

import copy
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
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = True
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
    def test_config_works_with_skills(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]

apps:
   app1:
     command: runme
     uses:
       - migration

uses:
  migration:
    type: migration-skill
    caps:
      - network-listener
""")

        config = snapcraft.yaml.Config()
        self.assertEqual(
            config.data,
            {'apps': {'app1': {'command': 'runme', 'slots': ['migration']}},
             'architectures': [snapcraft.common.get_arch()],
             'description': 'test',
             'name': 'test',
             'parts': {
                'part1': {
                    'snap': [], 'stage': [],
                    'stage-packages': ['fswebcam']}},
             'slots': {
                'migration': {
                    'caps': ['network-listener'],
                    'interface': 'old-security'}},
             'summary': 'test',
             'version': '1'})

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

    def test_config_adds_vcs_packages_to_build_packages(self):
        scenarios = [
            ('git://github.com/ubuntu-core/snapcraft.git', 'git'),
            ('lp:ubuntu-push', 'bzr'),
            ('https://github.com/ubuntu-core/snapcraft/archive/2.0.1.tar.gz',
             'tar'),
        ]
        yaml_t = """name: test
version: "1"
summary: test
description: test

parts:
  part1:
    source: {0}
    plugin: autotools
"""

        for s in scenarios:
            with self.subTest(key=(s[1])):
                self.make_snapcraft_yaml(yaml_t.format(s[0]))
                c = snapcraft.yaml.Config()

                self.assertTrue(
                    s[1] in c.build_tools,
                    '{} not found in {}'.format(s[1], c.build_tools))

    def test_config_adds_vcs_packages_to_build_packages_from_types(self):
        scenarios = [
            ('git', 'git'),
            ('hg', 'mercurial'),
            ('mercurial', 'mercurial'),
            ('bzr', 'bzr'),
            ('tar', 'tar'),
        ]
        yaml_t = """name: test
version: "1"
summary: test
description: test

parts:
  part1:
    source: http://something/somewhere
    source-type: {0}
    plugin: autotools
"""

        for s in scenarios:
            with self.subTest(key=(s[1])):
                self.make_snapcraft_yaml(yaml_t.format(s[0]))
                c = snapcraft.yaml.Config()

                self.assertTrue(
                    s[1] in c.build_tools,
                    '{} not found in {}'.format(s[1], c.build_tools))

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
                         "'name' is a required property")

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
                         "The 'name' property does not match the required "
                         "schema: 1 is not of type 'string'")

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_icon_extension(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.foo

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.message,
                         "'icon' must be either a .png or a .svg")

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_icon(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.mock_path_exists.return_value = False

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.png

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.message,
                         "Specified icon 'icon.png' does not exist")

    @unittest.mock.patch('snapcraft.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_apparmor(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.mock_path_exists.return_value = False

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

apps:
  foo:
    command: bar
    slots: [migration]

slots:
  migration:
    interface: old-security
    security-policy:
      apparmor: path/profile
      seccomp: path/profile

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.message,
                         "Specified file 'path/profile' does not exist")

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
            "The 'name' property does not match the required schema: "
            "'myapp@me_1.0' does not match '^[a-z0-9][a-z0-9+-]*$'")

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
            "'description' is a required property")

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
    plugin: nil
""")

        patcher = unittest.mock.patch('snapcraft.common.get_arch_triplet')
        mock_arch = patcher.start()
        mock_arch.return_value = 'x86_64-linux-gnu'
        self.addCleanup(patcher.stop)

    @unittest.mock.patch('snapcraft.common.get_snapdir')
    def test_config_snap_environment(self, mock_snapdir):
        mock_snapdir.return_value = 'foo'
        config = snapcraft.yaml.Config()
        environment = config.snap_env()
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

    @unittest.mock.patch('snapcraft.common.get_snapdir')
    def test_config_runtime_environment_ld(self, mock_snapdir):
        mock_snapdir.return_value = 'foo'

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
        environment = config.snap_env()

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

    @unittest.mock.patch('snapcraft.common.get_stagedir')
    def test_config_stage_environment(self, mock_stagedir):
        mock_stagedir.return_value = 'foo'

        config = snapcraft.yaml.Config()
        environment = config.stage_env()

        self.assertTrue('PATH="foo/bin:foo/usr/bin:$PATH"' in environment)
        self.assertTrue(
            'LD_LIBRARY_PATH="foo/lib:foo/usr/lib:'
            'foo/lib/x86_64-linux-gnu:foo/usr/lib/x86_64-linux-gnu:'
            '$LD_LIBRARY_PATH"' in environment)
        self.assertTrue(
            'CFLAGS="-Ifoo/include -Ifoo/usr/include '
            '-Ifoo/include/x86_64-linux-gnu '
            '-Ifoo/usr/include/x86_64-linux-gnu $CFLAGS"' in environment)
        self.assertTrue(
            'CPPFLAGS="-Ifoo/include -Ifoo/usr/include '
            '-Ifoo/include/x86_64-linux-gnu '
            '-Ifoo/usr/include/x86_64-linux-gnu $CPPFLAGS"' in environment)
        self.assertTrue(
            'LDFLAGS="-Lfoo/lib -Lfoo/usr/lib -Lfoo/lib/x86_64-linux-gnu '
            '-Lfoo/usr/lib/x86_64-linux-gnu $LDFLAGS"' in environment)
        self.assertTrue(
            'PKG_CONFIG_PATH=foo/lib/pkgconfig:'
            'foo/lib/x86_64-linux-gnu/pkgconfig:foo/usr/lib/pkgconfig:'
            'foo/usr/lib/x86_64-linux-gnu/pkgconfig:foo/usr/share/pkgconfig:'
            'foo/usr/local/lib/pkgconfig:'
            'foo/usr/local/lib/x86_64-linux-gnu/pkgconfig:'
            'foo/usr/local/share/pkgconfig:$PKG_CONFIG_PATH' in environment)
        self.assertTrue('PERL5LIB=foo/usr/share/perl5/' in environment)
        self.assertTrue('PKG_CONFIG_SYSROOT_DIR=foo' in environment)


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

                expected_message = ("The 'name' property does not match the "
                                    "required schema: '{}' does not match "
                                    "'^[a-z0-9][a-z0-9+-]*$'").format(name)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = (
            "The 'summary' property does not match the required schema: "
            "'{}' is too long").format(self.data['summary'])
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_valid_types(self):
        self.data['type'] = 'app'
        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_invalid_types(self):
        invalid_types = [
            'apps',
            'kernel',
            'framework',
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

                expected_message = (
                    "The 'type' property does not match the required "
                    "schema: '{}' is not one of ['app']").format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_valid_app_daemons(self):
        self.data['apps'] = {
            'service1': {'command': 'binary1 start', 'daemon': 'simple'},
            'service2': {
                'command': 'binary2',
                'stop-command': 'binary2 --stop',
                'daemon': 'simple'
            },
            'service3': {
                'command': 'binary3',
                'daemon': 'forking',
            },
            'service4': {
                'command': 'binary4',
                'daemon': 'simple',
                'restart-condition': 'always',
            }
        }

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_valid_restart_conditions(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
            }
        }
        valid_conditions = ['always', 'on-success', 'on-failure',
                            'on-abnormal', 'on-abort']

        for condition in valid_conditions:
            with self.subTest(key=condition):
                self.data['apps']['service1']['restart-condition'] = condition
                snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_invalid_restart_condition(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'restart-condition': 'on-watchdog',
            }
        }

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        self.assertEqual(
            "The 'restart-condition' property does not match the required "
            "schema: 'on-watchdog' is not one of ['on-success', "
            "'on-failure', 'on-abnormal', 'on-abort', 'always']",
            str(raised.exception))

    def test_invalid_app_names(self):
        invalid_names = {
            'qwe#rty': {'command': '1'},
            'qwe_rty': {'command': '1'},
            'que rty': {'command': '1'},
            'que  rty': {'command': '1'},
        }

        for t in invalid_names:
            data = self.data.copy()
            with self.subTest(key=t):
                data['apps'] = {t: invalid_names[t]}

                with self.assertRaises(
                        snapcraft.yaml.SnapcraftSchemaError) as raised:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                expected_message = (
                    "The 'apps' property does not match the required "
                    "schema: Additional properties are not allowed ('{}' "
                    "was unexpected)").format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_apps_required_properties(self):
        self.data['apps'] = {'service1': {}}

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = ("The 'service1' property does not match the "
                            "required schema: 'command' is a required "
                            "property")
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

        expected_message = ("The 'parts' property does not match the "
                            "required schema: Additional properties are not "
                            "allowed ('plugins' was unexpected)")
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_license_hook(self):
        self.data['license'] = 'LICENSE'

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_full_license_use(self):
        self.data['license'] = 'LICENSE'
        self.data['license-agreement'] = 'explicit'
        self.data['license-version'] = '1.0'

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_license_with_license_version(self):
        self.data['license'] = 'LICENSE'
        self.data['license-version'] = '1.0'

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_license_agreement_without_license_raises_exception(self):
        self.data['license-agreement'] = 'explicit'

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = "'license' is a dependency of 'license-agreement'"
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_license_version_without_license_raises_exception(self):
        self.data['license-version'] = '1.1'

        with self.assertRaises(snapcraft.yaml.SnapcraftSchemaError) as raised:
            snapcraft.yaml._validate_snapcraft_yaml(self.data)

        expected_message = "'license' is a dependency of 'license-version'"
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_valid_security_policy_for_apps(self):
        self.data['apps'] = {
            'app1': {
                'command': 'binary',
                'slots': ['file-migration'],
            },
        }
        self.data['slots'] = {
            'file-migration': {
                'interface': 'old-security',
                'security-policy': {
                    'seccomp': 'file.seccomp',
                    'apparmor': 'file.apparmor',
                },
            },
        }

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_valid_security_override_for_apps(self):
        self.data['apps'] = {
            'app1': {
                'command': 'binary',
                'slots': ['migration'],
            },
        }
        self.data['slots'] = {
            'migration': {
                'interface': 'old-security',
                'security-override': {
                    'read-paths': ['path1', 'path2'],
                    'write-paths': ['path1', 'path2'],
                    'abstractions': ['abstraction1', 'abstraction2'],
                    'syscalls': ['open', 'close'],
                },
            },
        }

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_valid_security_template_for_apps(self):
        self.data['apps'] = {
            'app1': {
                'command': 'binary',
                'slots': ['old-security'],
            },
        }
        self.data['slots'] = {
            'migration': {
                'interface': 'old-security',
                'security-template': 'unconfined',
            },
        }

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_valid_caps_for_apps(self):
        self.data['apps'] = {
            'app1': {
                'command': 'binary',
                'slots': ['migration'],
            },
        }
        self.data['slots'] = {
            'migration': {
                'interface': 'old-security',
                'caps': ['cap1', 'cap2'],
            },
        }

        snapcraft.yaml._validate_snapcraft_yaml(self.data)

    def test_invalid_security_override_combinations(self):
        self.data['apps'] = {
            'app1': {
                'command': 'binary',
                'slots': ['migration'],
            },
        }
        self.data['slots'] = {
            'migration': {
                'interface': 'old-security',
                'security-override': {
                    'read-paths': ['path1', 'path2'],
                    'write-paths': ['path1', 'path2'],
                    'abstractions': ['abstraction1', 'abstraction2'],
                    'syscalls': ['open', 'close'],
                },
                'caps': ['cap1', 'cap2'],
                'security-policy': {
                    'seccomp': 'file.seccomp',
                    'apparmor': 'file.apparmor',
                },
                'security-template': 'undefined',
            },
        }

        with self.subTest(key='all'):
            with self.assertRaises(Exception) as r:
                snapcraft.yaml._validate_snapcraft_yaml(self.data)

            self.assertTrue('is not valid under any of the given schemas'
                            in str(r.exception), str(r.exception))

        for sec in ['security-override', 'security-template', 'caps']:
            data = copy.deepcopy(self.data)
            del data['slots']['migration'][sec]
            with self.subTest(key=sec):
                with self.assertRaises(Exception) as r:
                    snapcraft.yaml._validate_snapcraft_yaml(data)

                self.assertTrue('is not valid under any of the given schemas'
                                in str(r.exception), str(r.exception))


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
