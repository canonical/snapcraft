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

import logging
import os
import subprocess
import sys
import tempfile
import unittest
import unittest.mock

import fixtures

import snapcraft
from snapcraft.internal import dirs, parts
from snapcraft.internal import project_loader
from snapcraft import tests
from snapcraft.tests import fixture_setup
from snapcraft._schema import SnapcraftSchemaError


class TestYaml(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch(
            'snapcraft.internal.project_loader._get_snapcraft_yaml')
        self.mock_get_yaml = patcher.start()
        self.mock_get_yaml.return_value = 'snapcraft.yaml'
        self.addCleanup(patcher.stop)
        self.part_schema = project_loader.Validator().part_schema
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_config_loads_plugins(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        project_loader.Config()
        mock_loadPlugin.assert_called_with('part1', 'go', {
            'stage-packages': ['fswebcam'],
            'stage': [], 'snap': [],
        })

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_config_loads_with_different_encodings(
            self, mock_loadPlugin):
        content = """name: test
version: "1"
summary: test
description: ñoño test
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
"""
        for enc in ['utf-8', 'utf-8-sig', 'utf-16']:
            with self.subTest(key=enc):
                self.make_snapcraft_yaml(content, encoding=enc)
                project_loader.Config()

                mock_loadPlugin.assert_called_with('part1', 'go', {
                    'stage-packages': ['fswebcam'],
                    'stage': [], 'snap': [],
                })

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_config_composes_with_remote_parts(self, mock_loadPlugin):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    stage-packages: [fswebcam]
""")

        parts.update()
        project_loader.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'source': 'http://source.tar.gz', 'stage-packages': ['fswebcam'],
            'stage': [], 'snap': []})

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_config_composes_with_remote_subpart(self, mock_loadPlugin):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    stage-packages: [fswebcam]
""")

        parts.update()
        project_loader.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'source': 'http://source.tar.gz', 'stage-packages': ['fswebcam'],
            'stage': [], 'snap': []})

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_chaining_remotes_not_locally_declared(self, mock_loadPlugin):
        """Test to verify we can load non locally declared chained remotes."""
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    after: [curl]
  curl:
    after: [long-described-part]
  part2:
    plugin: nil
""")

        parts.update()
        project_loader.Config()

    def test_config_composes_with_a_non_existent_remote_part(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  non-existing-part:
    stage-packages: [fswebcam]
""")

        parts.update()

        with self.assertRaises(parts.SnapcraftPartMissingError) as raised:
            project_loader.Config()
        self.assertEqual(
            'Cannot find the definition for part {!r}.\n'
            'It may be a remote part, run `snapcraft update` to refresh '
            'the remote parts cache.'.format('non-existing-part'),
            str(raised.exception))

    def test_config_after_is_an_undefined_part(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
    after: [non-existing-part]
""")

        parts.update()

        with self.assertRaises(parts.SnapcraftPartMissingError) as raised:
            project_loader.Config()
        self.assertEqual(
            'Cannot find the definition for part {!r}.\n'
            'It may be a remote part, run `snapcraft update` to refresh '
            'the remote parts cache.'.format('non-existing-part'),
            str(raised.exception))

    @unittest.mock.patch('snapcraft.internal.pluginhandler.load_plugin')
    def test_config_uses_remote_part_from_after(self, mock_load):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    after:
      - curl
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

        project_options = snapcraft.ProjectOptions()

        parts.update()
        project_loader.Config(project_options)

        call1 = unittest.mock.call(
            'curl',
            plugin_name='autotools',
            part_properties={
                'stage': [], 'snap': [], 'source': 'http://curl.org'},
            project_options=project_options,
            part_schema=self.part_schema)
        call2 = unittest.mock.call(
            'part1',
            plugin_name='go',
            part_properties={
                'stage': [], 'snap': [], 'stage-packages': ['fswebcam']},
            project_options=project_options,
            part_schema=self.part_schema)

        mock_load.assert_has_calls([call1, call2], any_order=True)

    def test_config_adds_vcs_packages_to_build_packages(self):
        scenarios = [
            ('git://github.com/ubuntu-core/snapcraft.git', 'git'),
            ('lp:ubuntu-push', 'bzr'),
            ('https://github.com/ubuntu-core/snapcraft/archive/2.0.1.tar.gz',
             ''),
        ]
        yaml_t = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    source: {0}
    plugin: autotools
"""

        for s in scenarios:
            with self.subTest(key=(s[1])):
                self.make_snapcraft_yaml(yaml_t.format(s[0]))
                c = project_loader.Config()

                if s[1]:
                    self.assertTrue(
                        s[1] in c.parts.build_tools,
                        '{} not found in {}'.format(
                            s[1], c.parts.build_tools))

    def test_config_adds_vcs_packages_to_build_packages_from_types(self):
        scenarios = [
            ('git', 'git'),
            ('hg', 'mercurial'),
            ('mercurial', 'mercurial'),
            ('bzr', 'bzr'),
            ('tar', ''),
            ('svn', 'subversion'),
            ('subversion', 'subversion'),
        ]
        yaml_t = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    source: http://something/somewhere
    source-type: {0}
    plugin: autotools
"""

        for s in scenarios:
            with self.subTest(key=(s[1])):
                self.make_snapcraft_yaml(yaml_t.format(s[0]))
                c = project_loader.Config()

                if s[1]:
                    self.assertTrue(
                        s[1] in c.parts.build_tools,
                        '{} not found in {}'.format(
                            s[1], c.parts.build_tools))

    def test_config_adds_extra_build_tools_when_cross_compiling(self):
        with unittest.mock.patch('platform.machine') as machine_mock:
            machine_mock.return_value = 'x86_64'
            project_options = snapcraft.ProjectOptions(target_deb_arch='armhf')

        yaml = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
"""
        self.make_snapcraft_yaml(yaml)
        config = project_loader.Config(project_options)

        self.assertEqual(config.parts.build_tools,
                         ['gcc-arm-linux-gnueabihf'])

    def test_config_has_no_extra_build_tools_when_not_cross_compiling(self):
        class ProjectOptionsFake(snapcraft.ProjectOptions):
            @property
            def is_cross_compiling(self):
                return False

        yaml = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
"""
        self.make_snapcraft_yaml(yaml)
        config = project_loader.Config(ProjectOptionsFake())

        self.assertEqual(config.parts.build_tools, [])

    def test_config_loop(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

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
        with self.assertRaises(parts.SnapcraftLogicError) as raised:
            project_loader.Config()

        self.assertEqual(
            raised.exception.message,
            'circular dependency chain found in parts definition')

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_missing_name(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(raised.exception.message,
                         "'name' is a required property")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_name_as_number(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: 1
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(raised.exception.message,
                         "The 'name' property does not match the required "
                         "schema: 1 is not of type 'string'")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_icon_extension(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.foo
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(raised.exception.message,
                         "'icon' must be either a .png or a .svg")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_missing_icon(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.png
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(raised.exception.message,
                         "Specified icon 'icon.png' does not exist")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_name_chars(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: myapp@me_1.0
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(
            raised.exception.message,
            "The 'name' property does not match the required schema: "
            "'myapp@me_1.0' does not match '^[a-z0-9][a-z0-9+-]*$'")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_missing_description(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(
            raised.exception.message,
            "'description' is a required property")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_yaml_missing_confinement_must_log(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        c = project_loader.Config()

        # Verify the default is "strict"
        self.assertTrue('confinement' in c.data,
                        'Expected "confinement" property to be in snap.yaml')
        self.assertEqual(c.data['confinement'], 'strict')
        self.assertTrue(
            '"confinement" property not specified: defaulting to "strict"'
            in fake_logger.output, 'Missing confinement hint in output')

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_yaml_missing_grade_must_log(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        c = project_loader.Config()

        # Verify the default is "stable"
        self.assertTrue('grade' in c.data,
                        'Expected "grade" property to be in snap.yaml')
        self.assertEqual(c.data['grade'], 'stable')
        self.assertTrue(
            '"grade" property not specified: defaulting to "stable"'
            in fake_logger.output, 'Missing grade hint in output')

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_yaml_valid_app_names(self, mock_loadPlugin):
        valid_app_names = [
            '1', 'a', 'aa', 'aaa', 'aaaa', 'Aa', 'aA', '1a', 'a1', '1-a',
            'a-1', 'a-a', 'aa-a', 'a-aa', 'a-b-c', '0a-a', 'a-0a',
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for app_name in valid_app_names:
            with self.subTest(key=app_name):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

apps:
  {!r}:
    command: foo

parts:
  part1:
    plugin: nil
""".format(app_name))
                c = project_loader.Config()
                self.assertTrue(app_name in c.data['apps'])

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_app_names(self, mock_loadPlugin):
        invalid_app_names = [
            '', '-', '--', 'a--a', 'a-', 'a ', ' a', 'a a', '日本語', '한글',
            'ру́сский язы́к', 'ໄຂ່​ອີ​ສ​ເຕີ້', ':a', 'a:', 'a:a', '_a', 'a_',
            'a_a',
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for app_name in invalid_app_names:
            with self.subTest(key=app_name):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

apps:
  {!r}:
    command: foo

parts:
  part1:
    plugin: nil
""".format(app_name))
                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Config()

                self.assertRegex(
                    raised.exception.message,
                    "The 'apps' property does not match the required "
                    "schema.*")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_yaml_valid_confinement_types(self, mock_loadPlugin):
        valid_confinement_types = [
            'strict',
            'devmode',
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for confinement_type in valid_confinement_types:
            with self.subTest(key=confinement_type):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: {}
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(confinement_type))
                c = project_loader.Config()
                self.assertEqual(c.data['confinement'], confinement_type)

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_confinement_types(self, mock_loadPlugin):
        invalid_confinement_types = [
            'foo',
            'strict-',
            '_devmode',
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for confinement_type in invalid_confinement_types:
            with self.subTest(key=confinement_type):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: {}
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(confinement_type))
                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Config()

                self.assertEqual(
                    raised.exception.message,
                    "The 'confinement' property does not match the required "
                    "schema: '{}' is not one of ['devmode', 'strict']".format(
                        confinement_type))

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_yaml_valid_grade_types(self, mock_loadPlugin):
        valid_grade_types = [
            'stable',
            'devel',
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for grade_type in valid_grade_types:
            with self.subTest(key=grade_type):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: {}

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(grade_type))
                c = project_loader.Config()
                self.assertEqual(c.data['grade'], grade_type)

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_grade_types(self, mock_loadPlugin):
        invalid_grade_types = [
            'foo',
            'unstable-',
            '_experimental',
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for grade_type in invalid_grade_types:
            with self.subTest(key=grade_type):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: {}

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(grade_type))
                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Config()

                self.assertEqual(
                    raised.exception.message,
                    "The 'grade' property does not match the required "
                    "schema: '{}' is not one of ['stable', 'devel']".format(
                        grade_type))

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_tab_in_yaml(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
\tconfinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")

        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Config()

        self.assertEqual(
            raised.exception.message,
            'found character \'\\t\' that cannot start any token '
            'on line 5 of snapcraft.yaml')

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_yaml_valid_epochs(self, mock_loadPlugin):
        valid_epochs = [
            {
                'yaml': 0,
                'expected': 0,
            },
            {
                'yaml': '"0"',
                'expected': '0',
            },
            {
                'yaml': '1*',
                'expected': '1*',
            },
            {
                'yaml': '"1*"',
                'expected': '1*',
            },
            {
                'yaml': 1,
                'expected': 1,
            },
            {
                'yaml': '"1"',
                'expected': '1',
            },
            {
                'yaml': '400*',
                'expected': '400*',
            },
            {
                'yaml': '"400*"',
                'expected': '400*',
            },
            {
                'yaml': 1234,
                'expected': 1234,
            },
            {
                'yaml': '"1234"',
                'expected': '1234',
            },
            {
                'yaml': '0001',
                'expected': 1,
            },
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for epoch in valid_epochs:
            with self.subTest(key=epoch):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
epoch: {}
parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(epoch['yaml']))
                c = project_loader.Config()
                self.assertEqual(c.data['epoch'], epoch['expected'])

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_invalid_yaml_invalid_epochs(self, mock_loadPlugin):
        invalid_epochs = [
            '0*',
            '_',
            '1-',
            '1+',
            '-1',
            '-1*',
            'a',
            '1a',
            '1**',
            '"01"',
            '1.2',
            '"1.2"',
            '[1]'
        ]

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        for epoch in invalid_epochs:
            with self.subTest(key=epoch):
                self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
epoch: {}
parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(epoch))
                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Config()

                self.assertRegex(
                    raised.exception.message,
                    "The 'epoch' property does not match the required "
                    "schema:.*is not a 'epoch' \(epochs are positive integers "
                    "followed by an optional asterisk\)")

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_config_expands_filesets(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

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
        project_loader.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'snap': ['/usr/lib/wget.so', '/usr/bin/wget',
                     '/usr/share/my-icon.png'],
            'stage-packages': ['fswebcam'],
            'stage': ['/usr/lib/wget.so', '/usr/bin/wget', '/usr/lib/wget.a'],
        })

    def test_get_prereqs(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = project_loader.Config()

        self.assertFalse(config.parts.get_prereqs('main'))
        self.assertEqual({'main'},
                         config.parts.get_prereqs('dependent'))

    def test_get_dependents(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = project_loader.Config()

        self.assertFalse(config.parts.get_dependents('dependent'))
        self.assertEqual({'dependent'},
                         config.parts.get_dependents('main'))

    @unittest.mock.patch('snapcraft.internal.parts.PartsConfig.load_plugin')
    def test_replace_snapcraft_variables(self, mock_load_plugin):
        self.make_snapcraft_yaml("""name: project-name
version: "1"
summary: test
description: test
confinement: strict

parts:
  main:
    plugin: make
    source: $SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION
    make-options: [DEP=$SNAPCRAFT_STAGE]
""")
        project_loader.Config()

        mock_load_plugin.assert_called_with('main', 'make', {
            'source': 'project-name-1',
            'stage': [], 'snap': [],
            'make-options': ['DEP={}'.format(self.stage_dir)],
        })


class InitTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

    def test_config_raises_on_missing_snapcraft_yaml(self):
        # no snapcraft.yaml
        with self.assertRaises(
                project_loader.SnapcraftYamlFileError) as raised:
            project_loader.Config()

        self.assertEqual(raised.exception.file, 'snapcraft.yaml')

    def test_two_snapcraft_yamls_cuase_error(self):
        open('snapcraft.yaml', 'w').close()
        open('.snapcraft.yaml', 'w').close()

        with self.assertRaises(EnvironmentError) as raised:
            project_loader.Config()

        self.assertEqual(
            str(raised.exception),
            "Found a 'snapcraft.yaml' and a '.snapcraft.yaml', "
            "please remove one")

    def test_hidden_snapcraft_yaml_loads(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil
""")

        os.rename('snapcraft.yaml', '.snapcraft.yaml')
        project_loader.Config()

    def test_visible_snapcraft_yaml_loads(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil
""")

        project_loader.Config()


class TestYamlEnvironment(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
""")

        project_options = snapcraft.ProjectOptions()
        patcher = unittest.mock.patch('snapcraft.ProjectOptions')
        mock_project_options = patcher.start()
        mock_project_options.return_value = project_options
        self.arch = project_options.deb_arch
        self.arch_triplet = project_options.arch_triplet
        self.addCleanup(patcher.stop)

    def test_config_snap_environment(self):
        config = project_loader.Config()

        lib_paths = [os.path.join(self.snap_dir, 'lib'),
                     os.path.join(self.snap_dir, 'usr', 'lib'),
                     os.path.join(self.snap_dir, 'lib',
                                  self.arch_triplet),
                     os.path.join(self.snap_dir, 'usr', 'lib',
                                  self.arch_triplet)]
        for lib_path in lib_paths:
            os.makedirs(lib_path)

        environment = config.snap_env()
        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.snap_dir)
            in environment)

        # Ensure that LD_LIBRARY_PATH is present and it contains only the
        # basics.
        paths = []
        for variable in environment:
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        expected = (os.path.join(self.snap_dir, i) for i in
                    ['lib', os.path.join('usr', 'lib'),
                     os.path.join('lib', self.arch_triplet),
                     os.path.join('usr', 'lib', self.arch_triplet)])
        for item in expected:
            self.assertTrue(
                item in paths,
                'Expected LD_LIBRARY_PATH in {!r} to include {!r}'.format(
                    paths, item))

    def test_config_snap_environment_with_no_library_paths(self):
        config = project_loader.Config()

        environment = config.snap_env()
        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.snap_dir)
            in environment,
            'Current PATH is {!r}'.format(environment))
        for e in environment:
            self.assertFalse('LD_LIBRARY_PATH' in e,
                             'Current environment is {!r}'.format(e))

    @unittest.mock.patch.object(snapcraft.internal.pluginhandler.PluginHandler,
                                'get_primed_dependency_paths')
    def test_config_snap_environment_with_dependencies(self,
                                                       mock_get_dependencies):
        library_paths = {
            os.path.join(self.snap_dir, 'lib1'),
            os.path.join(self.snap_dir, 'lib2'),
        }
        mock_get_dependencies.return_value = library_paths
        config = project_loader.Config()

        for lib_path in library_paths:
            os.makedirs(lib_path)

        # Ensure that LD_LIBRARY_PATH is present and it contains the
        # extra dependency paths.
        paths = []
        for variable in config.snap_env():
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        expected = (os.path.join(self.snap_dir, i) for i in ['lib1', 'lib2'])
        for item in expected:
            self.assertTrue(
                item in paths,
                'Expected LD_LIBRARY_PATH ({!r}) to include {!r}'.format(
                    paths, item))

    @unittest.mock.patch.object(snapcraft.internal.pluginhandler.PluginHandler,
                                'get_primed_dependency_paths')
    def test_config_snap_environment_with_dependencies_but_no_paths(
            self, mock_get_dependencies):
        library_paths = {
            os.path.join(self.snap_dir, 'lib1'),
            os.path.join(self.snap_dir, 'lib2'),
        }
        mock_get_dependencies.return_value = library_paths
        config = project_loader.Config()

        # Ensure that LD_LIBRARY_PATH is present, but is completey empty since
        # no library paths actually exist.
        for variable in config.snap_env():
            self.assertFalse(
                'LD_LIBRARY_PATH' in variable,
                'Expected no LD_LIBRARY_PATH (got {!r})'.format(variable))

    def test_config_runtime_environment_ld(self):
        # Place a few ld.so.conf files in supported locations. We expect the
        # contents of these to make it into the LD_LIBRARY_PATH.
        mesa_dir = os.path.join(
            self.snap_dir, 'usr', 'lib', 'my_arch', 'mesa')
        os.makedirs(mesa_dir)
        with open(os.path.join(mesa_dir, 'ld.so.conf'), 'w') as f:
            f.write('/mesa')

        mesa_egl_dir = os.path.join(
            self.snap_dir, 'usr', 'lib', 'my_arch', 'mesa-egl')
        os.makedirs(mesa_egl_dir)
        with open(os.path.join(mesa_egl_dir, 'ld.so.conf'), 'w') as f:
            f.write('# Standalone comment\n')
            f.write('/mesa-egl')

        config = project_loader.Config()
        environment = config.snap_env()

        # Ensure that the LD_LIBRARY_PATH includes all the above paths
        paths = []
        for variable in environment:
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        expected = (os.path.join(self.snap_dir, i) for i in
                    ['mesa', 'mesa-egl'])
        for item in expected:
            self.assertTrue(item in paths,
                            'Expected LD_LIBRARY_PATH to include "{}"'.format(
                                item))

    def test_config_stage_environment(self):
        paths = [os.path.join(self.stage_dir, 'lib'),
                 os.path.join(self.stage_dir, 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'lib'),
                 os.path.join(self.stage_dir, 'usr', 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'include'),
                 os.path.join(self.stage_dir, 'usr', 'include'),
                 os.path.join(self.stage_dir, 'include',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'include',
                              self.arch_triplet)]
        for path in paths:
            os.makedirs(path)

        config = project_loader.Config()
        environment = config.stage_env()

        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.stage_dir) in environment)
        self.assertTrue(
            'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{stage_dir}/lib:'
            '{stage_dir}/usr/lib:{stage_dir}/lib/{arch_triplet}:'
            '{stage_dir}/usr/lib/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CFLAGS="$CFLAGS -I{stage_dir}/include -I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CPPFLAGS="$CPPFLAGS -I{stage_dir}/include '
            '-I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CXXFLAGS="$CXXFLAGS -I{stage_dir}/include '
            '-I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'LDFLAGS="$LDFLAGS -L{stage_dir}/lib -L{stage_dir}/usr/lib '
            '-L{stage_dir}/lib/{arch_triplet} '
            '-L{stage_dir}/usr/lib/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue('PERL5LIB={}/usr/share/perl5/'.format(
            self.stage_dir) in environment)

    def test_parts_build_env_ordering_with_deps(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        self.useFixture(fixtures.EnvironmentVariable('PATH', '/bin'))

        self.maxDiff = None
        paths = [os.path.join(self.stage_dir, 'lib'),
                 os.path.join(self.stage_dir, 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'lib'),
                 os.path.join(self.stage_dir, 'usr', 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'include'),
                 os.path.join(self.stage_dir, 'usr', 'include'),
                 os.path.join(self.stage_dir, 'include',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'include',
                              self.arch_triplet),
                 os.path.join(self.parts_dir, 'part1', 'install',
                              'include'),
                 os.path.join(self.parts_dir, 'part1', 'install',
                              'lib'),
                 os.path.join(self.parts_dir, 'part2', 'install',
                              'include'),
                 os.path.join(self.parts_dir, 'part2', 'install',
                              'lib')]
        for path in paths:
            os.makedirs(path)

        config = project_loader.Config()
        part2 = [part for part in
                 config.parts.all_parts if part.name == 'part2'][0]
        env = config.parts.build_env_for_part(part2)
        env_lines = '\n'.join(['export {}\n'.format(e) for e in env])

        shell_env = {
            'CFLAGS': '-I/user-provided',
            'CXXFLAGS': '-I/user-provided',
            'CPPFLAGS': '-I/user-provided',
            'LDFLAGS': '-L/user-provided',
            'LD_LIBRARY_PATH': '/user-provided',
        }

        def get_envvar(envvar):
            with tempfile.NamedTemporaryFile(mode='w+') as f:
                f.write(env_lines)
                f.write('echo ${}'.format(envvar))
                f.flush()
                output = subprocess.check_output(['/bin/sh', f.name],
                                                 env=shell_env)
            return output.decode(sys.getfilesystemencoding()).strip()

        expected_cflags = (
            '-I/user-provided '
            '-I{parts_dir}/part2/install/include -I{stage_dir}/include '
            '-I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}'.format(
                parts_dir=self.parts_dir,
                stage_dir=self.stage_dir,
                arch_triplet=self.arch_triplet))
        self.assertEqual(get_envvar('CFLAGS'), expected_cflags)
        self.assertEqual(get_envvar('CXXFLAGS'), expected_cflags)
        self.assertEqual(get_envvar('CPPFLAGS'), expected_cflags)

        self.assertEqual(
            get_envvar('LDFLAGS'),
            '-L/user-provided '
            '-L{parts_dir}/part2/install/lib -L{stage_dir}/lib '
            '-L{stage_dir}/usr/lib -L{stage_dir}/lib/{arch_triplet} '
            '-L{stage_dir}/usr/lib/{arch_triplet}'.format(
                parts_dir=self.parts_dir,
                stage_dir=self.stage_dir,
                arch_triplet=self.arch_triplet))

        self.assertEqual(
            get_envvar('LD_LIBRARY_PATH'),
            '/user-provided:'
            '{parts_dir}/part2/install/lib:'
            '{stage_dir}/lib:'
            '{stage_dir}/usr/lib:'
            '{stage_dir}/lib/{arch_triplet}:'
            '{stage_dir}/usr/lib/{arch_triplet}:'
            '{stage_dir}/lib:'
            '{stage_dir}/usr/lib:'
            '{stage_dir}/lib/{arch_triplet}:'
            '{stage_dir}/usr/lib/{arch_triplet}'.format(
                parts_dir=self.parts_dir,
                stage_dir=self.stage_dir,
                arch_triplet=self.arch_triplet))


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

                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Validator(data).validate()

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

                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Validator(data).validate()

                expected_message = ("The 'name' property does not match the "
                                    "required schema: '{}' does not match "
                                    "'^[a-z0-9][a-z0-9+-]*$'").format(name)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Validator(self.data).validate()

        expected_message = (
            "The 'summary' property does not match the required schema: "
            "'{}' is too long").format(self.data['summary'])
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_valid_types(self):
        valid_types = [
            'app',
            'gadget',
            'kernel',
            'os',
        ]

        for t in valid_types:
            data = self.data.copy()
            with self.subTest(key=t):
                project_loader.Validator(data).validate()

    def test_invalid_types(self):
        invalid_types = [
            'apps',
            'framework',
            'platform',
            'oem',
        ]

        for t in invalid_types:
            data = self.data.copy()
            with self.subTest(key=t):
                data['type'] = t

                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Validator(data).validate()

                expected_message = (
                    "The 'type' property does not match the required "
                    "schema: '{}' is not one of "
                    "['app', 'gadget', 'kernel', 'os']").format(t)
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

        project_loader.Validator(self.data).validate()

    def test_valid_restart_conditions(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
            }
        }
        valid_conditions = ['always', 'on-success', 'on-failure',
                            'on-abnormal', 'on-abort', 'never']

        for condition in valid_conditions:
            with self.subTest(key=condition):
                self.data['apps']['service1']['restart-condition'] = condition
                project_loader.Validator(self.data).validate()

    def test_invalid_restart_condition(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'restart-condition': 'on-watchdog',
            }
        }

        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Validator(self.data).validate()

        self.assertEqual(
            "The 'apps/service1/restart-condition' property does not match "
            "the required schema: 'on-watchdog' is not one of ['on-success', "
            "'on-failure', 'on-abnormal', 'on-abort', 'always', 'never']",
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

                with self.assertRaises(SnapcraftSchemaError) as raised:
                    project_loader.Validator(data).validate()

                expected_message = (
                    "The 'apps' property does not match the required "
                    "schema: Additional properties are not allowed ('{}' "
                    "was unexpected)").format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_apps_required_properties(self):
        self.data['apps'] = {'service1': {}}

        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Validator(self.data).validate()

        expected_message = ("The 'apps/service1' property does not match the "
                            "required schema: 'command' is a required "
                            "property")
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_schema_file_not_found(self):
        mock_the_open = unittest.mock.mock_open()
        mock_the_open.side_effect = FileNotFoundError()

        with unittest.mock.patch('snapcraft._schema.open',
                                 mock_the_open, create=True):
            with self.assertRaises(SnapcraftSchemaError) as raised:
                project_loader.Validator(self.data).validate()

        expected_message = ('snapcraft validation file is missing from '
                            'installation path')
        self.assertEqual(raised.exception.message, expected_message)

    def test_icon_missing_is_valid_yaml(self):
        self.mock_path_exists.return_value = False

        project_loader.Validator(self.data).validate()

    def test_invalid_part_name_plugin_raises_exception(self):
        self.data['parts']['plugins'] = {'type': 'go'}

        with self.assertRaises(SnapcraftSchemaError) as raised:
            project_loader.Validator(self.data).validate()

        expected_message = ("The 'parts' property does not match the "
                            "required schema: Additional properties are not "
                            "allowed ('plugins' was unexpected)")
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)


class TestPluginLoadingProperties(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        self.data = """name: my-package-1
version: 1.0-snapcraft1~ppa1
summary: my summary less that 79 chars
description: description which can be pretty long
parts:
    part1:
        plugin: nil
"""

        self.fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(self.fake_logger)
        self.expected_message_template = (
            "Issue while loading plugin: properties failed to load for "
            "part1: Additional properties are not allowed ('{}' was "
            "unexpected)\n")

    def test_slots_as_properties_should_fail(self):
        self.data += '        slots: [slot1]'
        self.make_snapcraft_yaml(self.data)

        with self.assertRaises(SystemExit):
            project_loader.load_config()

        expected_message = self.expected_message_template.format('slots')
        self.assertIn(expected_message, self.fake_logger.output)

    def test_plugs_as_properties_should_fail(self):
        self.data += '        plugs: [plug1]'
        self.make_snapcraft_yaml(self.data)

        with self.assertRaises(SystemExit):
            project_loader.load_config()

        expected_message = self.expected_message_template.format('plugs')
        self.assertIn(expected_message, self.fake_logger.output)


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

        fs = project_loader._expand_filesets_for('stage', self.properties)
        self.assertEqual(fs, ['1', '2', '3'])

    def test_no_expansion(self):
        self.properties['stage'] = ['1']

        fs = project_loader._expand_filesets_for('stage', self.properties)
        self.assertEqual(fs, ['1'])

    def test_invalid_expansion(self):
        self.properties['stage'] = ['$3']

        with self.assertRaises(parts.SnapcraftLogicError) as raised:
            project_loader._expand_filesets_for('stage', self.properties)

        self.assertEqual(
            raised.exception.message,
            '\'$3\' referred to in the \'stage\' fileset but it is not '
            'in filesets')


class SnapcraftEnvTestCase(tests.TestCase):

    def test_string_replacements(self):
        replacements = (
            (
                'no replacement',
                'snapcraft_stage/usr/bin',
                'snapcraft_stage/usr/bin',
            ),
            (
                'replaced start',
                '$SNAPCRAFT_STAGE/usr/bin',
                '{}/usr/bin'.format(self.stage_dir),
            ),
            (
                'replaced between',
                '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                '--with-swig {}/usr/swig'.format(self.stage_dir),
            ),
            (
                'project replacement',
                '$SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION',
                'project_name-version',
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]), expected)

    def test_lists_with_string_replacements(self):
        replacements = (
            (
                'no replacement',
                [
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ],
                [
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ],
            ),
            (
                'replaced start',
                [
                    '$SNAPCRAFT_STAGE/usr/bin',
                    '/usr/bin',
                ],
                [
                    '{}/usr/bin'.format(self.stage_dir),
                    '/usr/bin',
                ],
            ),
            (
                'replaced between',
                [
                    '--without-python',
                    '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                ],
                [
                    '--without-python',
                    '--with-swig {}/usr/swig'.format(self.stage_dir),
                ],
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]), expected)

    def test_tuples_with_string_replacements(self):
        replacements = (
            (
                'no replacement',
                (
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ),
                [
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ],
            ),
            (
                'replaced start',
                (
                    '$SNAPCRAFT_STAGE/usr/bin',
                    '/usr/bin',
                ),
                [
                    '{}/usr/bin'.format(self.stage_dir),
                    '/usr/bin',
                ],
            ),
            (
                'replaced between',
                (
                    '--without-python',
                    '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                ),
                [
                    '--without-python',
                    '--with-swig {}/usr/swig'.format(self.stage_dir),
                ],
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]), expected)

    def test_dict_with_string_replacements(self):
        replacements = (
            (
                'no replacement',
                {
                    '1': 'snapcraft_stage/usr/bin',
                    '2': '/usr/bin',
                },
                {
                    '1': 'snapcraft_stage/usr/bin',
                    '2': '/usr/bin',
                },
            ),
            (
                'replaced start',
                {
                    '1': '$SNAPCRAFT_STAGE/usr/bin',
                    '2': '/usr/bin',
                },
                {
                    '1': '{}/usr/bin'.format(self.stage_dir),
                    '2': '/usr/bin',
                },
            ),
            (
                'replaced between',
                {
                    '1': '--without-python',
                    '2': '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                },
                {
                    '1': '--without-python',
                    '2': '--with-swig {}/usr/swig'.format(self.stage_dir),
                },
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]), expected)

    def test_string_replacement_with_complex_data(self):
        subject = {
            'filesets': {
                'files': [
                    'somefile',
                    '$SNAPCRAFT_STAGE/file1',
                    'SNAPCRAFT_STAGE/really',
                ]
            },
            'configflags': [
                '--with-python',
                '--with-swig $SNAPCRAFT_STAGE/swig',
            ],
        }

        expected = {
            'filesets': {
                'files': [
                    'somefile',
                    '{}/file1'.format(self.stage_dir),
                    'SNAPCRAFT_STAGE/really',
                ]
            },
            'configflags': [
                '--with-python',
                '--with-swig {}/swig'.format(self.stage_dir),
            ],
        }

        self.assertEqual(project_loader.replace_attr(
            subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                      ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                      ('$SNAPCRAFT_STAGE', self.stage_dir)]), expected)
