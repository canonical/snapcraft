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
from snapcraft.internal import yaml as internal_yaml
from snapcraft import tests
from snapcraft.tests import fixture_setup

from snapcraft._schema import SnapcraftSchemaError


class TestYaml(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch(
            'snapcraft.internal.yaml._get_snapcraft_yaml')
        self.mock_get_yaml = patcher.start()
        self.mock_get_yaml.return_value = 'snapcraft.yaml'
        self.addCleanup(patcher.stop)
        self.part_schema = internal_yaml.Validator().part_schema
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_config_loads_plugins(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        internal_yaml.Config()
        mock_loadPlugin.assert_called_with('part1', 'go', {
            'stage-packages': ['fswebcam'],
            'stage': [], 'snap': [],
        })

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_config_loads_with_different_encodings(
            self, mock_loadPlugin):
        content = """name: test
version: "1"
summary: test
description: ñoño test
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
"""
        for enc in ['utf-8', 'utf-8-sig', 'utf-16']:
            with self.subTest(key=enc):
                self.make_snapcraft_yaml(content, encoding=enc)
                internal_yaml.Config()

                mock_loadPlugin.assert_called_with('part1', 'go', {
                    'stage-packages': ['fswebcam'],
                    'stage': [], 'snap': [],
                })

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_config_composes_with_remote_parts(self, mock_loadPlugin):
        self.useFixture(fixture_setup.FakeParts())
        patcher = unittest.mock.patch(
            'snapcraft.internal.parts.ProgressBar',
            new=tests.SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part1:
    stage-packages: [fswebcam]
""")

        parts.update()
        internal_yaml.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'source': 'http://source.tar.gz', 'stage-packages': ['fswebcam'],
            'stage': [], 'snap': []})

    def test_config_composes_with_a_non_existent_remote_part(self):
        self.useFixture(fixture_setup.FakeParts())
        patcher = unittest.mock.patch(
            'snapcraft.internal.parts.ProgressBar',
            new=tests.SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  non-existing-part:
    stage-packages: [fswebcam]
""")

        parts.update()

        with self.assertRaises(internal_yaml.SnapcraftLogicError) as raised:
            internal_yaml.Config()
        self.assertEqual(
            str(raised.exception),
            '{!r} is missing the `plugin` entry and is not defined in the '
            'current remote parts cache, try to run `snapcraft update` '
            'to refresh'.format('non-existing-part'))

    def test_config_after_is_an_undefined_part(self):
        self.useFixture(fixture_setup.FakeParts())
        patcher = unittest.mock.patch(
            'snapcraft.internal.parts.ProgressBar',
            new=tests.SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part1:
    plugin: nil
    after: [non-existing-part]
""")

        parts.update()

        with self.assertRaises(internal_yaml.SnapcraftLogicError) as raised:
            internal_yaml.Config()
        self.assertEqual(
            str(raised.exception),
            'Cannot find definition for part {!r}. It may be a '
            'remote part, run `snapcraft update` to '
            'refresh the remote parts cache'.format('non-existing-part'))

    @unittest.mock.patch('snapcraft.internal.pluginhandler.load_plugin')
    def test_config_uses_remote_part_from_after(self, mock_load):
        self.useFixture(fixture_setup.FakeParts())
        patcher = unittest.mock.patch(
            'snapcraft.internal.parts.ProgressBar',
            new=tests.SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

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
        internal_yaml.Config(project_options)

        call1 = unittest.mock.call('part1', 'go', {
            'stage': [], 'snap': [], 'stage-packages': ['fswebcam']},
            project_options, self.part_schema)
        call2 = unittest.mock.call('curl', 'autotools', {
            'source': 'http://curl.org'},
            project_options, self.part_schema)

        mock_load.assert_has_calls([call1, call2])

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
confinement: strict

parts:
  part1:
    source: {0}
    plugin: autotools
"""

        for s in scenarios:
            with self.subTest(key=(s[1])):
                self.make_snapcraft_yaml(yaml_t.format(s[0]))
                c = internal_yaml.Config()

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
confinement: strict

parts:
  part1:
    source: http://something/somewhere
    source-type: {0}
    plugin: autotools
"""

        for s in scenarios:
            with self.subTest(key=(s[1])):
                self.make_snapcraft_yaml(yaml_t.format(s[0]))
                c = internal_yaml.Config()

                self.assertTrue(
                    s[1] in c.build_tools,
                    '{} not found in {}'.format(s[1], c.build_tools))

    def test_config_adds_extra_build_tools_when_cross_compiling(self):
        with unittest.mock.patch('platform.machine') as machine_mock:
            machine_mock.return_value = 'x86_64'
            project_options = snapcraft.ProjectOptions(target_deb_arch='armhf')

        yaml = """name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part1:
    plugin: nil
"""
        self.make_snapcraft_yaml(yaml)
        config = internal_yaml.Config(project_options)

        self.assertEqual(config.build_tools, ['gcc-arm-linux-gnueabihf'])

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

parts:
  part1:
    plugin: nil
"""
        self.make_snapcraft_yaml(yaml)
        config = internal_yaml.Config(ProjectOptionsFake())

        self.assertEqual(config.build_tools, [])

    def test_config_loop(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

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
        with self.assertRaises(internal_yaml.SnapcraftLogicError) as raised:
            internal_yaml.Config()

        self.assertEqual(
            raised.exception.message,
            'circular dependency chain found in parts definition')

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_name(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""
version: "1"
summary: test
description: nothing
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(raised.exception.message,
                         "'name' is a required property")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_name_as_number(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: 1
version: "1"
summary: test
description: nothing
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(raised.exception.message,
                         "The 'name' property does not match the required "
                         "schema: 1 is not of type 'string'")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_icon_extension(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.foo
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(raised.exception.message,
                         "'icon' must be either a .png or a .svg")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_icon(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.png
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(raised.exception.message,
                         "Specified icon 'icon.png' does not exist")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_invalid_yaml_invalid_name_chars(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: myapp@me_1.0
version: "1"
summary: test
description: nothing
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(
            raised.exception.message,
            "The 'name' property does not match the required schema: "
            "'myapp@me_1.0' does not match '^[a-z0-9][a-z0-9+-]*$'")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_invalid_yaml_missing_description(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(
            raised.exception.message,
            "'description' is a required property")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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
        c = internal_yaml.Config()

        # Verify the default is "strict"
        self.assertTrue('confinement' in c.data,
                        'Expected "confinement" property to be in snap.yaml')
        self.assertEqual(c.data['confinement'], 'strict')
        self.assertTrue(
            '"confinement" property not specified: defaulting to "strict"'
            in fake_logger.output, 'Missing confinement hint in output')

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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

apps:
  {!r}:
    command: foo

parts:
  part1:
    plugin: nil
""".format(app_name))
                c = internal_yaml.Config()
                self.assertTrue(app_name in c.data['apps'])

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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

apps:
  {!r}:
    command: foo

parts:
  part1:
    plugin: nil
""".format(app_name))
                with self.assertRaises(SnapcraftSchemaError) as raised:
                    internal_yaml.Config()

                self.assertRegex(
                    raised.exception.message,
                    "The 'apps' property does not match the required "
                    "schema.*")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(confinement_type))
                c = internal_yaml.Config()
                self.assertEqual(c.data['confinement'], confinement_type)

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(confinement_type))
                with self.assertRaises(SnapcraftSchemaError) as raised:
                    internal_yaml.Config()

                self.assertEqual(
                    raised.exception.message,
                    "The 'confinement' property does not match the required "
                    "schema: '{}' is not one of ['devmode', 'strict']".format(
                        confinement_type))

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_tab_in_yaml(self, mock_loadPlugin):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
\tconfinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")

        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Config()

        self.assertEqual(
            raised.exception.message,
            'found character \'\\t\' that cannot start any token '
            'on line 4 of snapcraft.yaml')

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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
                c = internal_yaml.Config()
                self.assertEqual(c.data['epoch'], epoch['expected'])

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
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
                    internal_yaml.Config()

                self.assertRegex(
                    raised.exception.message,
                    "The 'epoch' property does not match the required "
                    "schema:.*is not a 'epoch' \(epochs are positive integers "
                    "followed by an optional asterisk\)")

    @unittest.mock.patch('snapcraft.internal.yaml.Config.load_plugin')
    def test_config_expands_filesets(self, mock_loadPlugin):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

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
        internal_yaml.Config()

        mock_loadPlugin.assert_called_with('part1', 'go', {
            'snap': ['/usr/lib/wget.so', '/usr/bin/wget',
                     '/usr/share/my-icon.png'],
            'stage-packages': ['fswebcam'],
            'stage': ['/usr/lib/wget.so', '/usr/bin/wget', '/usr/lib/wget.a'],
        })

    def test_part_prereqs(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = internal_yaml.Config()

        self.assertFalse(config.part_prereqs('main'))
        self.assertEqual({'main'}, config.part_prereqs('dependent'))

    def test_part_dependents(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = internal_yaml.Config()

        self.assertFalse(config.part_dependents('dependent'))
        self.assertEqual({'dependent'}, config.part_dependents('main'))


class InitTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

    def test_config_raises_on_missing_snapcraft_yaml(self):
        # no snapcraft.yaml
        with self.assertRaises(
                snapcraft.internal.yaml.SnapcraftYamlFileError) as raised:
            internal_yaml.Config()

        self.assertEqual(raised.exception.file, 'snapcraft.yaml')

    def test_two_snapcraft_yamls_cuase_error(self):
        open('snapcraft.yaml', 'w').close()
        open('.snapcraft.yaml', 'w').close()

        with self.assertRaises(EnvironmentError) as raised:
            internal_yaml.Config()

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

parts:
  main:
    plugin: nil
""")

        os.rename('snapcraft.yaml', '.snapcraft.yaml')
        internal_yaml.Config()

    def test_visible_snapcraft_yaml_loads(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  main:
    plugin: nil
""")

        internal_yaml.Config()


class TestYamlEnvironment(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

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
        config = internal_yaml.Config()

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
            'PATH="{0}/bin:{0}/usr/bin:$PATH"'.format(self.snap_dir)
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
        config = internal_yaml.Config()

        environment = config.snap_env()
        self.assertTrue(
            'PATH="{0}/bin:{0}/usr/bin:$PATH"'.format(self.snap_dir)
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
        config = internal_yaml.Config()

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
        config = internal_yaml.Config()

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

        config = internal_yaml.Config()
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

        config = internal_yaml.Config()
        environment = config.stage_env()

        self.assertTrue(
            'PATH="{0}/bin:{0}/usr/bin:$PATH"'.format(
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

        config = internal_yaml.Config()
        part2 = [part for part in config.all_parts if part.name == 'part2'][0]
        env = config.build_env_for_part(part2)
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
                    internal_yaml.Validator(data).validate()

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
                    internal_yaml.Validator(data).validate()

                expected_message = ("The 'name' property does not match the "
                                    "required schema: '{}' does not match "
                                    "'^[a-z0-9][a-z0-9+-]*$'").format(name)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Validator(self.data).validate()

        expected_message = (
            "The 'summary' property does not match the required schema: "
            "'{}' is too long").format(self.data['summary'])
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_valid_types(self):
        valid_types = [
            'app',
            'kernel',
            'os',
        ]

        for t in valid_types:
            data = self.data.copy()
            with self.subTest(key=t):
                internal_yaml.Validator(data).validate()

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
                    internal_yaml.Validator(data).validate()

                expected_message = (
                    "The 'type' property does not match the required "
                    "schema: '{}' is not one of "
                    "['app', 'kernel', 'os']").format(t)
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

        internal_yaml.Validator(self.data).validate()

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
                internal_yaml.Validator(self.data).validate()

    def test_invalid_restart_condition(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'restart-condition': 'on-watchdog',
            }
        }

        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Validator(self.data).validate()

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

                with self.assertRaises(SnapcraftSchemaError) as raised:
                    internal_yaml.Validator(data).validate()

                expected_message = (
                    "The 'apps' property does not match the required "
                    "schema: Additional properties are not allowed ('{}' "
                    "was unexpected)").format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_apps_required_properties(self):
        self.data['apps'] = {'service1': {}}

        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Validator(self.data).validate()

        expected_message = ("The 'service1' property does not match the "
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
                internal_yaml.Validator(self.data).validate()

        expected_message = ('snapcraft validation file is missing from '
                            'installation path')
        self.assertEqual(raised.exception.message, expected_message)

    def test_icon_missing_is_valid_yaml(self):
        self.mock_path_exists.return_value = False

        internal_yaml.Validator(self.data).validate()

    def test_invalid_part_name_plugin_raises_exception(self):
        self.data['parts']['plugins'] = {'type': 'go'}

        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Validator(self.data).validate()

        expected_message = ("The 'parts' property does not match the "
                            "required schema: Additional properties are not "
                            "allowed ('plugins' was unexpected)")
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_license_hook(self):
        self.data['license'] = 'LICENSE'

        internal_yaml.Validator(self.data).validate()

    def test_full_license_use(self):
        self.data['license'] = 'LICENSE'
        self.data['license-agreement'] = 'explicit'
        self.data['license-version'] = '1.0'

        internal_yaml.Validator(self.data).validate()

    def test_license_with_license_version(self):
        self.data['license'] = 'LICENSE'
        self.data['license-version'] = '1.0'

        internal_yaml.Validator(self.data).validate()

    def test_license_agreement_without_license_raises_exception(self):
        self.data['license-agreement'] = 'explicit'

        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Validator(self.data).validate()

        expected_message = "'license' is a dependency of 'license-agreement'"
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_license_version_without_license_raises_exception(self):
        self.data['license-version'] = '1.1'

        with self.assertRaises(SnapcraftSchemaError) as raised:
            internal_yaml.Validator(self.data).validate()

        expected_message = "'license' is a dependency of 'license-version'"
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
            internal_yaml.load_config()

        expected_message = self.expected_message_template.format('slots')
        self.assertEqual(expected_message, self.fake_logger.output)

    def test_plugs_as_properties_should_fail(self):
        self.data += '        plugs: [plug1]'
        self.make_snapcraft_yaml(self.data)

        with self.assertRaises(SystemExit):
            internal_yaml.load_config()

        expected_message = self.expected_message_template.format('plugs')
        self.assertEqual(expected_message, self.fake_logger.output)


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

        fs = internal_yaml._expand_filesets_for('stage', self.properties)
        self.assertEqual(fs, ['1', '2', '3'])

    def test_no_expansion(self):
        self.properties['stage'] = ['1']

        fs = internal_yaml._expand_filesets_for('stage', self.properties)
        self.assertEqual(fs, ['1'])

    def test_invalid_expansion(self):
        self.properties['stage'] = ['$3']

        with self.assertRaises(internal_yaml.SnapcraftLogicError) as raised:
            internal_yaml._expand_filesets_for('stage', self.properties)

        self.assertEqual(
            raised.exception.message,
            '\'$3\' referred to in the \'stage\' fileset but it is not '
            'in filesets')


class TestPkgConfig(tests.TestCase):

    _PC_TEMPLATE = """prefix=/usr
exec_prefix=${{prefix}}
libdir=${{prefix}}/lib/x86_64-linux-gnu
includedir=${{prefix}}/include

Name: {module}
Description: test
Version: 1.0
Libs: -L${{libdir}} -llib{module}
Cflags: -I${{includedir}}/{module}
"""

    def setUp(self):
        super().setUp()

        self.installdir = os.path.join(os.getcwd(), 'installdir')
        os.makedirs(self.installdir)

        self.stagedir = os.path.join(os.getcwd(), 'stagedir')
        os.makedirs(self.stagedir)

        self.bindir = os.path.join(os.getcwd(), 'bin')
        os.makedirs(self.bindir)

        project_options = snapcraft.ProjectOptions()
        env = internal_yaml._create_pkg_config_override(
            self.bindir, self.installdir, self.stagedir,
            project_options.arch_triplet)
        self.assertEqual(env, ['PATH={}:$PATH'.format(self.bindir)])

        self.pkg_config_bin = os.path.join(self.bindir, 'pkg-config')

    def _create_pc_file(self, workdir, module):
        pkgconfig_dir = os.path.join(workdir, 'usr', 'lib', 'pkgconfig')
        os.makedirs(pkgconfig_dir, exist_ok=True)
        pc_module = os.path.join(pkgconfig_dir, '{}.pc'.format(module))
        with open(pc_module, 'w') as fn:
            fn.write(self._PC_TEMPLATE.format(module=module))

    def test_pkg_config_prefers_installdir(self):
        self._create_pc_file(self.installdir, 'module1')
        self._create_pc_file(self.stagedir, 'module1')

        out = subprocess.check_output([
            self.pkg_config_bin, '--cflags-only-I',
            'module1']).decode('utf-8').strip()

        self.assertEqual(
            out, '-I{}/usr/include/module1'.format(self.installdir))

    def test_pkg_config_finds_in_stagedir(self):
        self._create_pc_file(self.installdir, 'module2')
        self._create_pc_file(self.stagedir, 'module1')

        out = subprocess.check_output([
            self.pkg_config_bin, '--cflags-only-I',
            'module1']).decode('utf-8').strip()

        self.assertEqual(
            out, '-I{}/usr/include/module1'.format(self.stagedir))

    def test_pkg_config_works_with_two_modules(self):
        self._create_pc_file(self.installdir, 'module1')
        self._create_pc_file(self.installdir, 'module2')

        out = subprocess.check_output([
            self.pkg_config_bin, '--cflags-only-I',
            'module1', 'module2']).decode('utf-8').strip()

        self.assertEqual(out,
                         '-I{dir}/usr/include/module1 '
                         '-I{dir}/usr/include/module2'.format(
                            dir=self.installdir))
