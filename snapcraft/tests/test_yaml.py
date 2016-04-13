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
import snapcraft.common
import snapcraft.yaml
from snapcraft import (
    dirs,
    tests,
)
from snapcraft._schema import SnapcraftSchemaError


class TestYaml(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch('os.path.exists')
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = True
        self.addCleanup(patcher.stop)
        self.part_schema = snapcraft.yaml.Validator().part_schema

        self.deb_arch = snapcraft.ProjectOptions().deb_arch

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

        project_options = snapcraft.ProjectOptions()

        snapcraft.yaml.Config(project_options)

        call1 = unittest.mock.call('part1', 'go', {
            'stage': [], 'snap': [], 'stage-packages': ['fswebcam']},
            project_options, self.part_schema)
        call2 = unittest.mock.call('part2wiki', 'go', {
            'source': 'http://somesource'},
            project_options, self.part_schema)

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
        with self.assertRaises(SnapcraftSchemaError) as raised:
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
        with self.assertRaises(SnapcraftSchemaError) as raised:
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
        with self.assertRaises(SnapcraftSchemaError) as raised:
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
        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Config()

        self.assertEqual(raised.exception.message,
                         "Specified icon 'icon.png' does not exist")

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
        with self.assertRaises(SnapcraftSchemaError) as raised:
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
        with self.assertRaises(SnapcraftSchemaError) as raised:
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

        with self.assertRaises(SnapcraftSchemaError) as raised:
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

    def test_part_prereqs(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = snapcraft.yaml.Config()

        self.assertFalse(config.part_prereqs('main'))
        self.assertEqual({'main'}, config.part_prereqs('dependent'))

    def test_part_dependents(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = snapcraft.yaml.Config()

        self.assertFalse(config.part_dependents('dependent'))
        self.assertEqual({'dependent'}, config.part_dependents('main'))


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

        project_options = snapcraft.ProjectOptions()
        patcher = unittest.mock.patch('snapcraft.ProjectOptions')
        mock_project_options = patcher.start()
        mock_project_options.return_value = project_options
        self.arch = project_options.deb_arch
        self.arch_triplet = project_options.arch_triplet
        self.addCleanup(patcher.stop)

    @unittest.mock.patch('snapcraft.common.get_snapdir')
    def test_config_snap_environment(self, mock_snapdir):
        mock_snapdir.return_value = 'foo'
        config = snapcraft.yaml.Config()

        lib_paths = ['foo/lib', 'foo/usr/lib', 'foo/lib/{}'.format(
                        self.arch_triplet),
                     'foo/usr/lib/{}'.format(self.arch_triplet)]
        for lib_path in lib_paths:
            os.makedirs(lib_path)

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

        for expected in ['foo/lib', 'foo/usr/lib', 'foo/lib/{}'.format(
                            self.arch_triplet),
                         'foo/usr/lib/{}'.format(self.arch_triplet)]:
            self.assertTrue(
                expected in paths,
                'Expected LD_LIBRARY_PATH in {!r} to include {!r}'.format(
                    paths, expected))

    @unittest.mock.patch('snapcraft.common.get_snapdir')
    def test_config_snap_environment_with_no_library_paths(self, mock_snapdir):
        mock_snapdir.return_value = 'foo'
        config = snapcraft.yaml.Config()

        environment = config.snap_env()
        self.assertTrue('PATH="foo/bin:foo/usr/bin:$PATH"' in environment)
        for e in environment:
            self.assertFalse('LD_LIBRARY_PATH' in e,
                             'Current environment is {!r}'.format(e))

    @unittest.mock.patch('snapcraft.common.get_snapdir')
    @unittest.mock.patch.object(snapcraft.pluginhandler.PluginHandler,
                                'get_stripped_dependency_paths')
    def test_config_snap_environment_with_dependencies(self,
                                                       mock_get_dependencies,
                                                       mock_snapdir):
        library_paths = {'foo/lib1', 'foo/lib2'}
        mock_snapdir.return_value = 'foo'
        mock_get_dependencies.return_value = library_paths
        config = snapcraft.yaml.Config()

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

        for expected in ['foo/lib1', 'foo/lib2']:
            self.assertTrue(
                expected in paths,
                'Expected LD_LIBRARY_PATH ({!r}) to include {!r}'.format(
                    paths, expected))

    @unittest.mock.patch('snapcraft.common.get_snapdir')
    @unittest.mock.patch.object(snapcraft.pluginhandler.PluginHandler,
                                'get_stripped_dependency_paths')
    def test_config_snap_environment_with_dependencies_but_no_paths(
            self, mock_get_dependencies, mock_snapdir):
        library_paths = {'foo/lib1', 'foo/lib2'}
        mock_snapdir.return_value = 'foo'
        mock_get_dependencies.return_value = library_paths
        config = snapcraft.yaml.Config()

        # Ensure that LD_LIBRARY_PATH is present, but is completey empty since
        # no library paths actually exist.
        for variable in config.snap_env():
            self.assertFalse(
                'LD_LIBRARY_PATH' in variable,
                'Expected no LD_LIBRARY_PATH (got {!r})'.format(variable))

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

        paths = ['foo/lib', 'foo/usr/lib', 'foo/lib/{}'.format(
                    self.arch_triplet),
                 'foo/usr/lib/{}'.format(self.arch_triplet),
                 'foo/include', 'foo/usr/include',
                 'foo/include/{}'.format(self.arch_triplet),
                 'foo/usr/include/{}'.format(self.arch_triplet)]
        for path in paths:
            os.makedirs(path)

        config = snapcraft.yaml.Config()
        environment = config.stage_env()

        self.assertTrue('PATH="foo/bin:foo/usr/bin:$PATH"' in environment)
        self.assertTrue(
            'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:foo/lib:foo/usr/lib:'
            'foo/lib/x86_64-linux-gnu:foo/usr/lib/x86_64-linux-gnu"'
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CFLAGS="$CFLAGS -Ifoo/include -Ifoo/usr/include '
            '-Ifoo/include/x86_64-linux-gnu '
            '-Ifoo/usr/include/x86_64-linux-gnu"' in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CPPFLAGS="$CPPFLAGS -Ifoo/include -Ifoo/usr/include '
            '-Ifoo/include/x86_64-linux-gnu '
            '-Ifoo/usr/include/x86_64-linux-gnu"' in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CXXFLAGS="$CXXFLAGS -Ifoo/include -Ifoo/usr/include '
            '-Ifoo/include/x86_64-linux-gnu '
            '-Ifoo/usr/include/x86_64-linux-gnu"' in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'LDFLAGS="$LDFLAGS -Lfoo/lib -Lfoo/usr/lib '
            '-Lfoo/lib/x86_64-linux-gnu -Lfoo/usr/lib/x86_64-linux-gnu"'
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue('PERL5LIB=foo/usr/share/perl5/' in environment)

    @unittest.mock.patch('snapcraft.common.get_stagedir')
    @unittest.mock.patch('snapcraft.common.get_partsdir')
    def test_parts_build_env_ordering_with_deps(self,
                                                mock_partsdir,
                                                mock_stagedir):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test

parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        self.useFixture(fixtures.EnvironmentVariable('PATH', '/bin'))
        mock_partsdir.return_value = 'parts'
        mock_stagedir.return_value = 'foo'

        paths = ['foo/lib', 'foo/usr/lib', 'foo/lib/{}'.format(
                    self.arch_triplet),
                 'foo/usr/lib/{}'.format(self.arch_triplet),
                 'foo/include', 'foo/usr/include',
                 'foo/include/{}'.format(self.arch_triplet),
                 'foo/usr/include/{}'.format(self.arch_triplet),
                 'parts/part1/install/include',
                 'parts/part1/install/lib',
                 'parts/part2/install/include',
                 'parts/part2/install/lib']
        for path in paths:
            os.makedirs(path)

        config = snapcraft.yaml.Config()
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
            '-Iparts/part2/install/include -Ifoo/include -Ifoo/usr/include '
            '-Ifoo/include/{arch_triplet} '
            '-Ifoo/usr/include/{arch_triplet}'.format(
                arch_triplet=self.arch_triplet))
        self.assertEqual(get_envvar('CFLAGS'), expected_cflags)
        self.assertEqual(get_envvar('CXXFLAGS'), expected_cflags)
        self.assertEqual(get_envvar('CPPFLAGS'), expected_cflags)

        self.assertEqual(
            get_envvar('LDFLAGS'),
            '-L/user-provided '
            '-Lparts/part2/install/lib -Lfoo/lib -Lfoo/usr/lib '
            '-Lfoo/lib/{arch_triplet} -Lfoo/usr/lib/{arch_triplet}'.format(
                arch_triplet=self.arch_triplet))

        self.assertEqual(
            get_envvar('LD_LIBRARY_PATH'),
            '/user-provided:'
            'parts/part2/install/lib:foo/lib:foo/usr/lib:'
            'foo/lib/{arch_triplet}:foo/usr/lib/{arch_triplet}:'
            'foo/lib:foo/usr/lib:foo/lib/{arch_triplet}:'
            'foo/usr/lib/{arch_triplet}'.format(
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
                    snapcraft.yaml.Validator(data).validate()

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
                    snapcraft.yaml.Validator(data).validate()

                expected_message = ("The 'name' property does not match the "
                                    "required schema: '{}' does not match "
                                    "'^[a-z0-9][a-z0-9+-]*$'").format(name)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Validator(self.data).validate()

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
                snapcraft.yaml.Validator(data).validate()

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
                    snapcraft.yaml.Validator(data).validate()

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

        snapcraft.yaml.Validator(self.data).validate()

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
                snapcraft.yaml.Validator(self.data).validate()

    def test_invalid_restart_condition(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'restart-condition': 'on-watchdog',
            }
        }

        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Validator(self.data).validate()

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
                    snapcraft.yaml.Validator(data).validate()

                expected_message = (
                    "The 'apps' property does not match the required "
                    "schema: Additional properties are not allowed ('{}' "
                    "was unexpected)").format(t)
                self.assertEqual(raised.exception.message, expected_message,
                                 msg=data)

    def test_apps_required_properties(self):
        self.data['apps'] = {'service1': {}}

        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Validator(self.data).validate()

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
                snapcraft.yaml.Validator(self.data).validate()

        expected_path = os.path.join(snapcraft.common.get_schemadir(),
                                     'snapcraft.yaml')
        mock_the_open.assert_called_once_with(expected_path)
        expected_message = ('snapcraft validation file is missing from '
                            'installation path')
        self.assertEqual(raised.exception.message, expected_message)

    def test_icon_missing_is_valid_yaml(self):
        self.mock_path_exists.return_value = False

        snapcraft.yaml.Validator(self.data).validate()

    def test_invalid_part_name_plugin_raises_exception(self):
        self.data['parts']['plugins'] = {'type': 'go'}

        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Validator(self.data).validate()

        expected_message = ("The 'parts' property does not match the "
                            "required schema: Additional properties are not "
                            "allowed ('plugins' was unexpected)")
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_license_hook(self):
        self.data['license'] = 'LICENSE'

        snapcraft.yaml.Validator(self.data).validate()

    def test_full_license_use(self):
        self.data['license'] = 'LICENSE'
        self.data['license-agreement'] = 'explicit'
        self.data['license-version'] = '1.0'

        snapcraft.yaml.Validator(self.data).validate()

    def test_license_with_license_version(self):
        self.data['license'] = 'LICENSE'
        self.data['license-version'] = '1.0'

        snapcraft.yaml.Validator(self.data).validate()

    def test_license_agreement_without_license_raises_exception(self):
        self.data['license-agreement'] = 'explicit'

        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Validator(self.data).validate()

        expected_message = "'license' is a dependency of 'license-agreement'"
        self.assertEqual(raised.exception.message, expected_message,
                         msg=self.data)

    def test_license_version_without_license_raises_exception(self):
        self.data['license-version'] = '1.1'

        with self.assertRaises(SnapcraftSchemaError) as raised:
            snapcraft.yaml.Validator(self.data).validate()

        expected_message = "'license' is a dependency of 'license-version'"
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
        env = snapcraft.yaml._create_pkg_config_override(
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
