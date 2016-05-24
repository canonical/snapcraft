# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
import shutil
import tempfile
from unittest.mock import (
    call,
    Mock,
    MagicMock,
    patch,
)

import fixtures

import snapcraft
from snapcraft.internal import (
    common,
    pluginhandler,
    states,
)
from snapcraft import tests
from snapcraft.plugins import nil


class PluginTestCase(tests.TestCase):

    def test_init_unknown_plugin_must_raise_exception(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        with self.assertRaises(pluginhandler.PluginError) as raised:
            pluginhandler.load_plugin('fake-part', 'test_unexisting')

        self.assertEqual(raised.exception.__str__(),
                         'unknown plugin: test_unexisting')

    def test_fileset_include_excludes(self):
        stage_set = [
            '-etc',
            'opt/something',
            '-usr/lib/*.a',
            'usr/bin',
            '\-everything',
            r'\\a',
        ]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertEqual(include, ['opt/something', 'usr/bin',
                                   '-everything', r'\a'])
        self.assertEqual(exclude, ['etc', 'usr/lib/*.a'])

    @patch.object(snapcraft.plugins.nil.NilPlugin, 'snap_fileset')
    def test_migratable_fileset_for_no_options_modification(
            self, mock_snap_fileset):
        """Making sure migratable_fileset_for() doesn't modify options"""

        mock_snap_fileset.return_value = ['baz']

        handler = pluginhandler.load_plugin('test-part', 'nil')
        handler.code.options.snap = ['foo']
        handler.code.options.stage = ['bar']
        expected_options = copy.deepcopy(handler.code.options)

        handler.migratable_fileset_for('stage')
        self.assertEqual(expected_options.__dict__,
                         handler.code.options.__dict__,
                         'Expected options to be unmodified')

        handler.migratable_fileset_for('strip')
        self.assertEqual(expected_options.__dict__,
                         handler.code.options.__dict__,
                         'Expected options to be unmodified')

    def test_fileset_only_includes(self):
        stage_set = [
            'opt/something',
            'usr/bin',
        ]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertEqual(include, ['opt/something', 'usr/bin'])
        self.assertEqual(exclude, [])

    def test_fileset_only_excludes(self):
        stage_set = [
            '-etc',
            '-usr/lib/*.a',
        ]

        include, exclude = pluginhandler._get_file_list(stage_set)

        self.assertEqual(include, ['*'])
        self.assertEqual(exclude, ['etc', 'usr/lib/*.a'])

    def test_migrate_snap_files(self):
        filesets = {
            'nothing': {
                'fileset': ['-*'],
                'result': []
            },
            'all': {
                'fileset': ['*'],
                'result': [
                    'stage/1',
                    'stage/1/1a/1b',
                    'stage/1/1a',
                    'stage/1/a',
                    'stage/2',
                    'stage/2/2a',
                    'stage/3',
                    'stage/3/a',
                    'stage/a',
                    'stage/b',
                ],
            },
            'no1': {
                'fileset': ['-1'],
                'result': [
                    'stage/2',
                    'stage/2/2a',
                    'stage/3',
                    'stage/3/a',
                    'stage/a',
                    'stage/b',
                ],
            },
            'onlya': {
                'fileset': ['a'],
                'result': [
                    'stage/a',
                ],
            },
            'onlybase': {
                'fileset': ['*', '-*/*'],
                'result': [
                    'stage/a',
                    'stage/b',
                    'stage/1',
                    'stage/2',
                    'stage/3',
                ],
            },
            'nostara': {
                'fileset': ['-*/a'],
                'result': [
                    'stage/1',
                    'stage/1/1a/1b',
                    'stage/1/1a',
                    'stage/2',
                    'stage/2/2a',
                    'stage/3',
                    'stage/a',
                    'stage/b',
                ],
            },
        }

        for key in filesets:
            with self.subTest(key=key):
                tmpdirObject = tempfile.TemporaryDirectory()
                self.addCleanup(tmpdirObject.cleanup)
                tmpdir = tmpdirObject.name

                srcdir = tmpdir + '/install'
                os.makedirs(tmpdir + '/install/1/1a/1b')
                os.makedirs(tmpdir + '/install/2/2a')
                os.makedirs(tmpdir + '/install/3')
                open(tmpdir + '/install/a', mode='w').close()
                open(tmpdir + '/install/b', mode='w').close()
                open(tmpdir + '/install/1/a', mode='w').close()
                open(tmpdir + '/install/3/a', mode='w').close()

                dstdir = tmpdir + '/stage'
                os.makedirs(dstdir)

                files, dirs = pluginhandler._migratable_filesets(
                    filesets[key]['fileset'], srcdir)
                pluginhandler._migrate_files(files, dirs, srcdir, dstdir)

                expected = []
                for item in filesets[key]['result']:
                    expected.append(os.path.join(tmpdir, item))
                expected.sort()

                result = []
                for root, subdirs, files in os.walk(dstdir):
                    for item in files:
                        result.append(os.path.join(root, item))
                    for item in subdirs:
                        result.append(os.path.join(root, item))
                result.sort()

                self.assertEqual(expected, result)

    def test_migrate_snap_files_already_exists(self):
        os.makedirs('install')
        os.makedirs('stage')

        # Place the already-staged file
        with open('stage/foo', 'w') as f:
            f.write('staged')

        # Place the to-be-staged file with the same name
        with open('install/foo', 'w') as f:
            f.write('installed')

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(files, dirs, 'install', 'stage')

        # Verify that the staged file is the one that was staged last
        with open('stage/foo', 'r') as f:
            self.assertEqual(f.read(), 'installed',
                             'Expected staging to allow overwriting of '
                             'already-staged files')

    def test_migrate_files_supports_no_follow_symlinks(self):
        os.makedirs('install')
        os.makedirs('stage')

        with open(os.path.join('install', 'foo'), 'w') as f:
            f.write('installed')

        os.symlink('foo', os.path.join('install', 'bar'))

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=False)

        # Verify that the symlink was preserved
        self.assertTrue(os.path.islink(os.path.join('stage', 'bar')),
                        "Expected migrated 'bar' to still be a symlink.")
        self.assertEqual('foo', os.readlink(os.path.join('stage', 'bar')),
                         "Expected migrated 'bar' to point to 'foo'")

    def test_migrate_files_supports_follow_symlinks(self):
        os.makedirs('install')
        os.makedirs('stage')

        with open(os.path.join('install', 'foo'), 'w') as f:
            f.write('installed')

        os.symlink('foo', os.path.join('install', 'bar'))

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=True)

        # Verify that the symlink was preserved
        self.assertFalse(os.path.islink(os.path.join('stage', 'bar')),
                         "Expected migrated 'bar' to no longer be a symlink.")
        with open(os.path.join('stage', 'bar'), 'r') as f:
            self.assertEqual(f.read(), 'installed',
                             "Expected migrated 'bar' to be a copy of 'foo'")

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_non_local_plugins(self, plugin_mock,
                               local_load_mock, import_mock):
        mock_plugin = Mock()
        mock_plugin.schema.return_value = {}
        plugin_mock.return_value = mock_plugin
        local_load_mock.side_effect = ImportError()
        pluginhandler.PluginHandler(
            'mock', 'mock-part', {}, snapcraft.ProjectOptions(),
            {'properties': {}})
        import_mock.assert_called_with('snapcraft.plugins.mock')
        local_load_mock.assert_called_with('x-mock', self.local_plugins_dir)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_without_project(self, plugin_mock,
                                    local_load_mock, import_mock):
        class OldPlugin(snapcraft.BasePlugin):
            def __init__(self, name, options):
                super().__init__(name, options)

        plugin_mock.return_value = OldPlugin
        local_load_mock.side_effect = ImportError()
        plugin = pluginhandler.PluginHandler(
            'oldplugin', 'fake-part', {'source': '.'},
            snapcraft.ProjectOptions(),
            {'properties': {}})

        self.assertTrue(plugin.code.project is not None)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_without_project_not_from_base(self, plugin_mock,
                                                  local_load_mock,
                                                  import_mock):
        class NonBaseOldPlugin:
            @classmethod
            def schema(cls):
                return {}

            def __init__(self, name, options):
                pass

        plugin_mock.return_value = NonBaseOldPlugin
        local_load_mock.side_effect = ImportError()
        plugin = pluginhandler.PluginHandler(
            'nonbaseoldplugin', 'fake-part', {'source': '.'},
            snapcraft.ProjectOptions(),
            {'properties': {}})

        self.assertTrue(plugin.code.project is not None)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_schema_step_hint_pull(self, plugin_mock,
                                          local_load_mock, import_mock):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['pull-properties'] = ['foo']

                return schema

        plugin_mock.return_value = Plugin
        local_load_mock.side_effect = ImportError()
        pluginhandler.PluginHandler(
            'plugin', 'fake-part', {'source': '.'},
            snapcraft.ProjectOptions(),
            {'properties': {}})

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_schema_step_hint_build(self, plugin_mock,
                                           local_load_mock, import_mock):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['build-properties'] = ['foo']

                return schema

        plugin_mock.return_value = Plugin
        local_load_mock.side_effect = ImportError()
        pluginhandler.PluginHandler(
            'plugin', 'fake-part', {'source': '.'},
            snapcraft.ProjectOptions(),
            {'properties': {}})

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_schema_step_hint_pull_and_build(self, plugin_mock,
                                                    local_load_mock,
                                                    import_mock):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['pull-properties'] = ['foo']
                schema['build-properties'] = ['foo']

                return schema

        plugin_mock.return_value = Plugin
        local_load_mock.side_effect = ImportError()
        pluginhandler.PluginHandler(
            'plugin', 'fake-part', {'source': '.'},
            snapcraft.ProjectOptions(),
            {'properties': {}})

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_schema_invalid_pull_hint(self, plugin_mock,
                                             local_load_mock, import_mock):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['pull-properties'] = ['bar']

                return schema

        plugin_mock.return_value = Plugin
        local_load_mock.side_effect = ImportError()
        with self.assertRaises(pluginhandler.PluginError) as raised:
            pluginhandler.PluginHandler(
                'plugin', 'fake-part', {'source': '.'},
                snapcraft.ProjectOptions(),
                {'properties': {}})

        self.assertEqual(
            "properties failed to load for fake-part: Invalid "
            "pull-properties specified in plugin's schema: ['bar']",
            str(raised.exception))

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_plugin_schema_invalid_build_hint(self, plugin_mock,
                                              local_load_mock, import_mock):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['build-properties'] = ['bar']

                return schema

        plugin_mock.return_value = Plugin
        local_load_mock.side_effect = ImportError()
        with self.assertRaises(pluginhandler.PluginError) as raised:
            pluginhandler.PluginHandler(
                'plugin', 'fake-part', {'source': '.'},
                snapcraft.ProjectOptions(),
                {'properties': {}})

        self.assertEqual(
            "properties failed to load for fake-part: Invalid "
            "build-properties specified in plugin's schema: ['bar']",
            str(raised.exception))

    def test_filesets_includes_without_relative_paths(self):
        with self.assertRaises(pluginhandler.PluginError) as raised:
            pluginhandler._get_file_list(['rel', '/abs/include'])

        self.assertEqual(
            'path "/abs/include" must be relative', str(raised.exception))

    def test_filesets_exlcudes_without_relative_paths(self):
        with self.assertRaises(pluginhandler.PluginError) as raised:
            pluginhandler._get_file_list(['rel', '-/abs/exclude'])

        self.assertEqual(
            'path "/abs/exclude" must be relative', str(raised.exception))


class MigratableFilesetsTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        os.makedirs('install/foo/bar/baz')
        open('install/1', 'w').close()
        open('install/foo/2', 'w').close()
        open('install/foo/bar/3', 'w').close()
        open('install/foo/bar/baz/4', 'w').close()

    def test_migratable_filesets_everything(self):
        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        self.assertEqual({'1', 'foo/2', 'foo/bar/3', 'foo/bar/baz/4'}, files)
        self.assertEqual({'foo', 'foo/bar', 'foo/bar/baz'}, dirs)

    def test_migratable_filesets_foo(self):
        files, dirs = pluginhandler._migratable_filesets(['foo'], 'install')
        self.assertEqual({'foo/2', 'foo/bar/3', 'foo/bar/baz/4'}, files)
        self.assertEqual({'foo', 'foo/bar', 'foo/bar/baz'}, dirs)

    def test_migratable_filesets_everything_in_foo(self):
        files, dirs = pluginhandler._migratable_filesets(['foo/*'], 'install')
        self.assertEqual({'foo/2', 'foo/bar/3', 'foo/bar/baz/4'}, files)
        self.assertEqual({'foo', 'foo/bar', 'foo/bar/baz'}, dirs)

    def test_migratable_filesets_root_file(self):
        files, dirs = pluginhandler._migratable_filesets(['1'], 'install')
        self.assertEqual({'1'}, files)
        self.assertEqual(set(), dirs)

    def test_migratable_filesets_single_nested_file(self):
        files, dirs = pluginhandler._migratable_filesets(['foo/2'], 'install')
        self.assertEqual({'foo/2'}, files)
        self.assertEqual({'foo'}, dirs)

    def test_migratable_filesets_single_really_nested_file(self):
        files, dirs = pluginhandler._migratable_filesets(['foo/bar/2'],
                                                         'install')
        self.assertEqual({'foo/bar/2'}, files)
        self.assertEqual({'foo', 'foo/bar'}, dirs)

    def test_migratable_filesets_single_really_really_nested_file(self):
        files, dirs = pluginhandler._migratable_filesets(['foo/bar/baz/3'],
                                                         'install')
        self.assertEqual({'foo/bar/baz/3'}, files)
        self.assertEqual({'foo', 'foo/bar', 'foo/bar/baz'}, dirs)


class PluginMakedirsTestCase(tests.TestCase):

    scenarios = [
        ('existing_dirs', {'make_dirs': True}),
        ('unexisting_dirs', {'make_dirs': False})
    ]

    def get_plugin_dirs(self, part_name):
        parts_dir = os.path.join(self.path, 'parts')
        return [
            os.path.join(parts_dir, part_name, 'src'),
            os.path.join(parts_dir, part_name, 'build'),
            os.path.join(parts_dir, part_name, 'install'),
            os.path.join(self.path, 'stage'),
            os.path.join(self.path, 'snap')
        ]

    def test_makedirs_with_existing_dirs(self):
        part_name = 'test_part'
        dirs = self.get_plugin_dirs(part_name)
        if self.make_dirs:
            os.makedirs(os.path.join('parts', part_name))
            for d in dirs:
                os.mkdir(d)

        p = pluginhandler.load_plugin(part_name, 'nil')
        p.makedirs()
        for d in dirs:
            self.assertTrue(os.path.exists(d), '{} does not exist'.format(d))


class StateTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        part_name = 'test_part'
        self.handler = pluginhandler.load_plugin(part_name, 'nil')
        self.handler.makedirs()

    def test_mark_done_clears_later_steps(self):
        for index, step in enumerate(common.COMMAND_ORDER):
            shutil.rmtree(self.parts_dir)
            with self.subTest('{} step'.format(step)):
                handler = pluginhandler.load_plugin('foo', 'nil')
                handler.makedirs()

                for later_step in common.COMMAND_ORDER[index+1:]:
                    open(handler._step_state_file(later_step), 'w').close()

                handler.mark_done(step)

                for later_step in common.COMMAND_ORDER[index+1:]:
                    self.assertFalse(
                        os.path.exists(handler._step_state_file(later_step)),
                        'Expected later step states to be cleared')

    def test_state_file_migration(self):
        part_name = 'foo'
        for step in common.COMMAND_ORDER:
            shutil.rmtree(self.parts_dir)
            with self.subTest('{} step'.format(step)):
                part_dir = os.path.join(self.parts_dir, part_name)
                os.makedirs(part_dir)
                with open(os.path.join(part_dir, 'state'), 'w') as f:
                    f.write(step)

                handler = pluginhandler.load_plugin(part_name, 'nil')
                self.assertEqual(step, handler.last_step())

    @patch('snapcraft.internal.repo.Ubuntu')
    def test_pull_state(self, ubuntu_mock):
        self.assertEqual(None, self.handler.last_step())

        self.handler.pull()

        self.assertEqual('pull', self.handler.last_step())
        state = self.handler.get_state('pull')

        self.assertTrue(state, 'Expected pull to save state YAML')
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(0, len(state.properties))
        self.assertTrue(type(state.project_options) is dict)
        self.assertTrue('deb_arch' in state.project_options)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_pull_state_with_properties(self, plugin_mock, local_load_mock,
                                        import_mock):
        self.handler.pull_properties = ['foo']
        self.handler.code.options.foo = 'bar'

        self.assertEqual(None, self.handler.last_step())

        self.handler.pull()

        self.assertEqual('pull', self.handler.last_step())
        state = self.handler.get_state('pull')

        self.assertTrue(state, 'Expected pull to save state YAML')
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is dict)
        self.assertTrue('foo' in state.properties)
        self.assertEqual(state.properties['foo'], 'bar')
        self.assertTrue(type(state.project_options) is dict)
        self.assertTrue('deb_arch' in state.project_options)

    @patch.object(nil.NilPlugin, 'clean_pull')
    def test_clean_pull_state(self, mock_clean_pull):
        self.assertEqual(None, self.handler.last_step())

        self.handler.pull()

        self.handler.clean_pull()

        # Verify that the plugin had clean_pull() called
        mock_clean_pull.assert_called_once_with()

        self.assertEqual(None, self.handler.last_step())

    def test_build_state(self):
        self.assertEqual(None, self.handler.last_step())

        self.handler.build()

        self.assertEqual('build', self.handler.last_step())
        state = self.handler.get_state('build')

        self.assertTrue(state, 'Expected build to save state YAML')
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(0, len(state.properties))
        self.assertTrue(type(state.project_options) is dict)
        self.assertTrue('deb_arch' in state.project_options)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_build_state_with_properties(self, plugin_mock, local_load_mock,
                                         import_mock):
        self.handler.build_properties = ['foo']
        self.handler.code.options.foo = 'bar'

        self.assertEqual(None, self.handler.last_step())

        self.handler.build()

        self.assertEqual('build', self.handler.last_step())
        state = self.handler.get_state('build')

        self.assertTrue(state, 'Expected build to save state YAML')
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is dict)
        self.assertTrue('foo' in state.properties)
        self.assertEqual(state.properties['foo'], 'bar')
        self.assertTrue(type(state.project_options) is dict)
        self.assertTrue('deb_arch' in state.project_options)

    @patch.object(nil.NilPlugin, 'clean_build')
    def test_clean_build_state(self, mock_clean_build):
        self.assertEqual(None, self.handler.last_step())

        self.handler.mark_done('pull')
        self.handler.build()

        self.handler.clean_build()

        # Verify that the plugin had clean_build() called
        mock_clean_build.assert_called_once_with()

        self.assertEqual('pull', self.handler.last_step())

    def test_stage_state(self):
        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()

        self.assertEqual('stage', self.handler.last_step())
        state = self.handler.get_state('stage')

        self.assertTrue(state, 'Expected stage to save state YAML')
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/2' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertTrue('stage' in state.properties)
        self.assertEqual(state.properties['stage'], ['*'])
        self.assertTrue(type(state.project_options) is dict)
        self.assertEqual(0, len(state.project_options))

    def test_stage_state_with_stage_keyword(self):
        self.handler.code.options.stage = ['bin/1']

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()

        self.assertEqual('stage', self.handler.last_step())
        state = self.handler.get_state('stage')

        self.assertTrue(state, 'Expected stage to save state YAML')
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(1, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertTrue('stage' in state.properties)
        self.assertEqual(state.properties['stage'], ['bin/1'])
        self.assertTrue(type(state.project_options) is dict)
        self.assertEqual(0, len(state.project_options))

        self.assertEqual('stage', self.handler.last_step())

    def test_clean_stage_state(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.stage_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')

        self.handler.mark_done(
            'stage', states.StageState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_stage({})

        self.assertEqual('build', self.handler.last_step())
        self.assertFalse(os.path.exists(bindir))

    def test_clean_stage_state_multiple_parts(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.stage_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()
        open(os.path.join(bindir, '3'), 'w').close()

        self.handler.mark_done('build')

        self.handler.mark_done(
            'stage', states.StageState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_stage({})

        self.assertEqual('build', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertFalse(os.path.exists(os.path.join(bindir, '2')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '3')),
            "Expected 'bin/3' to remain as it wasn't staged by this part")

    def test_clean_stage_state_common_files(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.stage_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')

        self.handler.mark_done(
            'stage', states.StageState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_stage({
            'other_part': states.StageState({'bin/2'}, {'bin'})
        })

        self.assertEqual('build', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '2')),
            "Expected 'bin/2' to remain as it's required by other parts")

    def test_clean_stage_old_state(self):
        self.handler.mark_done('stage', None)
        with self.assertRaises(pluginhandler.MissingState) as raised:
            self.handler.clean_stage({})

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'stage': Missing necessary state. "
            "This won't work until a complete clean has occurred.")

    @patch('snapcraft.internal.pluginhandler._find_dependencies')
    @patch('shutil.copy')
    def test_strip_state(self, mock_copy, mock_find_dependencies):
        mock_find_dependencies.return_value = set()

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.strip()

        self.assertEqual('strip', self.handler.last_step())
        mock_find_dependencies.assert_called_once_with(self.handler.snapdir)
        self.assertFalse(mock_copy.called)

        state = self.handler.get_state('strip')

        self.assertTrue(type(state) is states.StripState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/2' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertEqual(0, len(state.dependency_paths))
        self.assertTrue('snap' in state.properties)
        self.assertEqual(state.properties['snap'], ['*'])
        self.assertTrue(type(state.project_options) is dict)
        self.assertEqual(0, len(state.project_options))

    @patch('snapcraft.internal.pluginhandler._find_dependencies')
    @patch('snapcraft.internal.pluginhandler._migrate_files')
    def test_strip_state_with_dependencies(self, mock_migrate_files,
                                           mock_find_dependencies):
        mock_find_dependencies.return_value = {
            '/foo/bar/baz',
            '{}/lib1/installed'.format(self.handler.installdir),
            '{}/lib2/staged'.format(self.handler.stagedir),
        }

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.strip()

        self.assertEqual('strip', self.handler.last_step())
        mock_find_dependencies.assert_called_once_with(self.handler.snapdir)
        mock_migrate_files.assert_has_calls([
            call({'bin/1', 'bin/2'}, {'bin'}, self.handler.stagedir,
                 self.handler.snapdir),
            call({'foo/bar/baz'}, {'foo/bar'}, '/', self.handler.snapdir,
                 follow_symlinks=True),
        ])

        state = self.handler.get_state('strip')

        self.assertTrue(type(state) is states.StripState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/2' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertEqual(3, len(state.dependency_paths))
        self.assertTrue('foo/bar' in state.dependency_paths)
        self.assertTrue('lib1' in state.dependency_paths)
        self.assertTrue('lib2' in state.dependency_paths)
        self.assertTrue('snap' in state.properties)
        self.assertEqual(state.properties['snap'], ['*'])
        self.assertTrue(type(state.project_options) is dict)
        self.assertEqual(0, len(state.project_options))

    @patch('snapcraft.internal.pluginhandler._find_dependencies')
    @patch('shutil.copy')
    def test_strip_state_with_snap_keyword(self, mock_copy,
                                           mock_find_dependencies):
        mock_find_dependencies.return_value = set()
        self.handler.code.options.snap = ['bin/1']

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.strip()

        self.assertEqual('strip', self.handler.last_step())
        mock_find_dependencies.assert_called_once_with(self.handler.snapdir)
        self.assertFalse(mock_copy.called)

        state = self.handler.get_state('strip')

        self.assertTrue(type(state) is states.StripState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is dict)
        self.assertEqual(1, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertEqual(0, len(state.dependency_paths))
        self.assertTrue('snap' in state.properties)
        self.assertEqual(state.properties['snap'], ['bin/1'])
        self.assertTrue(type(state.project_options) is dict)
        self.assertEqual(0, len(state.project_options))

    def test_clean_strip_state(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.snap_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'strip', states.StripState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_strip({})

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(bindir))

    def test_clean_strip_state_multiple_parts(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.snap_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()
        open(os.path.join(bindir, '3'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'strip', states.StripState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_strip({})

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertFalse(os.path.exists(os.path.join(bindir, '2')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '3')),
            "Expected 'bin/3' to remain as it wasn't stripped by this part")

    def test_clean_strip_state_common_files(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.snap_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'strip', states.StripState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_strip({
            'other_part': states.StripState({'bin/2'}, {'bin'})
        })

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '2')),
            "Expected 'bin/2' to remain as it's required by other parts")

    def test_clean_strip_old_state(self):
        self.handler.mark_done('strip', None)
        with self.assertRaises(pluginhandler.MissingState) as raised:
            self.handler.clean_strip({})

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'strip': Missing necessary state. "
            "This won't work until a complete clean has occurred.")


class IsDirtyTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.handler = pluginhandler.load_plugin(
            'test-part', 'nil', project_options=snapcraft.ProjectOptions(
                target_deb_arch='amd64'))
        self.handler.makedirs()

    def test_strip_is_dirty(self):
        self.handler.code.options.snap = ['foo']
        self.handler.mark_done(
            'strip', states.StripState(
                set(), set(), set(), self.handler.code.options))
        self.assertFalse(self.handler.is_clean('strip'),
                         'Strip step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('strip'),
                         'Strip step was unexpectedly dirty')

        # Change the `snap` keyword-- thereby making the strip step dirty.
        self.handler.code.options.snap = ['bar']
        self.assertFalse(self.handler.is_clean('strip'),
                         'Strip step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('strip'),
                        'Expected strip step to be dirty')

    def test_strip_not_dirty_if_clean(self):
        self.assertTrue(self.handler.is_clean('strip'),
                        'Expected vanilla handler to have clean strip step')
        self.assertFalse(
            self.handler.is_dirty('strip'),
            'Expected vanilla handler to not have a dirty strip step')

    def test_stage_is_dirty(self):
        self.handler.code.options.stage = ['foo']
        self.handler.mark_done(
            'stage', states.StageState(
                set(), set(), self.handler.code.options))
        self.assertFalse(self.handler.is_clean('stage'),
                         'Stage step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('stage'),
                         'Stage step was unexpectedly dirty')

        # Change the `stage` keyword-- thereby making the stage step dirty.
        self.handler.code.options.stage = ['bar']
        self.assertFalse(self.handler.is_clean('stage'),
                         'Stage step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('stage'),
                        'Expected stage step to be dirty')

    def test_stage_not_dirty_if_clean(self):
        self.assertTrue(self.handler.is_clean('stage'),
                        'Expected vanilla handler to have clean stage step')
        self.assertFalse(
            self.handler.is_dirty('stage'),
            'Expected vanilla handler to not have a dirty stage step')

    def test_build_is_dirty_from_options(self):
        self.handler.build_properties = ['foo']
        self.handler.code.options.foo = ['bar']
        self.handler.mark_done(
            'build', states.BuildState(self.handler.build_properties,
                                       self.handler.code.options,
                                       snapcraft.ProjectOptions()))
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('build'),
                         'Build step was unexpectedly dirty')

        # Change the `foo` keyword-- thereby making the build step dirty.
        self.handler.code.options.foo = ['baz']
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('build'),
                        'Expected build step to be dirty')

    @patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    def test_build_is_dirty_from_project(self, mock_enable_cross_compilation):
        self.handler.mark_done(
            'build', states.BuildState(self.handler.build_properties,
                                       self.handler.code.options,
                                       snapcraft.ProjectOptions()))
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('build'),
                         'Build step was unexpectedly dirty')

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        self.handler = pluginhandler.load_plugin(
            'test-part', 'nil', project_options=snapcraft.ProjectOptions(
                target_deb_arch='armhf'))
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('build'),
                        'Expected build step to be dirty')

    def test_build_not_dirty_if_clean(self):
        self.assertTrue(self.handler.is_clean('build'),
                        'Expected vanilla handler to have clean build step')
        self.assertFalse(
            self.handler.is_dirty('build'),
            'Expected vanilla handler to not have a dirty build step')

    def test_pull_is_dirty_from_options(self):
        self.handler.pull_properties = ['foo']
        self.handler.code.options.foo = ['bar']
        self.handler.mark_done(
            'pull', states.PullState(self.handler.pull_properties,
                                     self.handler.code.options,
                                     snapcraft.ProjectOptions()))
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('pull'),
                         'Pull step was unexpectedly dirty')

        # Change the `foo` keyword-- thereby making the pull step dirty.
        self.handler.code.options.foo = ['baz']
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('pull'),
                        'Expected pull step to be dirty')

    @patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    def test_pull_is_dirty_from_project(self, mock_enable_cross_compilation):
        self.handler.mark_done(
            'pull', states.PullState(self.handler.pull_properties,
                                     self.handler.code.options,
                                     snapcraft.ProjectOptions()))
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('pull'),
                         'Pull step was unexpectedly dirty')

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        self.handler = pluginhandler.load_plugin(
            'test-part', 'nil', project_options=snapcraft.ProjectOptions(
                target_deb_arch='armhf'))
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('pull'),
                        'Expected pull step to be dirty')

    def test_pull_not_dirty_if_clean(self):
        self.assertTrue(self.handler.is_clean('pull'),
                        'Expected vanilla handler to have clean pull step')
        self.assertFalse(
            self.handler.is_dirty('pull'),
            'Expected vanilla handler to not have a dirty pull step')


class CleanTestCase(tests.TestCase):

    @patch.object(pluginhandler.PluginHandler, 'is_clean')
    @patch('os.rmdir')
    @patch('os.listdir')
    @patch('os.path.exists')
    def test_clean_part_that_exists(self, mock_exists, mock_listdir,
                                    mock_rmdir, mock_is_clean):
        mock_exists.return_value = True
        mock_listdir.return_value = False
        mock_is_clean.return_value = True

        part_name = 'test_part'
        p = pluginhandler.load_plugin(part_name, 'nil')
        p.clean()

        partdir = os.path.join(
            os.path.abspath(os.curdir), 'parts', part_name)
        mock_exists.assert_called_once_with(partdir)
        mock_listdir.assert_called_once_with(partdir)
        mock_rmdir.assert_called_once_with(partdir)

    @patch('os.rmdir')
    @patch('os.listdir')
    @patch('os.path.exists')
    def test_clean_part_already_clean(self, mock_exists, mock_listdir,
                                      mock_rmdir):
        mock_exists.return_value = False

        part_name = 'test_part'
        p = pluginhandler.load_plugin(part_name, 'nil')
        p.clean()

        partdir = os.path.join(
            os.path.abspath(os.curdir), 'parts', part_name)
        mock_exists.assert_has_calls([call(partdir)])
        self.assertFalse(mock_listdir.called)
        self.assertFalse(mock_rmdir.called)

    @patch.object(pluginhandler.PluginHandler, 'is_clean')
    @patch('os.rmdir')
    @patch('os.listdir')
    @patch('os.path.exists')
    def test_clean_part_remaining_parts(self, mock_exists, mock_listdir,
                                        mock_rmdir, mock_is_clean):
        mock_exists.return_value = True
        mock_listdir.return_value = True
        mock_is_clean.return_value = True

        part_name = 'test_part'
        p = pluginhandler.load_plugin(part_name, 'nil')
        p.clean()

        partdir = os.path.join(
            os.path.abspath(os.curdir), 'parts', part_name)
        mock_exists.assert_called_once_with(partdir)
        mock_listdir.assert_called_once_with(partdir)
        self.assertFalse(mock_rmdir.called)

    def clear_common_directories(self):
        if os.path.exists(self.parts_dir):
            shutil.rmtree(self.parts_dir)

        if os.path.exists(self.stage_dir):
            shutil.rmtree(self.stage_dir)

        if os.path.exists(self.snap_dir):
            shutil.rmtree(self.snap_dir)

    def test_clean_strip(self):
        filesets = {
            'all': {
                'fileset': ['*'],
            },
            'no1': {
                'fileset': ['-1'],
            },
            'onlya': {
                'fileset': ['a'],
            },
            'onlybase': {
                'fileset': ['*', '-*/*'],
            },
            'only1a': {
                'fileset': ['1/a']
            },
            'nostara': {
                'fileset': ['-*/a'],
            },
        }

        for key, value in filesets.items():
            with self.subTest(key=key):
                self.clear_common_directories()

                schema = {'snap': {'type': 'array'}}
                properties = {'snap': value['fileset']}

                handler = pluginhandler.load_plugin(
                    'test_part', 'nil', properties, snapcraft.ProjectOptions(),
                    schema)
                handler.makedirs()

                installdir = handler.code.installdir
                os.makedirs(installdir + '/1/1a/1b')
                os.makedirs(installdir + '/2/2a')
                os.makedirs(installdir + '/3')
                open(installdir + '/a', mode='w').close()
                open(installdir + '/b', mode='w').close()
                open(installdir + '/1/a', mode='w').close()
                open(installdir + '/3/a', mode='w').close()

                handler.mark_done('build')

                # Stage the installed files
                handler.stage()

                # Now strip them
                handler.strip()

                self.assertTrue(os.listdir(self.snap_dir))

                handler.clean_strip({})

                self.assertFalse(os.listdir(self.snap_dir),
                                 'Expected snapdir to be completely cleaned')

    def test_clean_strip_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = pluginhandler.load_plugin('part1', 'nil')
        handler1.makedirs()

        bindir = os.path.join(handler1.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()

        handler1.mark_done('build')

        # Now create part2 and get it through the "build" step.
        handler2 = pluginhandler.load_plugin('part2', 'nil')
        handler2.makedirs()

        bindir = os.path.join(handler2.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '2'), 'w').close()

        handler2.mark_done('build')

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # And strip both parts
        handler1.strip()
        handler2.strip()

        # Verify that part1's file has been stripped
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')))

        # Verify that part2's file has been stripped
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')))

        # Now clean the strip step for part1
        handler1.clean_strip({})

        # Verify that part1's file is no longer stripped
        self.assertFalse(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')),
            "Expected part1's stripped files to be cleaned")

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')),
            "Expected part2's stripped files to be untouched")

    def test_clean_strip_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        bindir = os.path.join(handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        handler.mark_done('build')
        handler.stage()
        handler.strip()

        # Verify that both files have been stripped
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')))
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')))

        # Now update the `snap` fileset to only snap one of these files
        handler.code.options.snap = ['bin/1']

        # Now clean the strip step for part1
        handler.clean_strip({})

        # Verify that part1's file is no longer stripped
        self.assertFalse(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')),
            'Expected bin/1 to be cleaned')
        self.assertFalse(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')),
            'Expected bin/2 to be cleaned as well, even though the filesets '
            'changed since it was stripped.')

    def test_clean_old_strip_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        open(os.path.join(self.snap_dir, '1'), 'w').close()

        handler.mark_done('strip', None)

        self.assertTrue(os.path.exists(handler.code.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.code.partdir))

    def test_clean_strip_old_strip_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        stripped_file = os.path.join(self.snap_dir, '1')
        open(stripped_file, 'w').close()

        handler.mark_done('strip', None)

        with self.assertRaises(pluginhandler.MissingState) as raised:
            handler.clean(step='strip')

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'strip': Missing necessary state. "
            "This won't work until a complete clean has occurred.")

        self.assertTrue(os.path.isfile(stripped_file))

    def test_clean_stage(self):
        filesets = {
            'all': {
                'fileset': ['*'],
            },
            'no1': {
                'fileset': ['-1'],
            },
            'onlya': {
                'fileset': ['a'],
            },
            'onlybase': {
                'fileset': ['*', '-*/*'],
            },
            'only1a': {
                'fileset': ['1/a']
            },
            'nostara': {
                'fileset': ['-*/a'],
            },
        }

        for key, value in filesets.items():
            with self.subTest(key=key):
                self.clear_common_directories()

                schema = {'stage': {'type': 'array'}}
                properties = {'stage': value['fileset']}

                handler = pluginhandler.load_plugin(
                    'test_part', 'nil', properties, snapcraft.ProjectOptions(),
                    schema)
                handler.makedirs()

                installdir = handler.code.installdir
                os.makedirs(installdir + '/1/1a/1b')
                os.makedirs(installdir + '/2/2a')
                os.makedirs(installdir + '/3')
                open(installdir + '/a', mode='w').close()
                open(installdir + '/b', mode='w').close()
                open(installdir + '/1/a', mode='w').close()
                open(installdir + '/3/a', mode='w').close()

                handler.mark_done('build')

                # Stage the installed files
                handler.stage()

                self.assertTrue(os.listdir(self.stage_dir))

                handler.clean_stage({})

                self.assertFalse(os.listdir(self.stage_dir),
                                 'Expected snapdir to be completely cleaned')

    def test_clean_stage_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = pluginhandler.load_plugin('part1', 'nil')
        handler1.makedirs()

        bindir = os.path.join(handler1.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()

        handler1.mark_done('build')

        # Now create part2 and get it through the "build" step.
        handler2 = pluginhandler.load_plugin('part2', 'nil')
        handler2.makedirs()

        bindir = os.path.join(handler2.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '2'), 'w').close()

        handler2.mark_done('build')

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # Verify that part1's file has been staged
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '1')))

        # Verify that part2's file has been staged
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '2')))

        # Now clean the stage step for part1
        handler1.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '1')),
            "Expected part1's staged files to be cleaned")

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '2')),
            "Expected part2's staged files to be untouched")

    def test_clean_stage_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        bindir = os.path.join(handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        handler.mark_done('build')
        handler.stage()

        # Verify that both files have been staged
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '1')))
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '2')))

        # Now update the `stage` fileset to only snap one of these files
        handler.code.options.stage = ['bin/1']

        # Now clean the strip step for part1
        handler.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '1')),
            'Expected bin/1 to be cleaned')
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, 'bin', '2')),
            'Expected bin/2 to be cleaned as well, even though the filesets '
            'changed since it was staged.')

    def test_clean_old_stage_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        open(os.path.join(self.stage_dir, '1'), 'w').close()

        handler.mark_done('stage', None)

        self.assertTrue(os.path.exists(handler.code.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.code.partdir))

    def test_clean_stage_old_stage_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        staged_file = os.path.join(self.stage_dir, '1')
        open(staged_file, 'w').close()

        handler.mark_done('stage', None)

        with self.assertRaises(pluginhandler.MissingState) as raised:
            handler.clean(step='stage')

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'stage': Missing necessary state. "
            "This won't work until a complete clean has occurred.")

        self.assertTrue(os.path.isfile(staged_file))


class PerStepCleanTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.manager_mock = MagicMock()

        patcher = patch.object(pluginhandler.PluginHandler, 'clean_pull')
        self.manager_mock.attach_mock(patcher.start(), 'clean_pull')
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, 'clean_build')
        self.manager_mock.attach_mock(patcher.start(), 'clean_build')
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, 'clean_stage')
        self.manager_mock.attach_mock(patcher.start(), 'clean_stage')
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, 'clean_strip')
        self.manager_mock.attach_mock(patcher.start(), 'clean_strip')
        self.addCleanup(patcher.stop)

    def test_clean_with_hint(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='pull', hint='foo')

        # Verify the step cleaning order
        self.assertEqual(4, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}, 'foo'),
            call.clean_stage({}, 'foo'),
            call.clean_build('foo'),
            call.clean_pull('foo'),
        ])

    def test_clean_pull_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='pull')

        # Verify the step cleaning order
        self.assertEqual(4, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}, ''),
            call.clean_stage({}, ''),
            call.clean_build(''),
            call.clean_pull(''),
        ])

    def test_clean_build_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='build')

        # Verify the step cleaning order
        self.assertEqual(3, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}, ''),
            call.clean_stage({}, ''),
            call.clean_build(''),
        ])

    def test_clean_stage_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='stage')

        # Verify the step cleaning order
        self.assertEqual(2, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}, ''),
            call.clean_stage({}, ''),
        ])

    def test_clean_strip_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='strip')

        # Verify the step cleaning order
        self.assertEqual(1, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}, ''),
        ])


class CollisionTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = pluginhandler.load_plugin('part1', 'nil')
        part1.code.installdir = tmpdir + '/install1'
        os.makedirs(part1.installdir + '/a')
        open(part1.installdir + '/a/1', mode='w').close()

        part2 = pluginhandler.load_plugin('part2', 'nil')
        part2.code.installdir = tmpdir + '/install2'
        os.makedirs(part2.installdir + '/a')
        with open(part2.installdir + '/1', mode='w') as f:
            f.write('1')
        open(part2.installdir + '/2', mode='w').close()
        with open(part2.installdir + '/a/2', mode='w') as f:
            f.write('a/2')

        part3 = pluginhandler.load_plugin('part3', 'nil')
        part3.code.installdir = tmpdir + '/install3'
        os.makedirs(part3.installdir + '/a')
        os.makedirs(part3.installdir + '/b')
        with open(part3.installdir + '/1', mode='w') as f:
            f.write('2')
        with open(part2.installdir + '/2', mode='w') as f:
            f.write('1')
        open(part3.installdir + '/a/2', mode='w').close()

        self.part1 = part1
        self.part2 = part2
        self.part3 = part3

    def test_no_collisions(self):
        """No exception is expected as there are no collisions."""
        pluginhandler.check_for_collisions([self.part1, self.part2])

    def test_collisions_between_two_parts(self):
        with self.assertRaises(EnvironmentError) as raised:
            pluginhandler.check_for_collisions(
                [self.part1, self.part2, self.part3])

        self.assertEqual(
            raised.exception.__str__(),
            "Parts 'part2' and 'part3' have the following file paths in "
            "common which have different contents:\n1\na/2")


class StageEnvTestCase(tests.TestCase):

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
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(
                pluginhandler._expand_env(subject, self.stage_dir), expected)

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
            self.assertEqual(
                pluginhandler._expand_env(subject, self.stage_dir), expected)

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
            self.assertEqual(
                pluginhandler._expand_env(subject, self.stage_dir), expected)

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
            self.assertEqual(
                pluginhandler._expand_env(subject, self.stage_dir), expected)

    def test_string_replacement_with_complex_data(self):
        subject = {
            'filesets': {
                'files': [
                    'somefile',
                    '$SNAPCRAFT_STAGE/file1',
                    'SNAPCRAFT_STAGE/really',
                ]
            },
            'configFlags': [
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
            'configFlags': [
                '--with-python',
                '--with-swig {}/swig'.format(self.stage_dir),
            ],
        }

        self.assertEqual(
            pluginhandler._expand_env(subject, self.stage_dir), expected)


class FindDependenciesTestCase(tests.TestCase):

    @patch('magic.open')
    @patch('snapcraft.internal.libraries.get_dependencies')
    def test_find_dependencies(self, mock_dependencies, mock_magic):
        workdir = os.path.join(os.getcwd(), 'workdir')
        os.makedirs(workdir)

        linked_elf_path = os.path.join(workdir, 'linked')
        open(linked_elf_path, 'w').close()

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'dynamically linked interpreter /lib64/ld-linux-x86-64.so.2, '
            'for GNU/Linux 2.6.32, BuildID[sha1]=XYZ, stripped')

        mock_dependencies.return_value = ['/usr/lib/libDepends.so']

        dependencies = pluginhandler._find_dependencies(workdir)

        mock_ms.file.assert_called_once_with(bytes(linked_elf_path, 'utf-8'))
        self.assertEqual(dependencies, {'/usr/lib/libDepends.so'})

    @patch('magic.open')
    @patch('snapcraft.internal.libraries.get_dependencies')
    def test_find_dependencies_skip_object_files(self, mock_dependencies,
                                                 mock_magic):
        workdir = os.path.join(os.getcwd(), 'workdir')
        os.makedirs(workdir)
        open(os.path.join(workdir, 'object_file.o'), 'w').close()

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'dynamically linked interpreter /lib64/ld-linux-x86-64.so.2, '
            'for GNU/Linux 2.6.32, BuildID[sha1]=XYZ, stripped')

        mock_dependencies.return_value = ['/usr/lib/libDepends.so']

        dependencies = pluginhandler._find_dependencies(workdir)

        self.assertFalse(mock_ms.file.called,
                         'Expected object file to be skipped')
        self.assertEqual(dependencies, set())

    @patch('magic.open')
    @patch('snapcraft.internal.libraries.get_dependencies')
    def test_no_find_dependencies_of_non_dynamically_linked(
            self, mock_dependencies, mock_magic):
        workdir = os.path.join(os.getcwd(), 'workdir')
        os.makedirs(workdir)

        statically_linked_elf_path = os.path.join(workdir, 'statically-linked')
        open(statically_linked_elf_path, 'w').close()

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'statically linked, for GNU/Linux 2.6.32, '
            'BuildID[sha1]=XYZ, stripped')

        dependencies = pluginhandler._find_dependencies(workdir)

        mock_ms.file.assert_called_once_with(
            bytes(statically_linked_elf_path, 'utf-8'))

        self.assertFalse(
            mock_dependencies.called,
            'statically linked files should not have library dependencies')

        self.assertFalse(dependencies)

    @patch('magic.open')
    @patch('snapcraft.internal.libraries.get_dependencies')
    def test_no_find_dependencies_of_non_elf_files(
            self, mock_dependencies, mock_magic):
        workdir = os.path.join(os.getcwd(), 'workdir')
        os.makedirs(workdir)

        non_elf_path = os.path.join(workdir, 'non-elf')
        open(non_elf_path, 'w').close()

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = 'JPEG image data, Exif standard: ...'

        dependencies = pluginhandler._find_dependencies(workdir)

        mock_ms.file.assert_called_once_with(bytes(non_elf_path, 'utf-8'))

        self.assertFalse(
            mock_dependencies.called,
            'non elf files should not have library dependencies')

        self.assertFalse(
            dependencies,
            'non elf files should not have library dependencies')

    @patch('magic.open')
    @patch('snapcraft.internal.libraries.get_dependencies')
    def test_no_find_dependencies_of_symlinks(
            self, mock_dependencies, mock_magic):
        workdir = os.path.join(os.getcwd(), 'workdir')
        os.makedirs(workdir)

        symlinked_path = os.path.join(workdir, 'symlinked')
        os.symlink('/bin/dash', symlinked_path)

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0

        dependencies = pluginhandler._find_dependencies(workdir)

        self.assertFalse(
            mock_ms.file.called, 'magic is not needed for symlinks')

        self.assertFalse(
            mock_dependencies.called,
            'statically linked files should not have library dependencies')

        self.assertFalse(
            dependencies,
            'statically linked files should not have library dependencies')

    @patch('magic.open')
    def test_fail_to_load_magic_raises_exception(self, mock_magic):
        mock_magic.return_value.load.return_value = 1

        with self.assertRaises(RuntimeError) as raised:
            pluginhandler._find_dependencies('.')

        self.assertEqual(
            raised.exception.__str__(), 'Cannot load magic header detection')
