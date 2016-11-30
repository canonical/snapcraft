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

from collections import OrderedDict
import copy
import logging
import os
import shutil
import stat
import sys
import tempfile
from unittest.mock import (
    call,
    Mock,
    MagicMock,
    patch,
)

import fixtures

import snapcraft
from snapcraft.internal.errors import SnapcraftPartConflictError
from snapcraft.internal import (
    common,
    lifecycle,
    pluginhandler,
    repo,
    states,
)
from snapcraft import tests
from snapcraft.tests import fixture_setup
from snapcraft.plugins import nil
from snapcraft.plugins import cmake  # noqa


class TestPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'type': 'object',
            'additionalProperties': False,
            'properties': {
                'test-property': {
                    'type': 'string'
                }
            },
        }

    @classmethod
    def get_pull_properties(cls):
        return ['test-property']

    @classmethod
    def get_build_properties(cls):
        return ['test-property']


class PluginTestCase(tests.TestCase):

    def test_build_with_subdir_copies_sourcedir(self):
        handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            part_properties={'source-subdir': 'src'},
            part_schema={'source-subdir': {'type': 'string'}})

        sourcedir = handler.sourcedir
        source_subdir = handler.code.options.source_subdir

        subdir = os.path.join(sourcedir, source_subdir)
        os.makedirs(subdir)
        open(os.path.join(sourcedir, 'file1'), 'w').close()
        open(os.path.join(subdir, 'file2'), 'w').close()

        self.assertEqual(
            os.path.join(handler.code.build_basedir, source_subdir),
            handler.code.builddir)

        handler.build()

        self.assertTrue(
            os.path.exists(os.path.join(handler.code.build_basedir, 'file1')))
        self.assertTrue(
            os.path.exists(os.path.join(handler.code.builddir, 'file2')))

    def test_build_without_subdir_copies_sourcedir(self):
        handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil')

        os.makedirs(handler.sourcedir)
        open(os.path.join(handler.sourcedir, 'file'), 'w').close()

        self.assertEqual(handler.code.build_basedir, handler.code.builddir)

        handler.build()

        self.assertTrue(
            os.path.exists(os.path.join(handler.code.build_basedir, 'file')))

    @patch('os.path.isdir', return_value=False)
    def test_local_non_dir_source_path_must_raise_exception(self, mock_isdir):
        with self.assertRaises(ValueError) as raised:
            pluginhandler.load_plugin(
                part_name='test-part',
                plugin_name='nil',
                part_properties={'source': 'file'},
                part_schema={'source': {'type': 'string'}})

        mock_isdir.assert_called_once_with('file')

        self.assertEqual(raised.exception.__str__(),
                         'local source is not a directory')

    def test_init_unknown_plugin_must_raise_exception(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        with self.assertRaises(pluginhandler.PluginError) as raised:
            pluginhandler.load_plugin(part_name='fake-part',
                                      plugin_name='test_unexisting')

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

        handler = pluginhandler.load_plugin(part_name='test-part',
                                            plugin_name='nil')
        handler.code.options.snap = ['foo']
        handler.code.options.stage = ['bar']
        expected_options = copy.deepcopy(handler.code.options)

        handler.migratable_fileset_for('stage')
        self.assertEqual(vars(expected_options),
                         vars(handler.code.options),
                         'Expected options to be unmodified')

        handler.migratable_fileset_for('prime')
        self.assertEqual(vars(expected_options),
                         vars(handler.code.options),
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

    @patch('os.chown')
    def test_migrate_files_preserves_ownership(self, chown_mock):
        os.makedirs('install')
        os.makedirs('stage')

        foo = os.path.join('install', 'foo')

        with open(foo, 'w') as f:
            f.write('installed')

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=True)

        self.assertTrue(chown_mock.called)

    @patch('os.chown')
    def test_migrate_files_chown_permissions(self, chown_mock):
        os.makedirs('install')
        os.makedirs('stage')

        chown_mock.side_effect = PermissionError("No no no")

        foo = os.path.join('install', 'foo')

        with open(foo, 'w') as f:
            f.write('installed')

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=True)

        self.assertTrue(chown_mock.called)

    def test_migrate_files_preserves_file_mode(self):
        os.makedirs('install')
        os.makedirs('stage')

        foo = os.path.join('install', 'foo')

        with open(foo, 'w') as f:
            f.write('installed')

        mode = os.stat(foo).st_mode

        new_mode = 0o777
        os.chmod(foo, new_mode)
        self.assertNotEqual(mode, new_mode)

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=True)

        self.assertEqual(stat.S_IMODE(
            os.stat(os.path.join('stage', 'foo')).st_mode), new_mode)

    @patch('os.chown')
    def test_migrate_files_preserves_file_mode_chown_permissions(self,
                                                                 chown_mock):
        chown_mock.side_effect = PermissionError("No no no")
        os.makedirs('install')
        os.makedirs('stage')

        foo = os.path.join('install', 'foo')

        with open(foo, 'w') as f:
            f.write('installed')

        mode = os.stat(foo).st_mode

        new_mode = 0o777
        os.chmod(foo, new_mode)
        self.assertNotEqual(mode, new_mode)

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=True)

        self.assertEqual(stat.S_IMODE(
            os.stat(os.path.join('stage', 'foo')).st_mode), new_mode)

        self.assertTrue(chown_mock.called)

    def test_migrate_files_preserves_directory_mode(self):
        os.makedirs('install/foo')
        os.makedirs('stage')

        foo = os.path.join('install', 'foo', 'bar')

        with open(foo, 'w') as f:
            f.write('installed')

        mode = os.stat(foo).st_mode

        new_mode = 0o777
        self.assertNotEqual(mode, new_mode)
        os.chmod(os.path.dirname(foo), new_mode)
        os.chmod(foo, new_mode)

        files, dirs = pluginhandler._migratable_filesets(['*'], 'install')
        pluginhandler._migrate_files(
            files, dirs, 'install', 'stage', follow_symlinks=True)

        self.assertEqual(stat.S_IMODE(
            os.stat(os.path.join('stage', 'foo')).st_mode), new_mode)
        self.assertEqual(stat.S_IMODE(
            os.stat(os.path.join('stage', 'foo', 'bar')).st_mode), new_mode)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_non_local_plugins(self, plugin_mock,
                               local_load_mock, import_mock):
        mock_plugin = Mock()
        mock_plugin.schema.return_value = {}
        mock_plugin.get_pull_properties.return_value = []
        mock_plugin.get_build_properties.return_value = []
        plugin_mock.return_value = mock_plugin
        local_load_mock.side_effect = ImportError()
        pluginhandler.PluginHandler(
            plugin_name='mock',
            part_name='mock-part',
            part_properties={},
            project_options=snapcraft.ProjectOptions(),
            part_schema={'properties': {}})
        import_mock.assert_called_with('snapcraft.plugins.mock')
        local_load_mock.assert_called_with('x-mock', self.local_plugins_dir)

    def test_plugin_without_project(self):
        class OldPlugin(snapcraft.BasePlugin):

            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['fake-property'] = {
                    'type': 'string',
                }
                return schema

            def __init__(self, name, options):
                super().__init__(name, options)

        self.useFixture(fixture_setup.FakePlugin('oldplugin', OldPlugin))
        plugin = pluginhandler.PluginHandler(
            plugin_name='oldplugin',
            part_name='fake-part',
            part_properties={'fake-property': '.'},
            project_options=snapcraft.ProjectOptions(),
            part_schema=OldPlugin.schema()['properties'])

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

            @classmethod
            def get_pull_properties(cls):
                return []

            @classmethod
            def get_build_properties(cls):
                return []

            def __init__(self, name, options):
                pass

        plugin_mock.return_value = NonBaseOldPlugin
        local_load_mock.side_effect = ImportError()
        plugin = pluginhandler.PluginHandler(
            plugin_name='nonbaseoldplugin',
            part_name='fake-part',
            part_properties={'source': '.'},
            project_options=snapcraft.ProjectOptions(),
            part_schema={'properties': {}})

        self.assertTrue(plugin.code.project is not None)

    def test_plugin_schema_step_hint_pull(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['pull-properties'] = ['foo']

                return schema

        self.useFixture(fixture_setup.FakePlugin('plugin', Plugin))
        pluginhandler.PluginHandler(
            plugin_name='plugin',
            part_name='fake-part',
            part_properties={'source': '.'},
            project_options=snapcraft.ProjectOptions(),
            part_schema={'properties': {}})

    def test_plugin_schema_step_hint_build(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['build-properties'] = ['foo']

                return schema

        self.useFixture(fixture_setup.FakePlugin('plugin', Plugin))
        pluginhandler.PluginHandler(
            plugin_name='plugin',
            part_name='fake-part',
            part_properties={'source': '.'},
            project_options=snapcraft.ProjectOptions(),
            part_schema={'properties': {}})

    def test_plugin_schema_step_hint_pull_and_build(self):
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

        self.useFixture(fixture_setup.FakePlugin('plugin', Plugin))
        pluginhandler.PluginHandler(
            plugin_name='plugin',
            part_name='fake-part',
            part_properties={'source': '.'},
            project_options=snapcraft.ProjectOptions(),
            part_schema={'properties': {}})

    def test_plugin_schema_invalid_pull_hint(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['pull-properties'] = ['bar']

                return schema

        self.useFixture(fixture_setup.FakePlugin('plugin', Plugin))
        with self.assertRaises(ValueError) as raised:
            pluginhandler.PluginHandler(
                plugin_name='plugin',
                part_name='fake-part',
                part_properties={'source': '.'},
                project_options=snapcraft.ProjectOptions(),
                part_schema={'properties': {}})

        self.assertEqual(
            "Invalid pull properties specified by 'plugin' plugin: ['bar']",
            str(raised.exception))

    def test_plugin_schema_invalid_build_hint(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema['properties']['foo'] = {
                    'type': 'string',
                }
                schema['build-properties'] = ['bar']

                return schema

        self.useFixture(fixture_setup.FakePlugin('plugin', Plugin))
        with self.assertRaises(ValueError) as raised:
            pluginhandler.PluginHandler(
                plugin_name='plugin',
                part_name='fake-part',
                part_properties={'source': '.'},
                project_options=snapcraft.ProjectOptions(),
                part_schema={'properties': {}})

        self.assertEqual(
            "Invalid build properties specified by 'plugin' plugin: ['bar']",
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


class OrganizeTestCase(tests.TestCase):

    scenarios = [
        ('simple_file', dict(
            setup_dirs=[],
            setup_files=['foo'],
            organize_set={'foo': 'bar'},
            expected=[(['bar'], '')]
        )),
        ('simple_dir_with_file', dict(
            setup_dirs=['foodir'],
            setup_files=[os.path.join('foodir', 'foo')],
            organize_set={'foodir': 'bardir'},
            expected=[
                (['bardir'], ''),
                (['foo'], 'bardir'),
            ]
        )),
        ('organize_to_the_same_directory', dict(
            setup_dirs=['bardir', 'foodir'],
            setup_files=[
                os.path.join('foodir', 'foo'),
                os.path.join('bardir', 'bar'),
                'basefoo',
            ],
            organize_set={
                'foodir': 'bin',
                'bardir': 'bin',
                'basefoo': 'bin/basefoo'
            },
            expected=[
                (['bin'], ''),
                (['bar', 'basefoo', 'foo'], 'bin'),
            ]
        )),
        ('overwrite_existing_file', dict(
            setup_dirs=[],
            setup_files=['foo', 'bar'],
            organize_set={'foo': 'bar'},
            expected=EnvironmentError,
        )),
        ('*_for_files', dict(
            setup_dirs=[],
            setup_files=['foo.conf', 'bar.conf'],
            organize_set={'*.conf': 'dir/'},
            expected=[
                (['dir'], ''),
                (['bar.conf', 'foo.conf'], 'dir'),
            ]
        )),
        ('*_for_files_with_non_dir_dst', dict(
            setup_dirs=[],
            setup_files=['foo.conf', 'bar.conf'],
            organize_set={'*.conf': 'dir'},
            expected=EnvironmentError,
        )),
        ('*_for_directories', dict(
            setup_dirs=['dir1', 'dir2'],
            setup_files=[
                os.path.join('dir1', 'foo'),
                os.path.join('dir2', 'bar'),
            ],
            organize_set={'dir*': 'dir/'},
            expected=[
                (['dir'], ''),
                (['dir1', 'dir2'], 'dir'),
                (['foo'], os.path.join('dir', 'dir1')),
                (['bar'], os.path.join('dir', 'dir2')),
            ]
        )),
        ('combined_*_with_file', dict(
            setup_dirs=['dir1', 'dir2'],
            setup_files=[
                os.path.join('dir1', 'foo'),
                os.path.join('dir1', 'bar'),
                os.path.join('dir2', 'bar'),
            ],
            organize_set={
                'dir*': 'dir/',
                'dir1/bar': '.'
            },
            expected=[
                (['bar', 'dir'], ''),
                (['dir1', 'dir2'], 'dir'),
                (['foo'], os.path.join('dir', 'dir1')),
                (['bar'], os.path.join('dir', 'dir2')),
            ]
        )),
    ]

    def test_organize_file(self):
        base_dir = 'install'
        os.makedirs(base_dir)

        for directory in self.setup_dirs:
            os.makedirs(os.path.join(base_dir, directory))

        for file_entry in self.setup_files:
            with open(os.path.join(base_dir, file_entry), 'w') as f:
                f.write(file_entry)

        if (isinstance(self.expected, type) and
                issubclass(self.expected, Exception)):
            with self.assertRaises(self.expected):
                pluginhandler._organize_filesets(self.organize_set, base_dir)
        else:
            pluginhandler._organize_filesets(self.organize_set, base_dir)
            for expect in self.expected:
                dir_path = os.path.join(base_dir, expect[1])
                dir_contents = os.listdir(dir_path)
                dir_contents.sort()
                self.assertEqual(expect[0], dir_contents)


class RealStageTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.make_snapcraft_yaml("""name: pc-file-test
version: 1.0
summary: test pkg-config .pc
description: when the .pc files reach stage the should be reprefixed
confinement: strict
grade: stable

parts:
    stage-pc:
        plugin: nil
""")

    def test_pc_files_correctly_prefixed(self):
        pc_file = os.path.join('usr', 'lib', 'pkgconfig', 'granite.pc')
        stage_pc_install = os.path.join(
            'parts', 'stage-pc', 'install', pc_file)
        stage_pc_stage = os.path.join('stage', pc_file)

        # Run build
        lifecycle.execute('build', snapcraft.ProjectOptions())

        # Simulate a .pc file was installed
        os.makedirs(os.path.dirname(stage_pc_install))
        with open(stage_pc_install, 'w') as f:
            f.write('prefix=/usr\n')
            f.write('exec_prefix=${prefix}\n')
            f.write('libdir=${prefix}/lib\n')
            f.write('includedir=${prefix}/include\n')
            f.write('\n')
            f.write('Name: granite\n')
            f.write('Description: elementary\'s Application Framework\n')
            f.write('Version: 0.4\n')
            f.write('Libs: -L${libdir} -lgranite\n')
            f.write('Cflags: -I${includedir}/granite\n')
            f.write('Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 '
                    'gobject-2.0\n')

        # Now we stage
        lifecycle.execute('stage', snapcraft.ProjectOptions())

        with open(stage_pc_stage) as f:
            pc_file_content = f.read()
        expected_pc_file_content = """prefix={}/stage/usr
exec_prefix=${{prefix}}
libdir=${{prefix}}/lib
includedir=${{prefix}}/include

Name: granite
Description: elementary's Application Framework
Version: 0.4
Libs: -L${{libdir}} -lgranite
Cflags: -I${{includedir}}/granite
Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
""".format(os.getcwd())

        self.assertEqual(pc_file_content, expected_pc_file_content)

    def test_pc_files_correctly_prefixed_when_installed(self):
        pc_file = os.path.join('usr', 'lib', 'pkgconfig', 'granite.pc')
        install_path = os.path.join(
            os.getcwd(), 'parts', 'stage-pc', 'install')
        stage_pc_install = os.path.join(install_path, pc_file)
        stage_pc_stage = os.path.join('stage', pc_file)

        # Run build
        lifecycle.execute('build', snapcraft.ProjectOptions())

        # Simulate a .pc file was installed
        os.makedirs(os.path.dirname(stage_pc_install))
        with open(stage_pc_install, 'w') as f:
            f.write('prefix={}/usr\n'.format(install_path))
            f.write('exec_prefix=${prefix}\n')
            f.write('libdir=${prefix}/lib\n')
            f.write('includedir=${prefix}/include\n')
            f.write('\n')
            f.write('Name: granite\n')
            f.write('Description: elementary\'s Application Framework\n')
            f.write('Version: 0.4\n')
            f.write('Libs: -L${libdir} -lgranite\n')
            f.write('Cflags: -I${includedir}/granite\n')
            f.write('Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 '
                    'gobject-2.0\n')

        # Now we stage
        lifecycle.execute('stage', snapcraft.ProjectOptions())

        with open(stage_pc_stage) as f:
            pc_file_content = f.read()
        expected_pc_file_content = """prefix={}/stage/usr
exec_prefix=${{prefix}}
libdir=${{prefix}}/lib
includedir=${{prefix}}/include

Name: granite
Description: elementary's Application Framework
Version: 0.4
Libs: -L${{libdir}} -lgranite
Cflags: -I${{includedir}}/granite
Requires: cairo gee-0.8 glib-2.0 gio-unix-2.0 gobject-2.0
""".format(os.getcwd())

        self.assertEqual(pc_file_content, expected_pc_file_content)


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
            os.path.join(self.path, 'prime')
        ]

    def test_makedirs_with_existing_dirs(self):
        part_name = 'test_part'
        dirs = self.get_plugin_dirs(part_name)
        if self.make_dirs:
            os.makedirs(os.path.join('parts', part_name))
            for d in dirs:
                os.mkdir(d)

        p = pluginhandler.load_plugin(part_name=part_name, plugin_name='nil')
        p.makedirs()
        for d in dirs:
            self.assertTrue(os.path.exists(d), '{} does not exist'.format(d))


class StateTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = patch('snapcraft._baseplugin.BasePlugin.get_pull_properties')
        self.get_pull_properties_mock = patcher.start()
        self.get_pull_properties_mock.return_value = []
        self.addCleanup(patcher.stop)

        patcher = patch(
            'snapcraft._baseplugin.BasePlugin.get_build_properties')
        self.get_build_properties_mock = patcher.start()
        self.get_build_properties_mock.return_value = []
        self.addCleanup(patcher.stop)

        part_name = 'test_part'
        self.handler = pluginhandler.load_plugin(part_name=part_name,
                                                 plugin_name='nil')
        self.handler.makedirs()

    def test_mark_done_clears_later_steps(self):
        for index, step in enumerate(common.COMMAND_ORDER):
            shutil.rmtree(self.parts_dir)
            with self.subTest('{} step'.format(step)):
                handler = pluginhandler.load_plugin(part_name='foo',
                                                    plugin_name='nil')
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

                handler = pluginhandler.load_plugin(part_name=part_name,
                                                    plugin_name='nil')
                self.assertEqual(step, handler.last_step())

    @patch('snapcraft.internal.repo.Ubuntu')
    def test_pull_state(self, ubuntu_mock):
        self.assertEqual(None, self.handler.last_step())

        self.handler.pull()

        self.assertEqual('pull', self.handler.last_step())
        state = self.handler.get_state('pull')

        self.assertTrue(state, 'Expected pull to save state YAML')
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(7, len(state.properties))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue('deb_arch' in state.project_options)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_pull_state_with_properties(self, plugin_mock, local_load_mock,
                                        import_mock):
        self.get_pull_properties_mock.return_value = ['foo']
        self.handler.code.options.foo = 'bar'
        self.handler._part_properties = {'foo': 'bar'}

        self.assertEqual(None, self.handler.last_step())

        self.handler.pull()

        self.assertEqual('pull', self.handler.last_step())
        state = self.handler.get_state('pull')

        self.assertTrue(state, 'Expected pull to save state YAML')
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertTrue('foo' in state.properties)
        self.assertEqual(state.properties['foo'], 'bar')
        self.assertTrue(type(state.project_options) is OrderedDict)
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
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(0, len(state.properties))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue('deb_arch' in state.project_options)

    @patch('importlib.import_module')
    @patch('snapcraft.internal.pluginhandler._load_local')
    @patch('snapcraft.internal.pluginhandler._get_plugin')
    def test_build_state_with_properties(self, plugin_mock, local_load_mock,
                                         import_mock):
        self.get_build_properties_mock.return_value = ['foo']
        self.handler.code.options.foo = 'bar'
        self.handler._part_properties = {'foo': 'bar'}

        self.assertEqual(None, self.handler.last_step())

        self.handler.build()

        self.assertEqual('build', self.handler.last_step())
        state = self.handler.get_state('build')

        self.assertTrue(state, 'Expected build to save state YAML')
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertTrue('foo' in state.properties)
        self.assertEqual(state.properties['foo'], 'bar')
        self.assertTrue(type(state.project_options) is OrderedDict)
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
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/2' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertTrue('stage' in state.properties)
        self.assertEqual(state.properties['stage'], ['*'])
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertEqual(0, len(state.project_options))

    def test_stage_state_with_stage_keyword(self):
        self.handler.code.options.stage = ['bin/1']
        self.handler._part_properties = {'stage': ['bin/1']}

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
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(1, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertTrue('stage' in state.properties)
        self.assertEqual(state.properties['stage'], ['bin/1'])
        self.assertTrue(type(state.project_options) is OrderedDict)
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
    def test_prime_state(self, mock_copy, mock_find_dependencies):
        mock_find_dependencies.return_value = set()

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.prime()

        self.assertEqual('prime', self.handler.last_step())
        mock_find_dependencies.assert_called_once_with(self.handler.snapdir,
                                                       {'bin/1', 'bin/2'})
        self.assertFalse(mock_copy.called)

        state = self.handler.get_state('prime')

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/2' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertEqual(0, len(state.dependency_paths))
        self.assertTrue('snap' in state.properties)
        self.assertEqual(state.properties['snap'], ['*'])
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertEqual(0, len(state.project_options))

    @patch('snapcraft.internal.pluginhandler._find_dependencies')
    @patch('shutil.copy')
    def test_prime_state_with_stuff_already_primed(self, mock_copy,
                                                   mock_find_dependencies):
        mock_find_dependencies.return_value = set()

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        bindir = os.path.join(self.handler.snapdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.prime()

        self.assertEqual('prime', self.handler.last_step())
        # bin/2 shouldn't be in this list as it was already primed by another
        # part.
        mock_find_dependencies.assert_called_once_with(self.handler.snapdir,
                                                       {'bin/1'})
        self.assertFalse(mock_copy.called)

        state = self.handler.get_state('prime')

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(1, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertEqual(0, len(state.dependency_paths))
        self.assertTrue('snap' in state.properties)
        self.assertEqual(state.properties['snap'], ['*'])
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertEqual(0, len(state.project_options))

    @patch('snapcraft.internal.pluginhandler._find_dependencies')
    @patch('snapcraft.internal.pluginhandler._migrate_files')
    def test_prime_state_with_dependencies(self, mock_migrate_files,
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
        self.handler.prime()

        self.assertEqual('prime', self.handler.last_step())
        mock_find_dependencies.assert_called_once_with(
            self.handler.snapdir, {'bin/1', 'bin/2'})
        mock_migrate_files.assert_has_calls([
            call({'bin/1', 'bin/2'}, {'bin'}, self.handler.stagedir,
                 self.handler.snapdir),
            call({'foo/bar/baz'}, {'foo/bar'}, '/', self.handler.snapdir,
                 follow_symlinks=True),
        ])

        state = self.handler.get_state('prime')

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
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
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertEqual(0, len(state.project_options))

    @patch('snapcraft.internal.pluginhandler._find_dependencies')
    @patch('shutil.copy')
    def test_prime_state_with_snap_keyword(self, mock_copy,
                                           mock_find_dependencies):
        mock_find_dependencies.return_value = set()
        self.handler.code.options.snap = ['bin/1']
        self.handler._part_properties = {'snap': ['bin/1']}

        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.prime()

        self.assertEqual('prime', self.handler.last_step())
        mock_find_dependencies.assert_called_once_with(self.handler.snapdir,
                                                       {'bin/1'})
        self.assertFalse(mock_copy.called)

        state = self.handler.get_state('prime')

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertEqual(1, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)
        self.assertEqual(0, len(state.dependency_paths))
        self.assertTrue('snap' in state.properties)
        self.assertEqual(state.properties['snap'], ['bin/1'])
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertEqual(0, len(state.project_options))

    def test_clean_prime_state(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.snap_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'prime', states.PrimeState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_prime({})

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(bindir))

    def test_clean_prime_state_multiple_parts(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.snap_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()
        open(os.path.join(bindir, '3'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'prime', states.PrimeState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_prime({})

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertFalse(os.path.exists(os.path.join(bindir, '2')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '3')),
            "Expected 'bin/3' to remain as it wasn't primed by this part")

    def test_clean_prime_state_common_files(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(self.snap_dir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'prime', states.PrimeState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_prime({
            'other_part': states.PrimeState({'bin/2'}, {'bin'})
        })

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '2')),
            "Expected 'bin/2' to remain as it's required by other parts")

    def test_clean_prime_old_state(self):
        self.handler.mark_done('prime', None)
        with self.assertRaises(pluginhandler.MissingState) as raised:
            self.handler.clean_prime({})

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'prime': Missing necessary state. "
            "This won't work until a complete clean has occurred.")


class IsDirtyTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            project_options=snapcraft.ProjectOptions())
        self.handler.makedirs()

    def test_prime_is_dirty(self):
        self.handler.code.options.snap = ['foo']
        self.handler._part_properties = {'snap': ['foo']}
        self.handler.mark_done(
            'prime', states.PrimeState(
                set(), set(), set(), self.handler._part_properties))
        self.assertFalse(self.handler.is_clean('prime'),
                         'Prime step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('prime'),
                         'Strip step was unexpectedly dirty')

        # Change the `snap` keyword-- thereby making the prime step dirty.
        self.handler.code.options.snap = ['bar']
        self.handler._part_properties = {'snap': ['bar']}
        self.assertFalse(self.handler.is_clean('prime'),
                         'Strip step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('prime'),
                        'Expected prime step to be dirty')

    def test_prime_not_dirty_if_clean(self):
        self.assertTrue(self.handler.is_clean('prime'),
                        'Expected vanilla handler to have clean prime step')
        self.assertFalse(
            self.handler.is_dirty('prime'),
            'Expected vanilla handler to not have a dirty prime step')

    def test_stage_is_dirty(self):
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            part_properties={'stage': ['foo']},
            part_schema={'stage': {'type': 'array'}})

        self.handler.mark_stage_done(set(), set())
        self.assertFalse(self.handler.is_clean('stage'),
                         'Stage step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('stage'),
                         'Stage step was unexpectedly dirty')

        # Change the `stage` keyword-- thereby making the stage step dirty.
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            part_properties={'stage': ['bar']},
            part_schema={'stage': {'type': 'array'}})
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
        self.useFixture(fixture_setup.FakePlugin('test-plugin', TestPlugin))
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='test-plugin',
            part_properties={'test-property': 'foo'})
        self.handler.mark_build_done()
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('build'),
                         'Build step was unexpectedly dirty')

        # Change `test-property`, thereby making the build step dirty.
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='test-plugin',
            part_properties={'test-property': 'bar'})
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('build'),
                        'Expected build step to be dirty')

    @patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    def test_build_is_dirty_from_project(self, mock_enable_cross_compilation):
        project_options = snapcraft.ProjectOptions(target_deb_arch='amd64')
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            project_options=project_options)
        self.handler.mark_build_done()
        self.assertFalse(self.handler.is_clean('build'),
                         'Build step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('build'),
                         'Build step was unexpectedly dirty')

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        project_options = snapcraft.ProjectOptions(target_deb_arch='armhf')
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            project_options=project_options)
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
        self.useFixture(fixture_setup.FakePlugin('test-plugin', TestPlugin))
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='test-plugin',
            part_properties={'test-property': 'foo'})
        self.handler.mark_pull_done()
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('pull'),
                         'Pull step was unexpectedly dirty')

        # Change `test-property`, thereby making the pull step dirty.
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='test-plugin',
            part_properties={'test-property': 'bar'})
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertTrue(self.handler.is_dirty('pull'),
                        'Expected pull step to be dirty')

    @patch.object(snapcraft.BasePlugin, 'enable_cross_compilation')
    def test_pull_is_dirty_from_project(self, mock_enable_cross_compilation):
        project_options = snapcraft.ProjectOptions(target_deb_arch='amd64')
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            project_options=project_options)
        self.handler.mark_pull_done()
        self.assertFalse(self.handler.is_clean('pull'),
                         'Pull step was unexpectedly clean')
        self.assertFalse(self.handler.is_dirty('pull'),
                         'Pull step was unexpectedly dirty')

        # Reload the plugin with new project options arch, thereby making it
        # dirty.
        project_options = snapcraft.ProjectOptions(target_deb_arch='armhf')
        self.handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            project_options=project_options)
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
        p = pluginhandler.load_plugin(part_name=part_name, plugin_name='nil')
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
        p = pluginhandler.load_plugin(part_name=part_name, plugin_name='nil')
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
        p = pluginhandler.load_plugin(part_name=part_name, plugin_name='nil')
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

    def test_clean_prime(self):
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
                    part_name='test_part',
                    plugin_name='nil',
                    part_properties=properties,
                    project_options=snapcraft.ProjectOptions(),
                    part_schema=schema)
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

                # Now prime them
                handler.prime()

                self.assertTrue(os.listdir(self.snap_dir))

                handler.clean_prime({})

                self.assertFalse(os.listdir(self.snap_dir),
                                 'Expected snapdir to be completely cleaned')

    def test_clean_prime_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = pluginhandler.load_plugin(part_name='part1',
                                             plugin_name='nil')
        handler1.makedirs()

        bindir = os.path.join(handler1.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()

        handler1.mark_done('build')

        # Now create part2 and get it through the "build" step.
        handler2 = pluginhandler.load_plugin(part_name='part2',
                                             plugin_name='nil')
        handler2.makedirs()

        bindir = os.path.join(handler2.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '2'), 'w').close()

        handler2.mark_done('build')

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # And prime both parts
        handler1.prime()
        handler2.prime()

        # Verify that part1's file has been primeped
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')))

        # Verify that part2's file has been primeped
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')))

        # Now clean the prime step for part1
        handler1.clean_prime({})

        # Verify that part1's file is no longer primeped
        self.assertFalse(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')),
            "Expected part1's primeped files to be cleaned")

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')),
            "Expected part2's primeped files to be untouched")

    def test_clean_prime_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = pluginhandler.load_plugin(
            part_name='part1', plugin_name='nil')
        handler.makedirs()

        bindir = os.path.join(handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        handler.mark_done('build')
        handler.stage()
        handler.prime()

        # Verify that both files have been primeped
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')))
        self.assertTrue(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')))

        # Now update the `snap` fileset to only snap one of these files
        handler.code.options.snap = ['bin/1']

        # Now clean the prime step for part1
        handler.clean_prime({})

        # Verify that part1's file is no longer primeped
        self.assertFalse(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '1')),
            'Expected bin/1 to be cleaned')
        self.assertFalse(
            os.path.exists(os.path.join(self.snap_dir, 'bin', '2')),
            'Expected bin/2 to be cleaned as well, even though the filesets '
            'changed since it was primeped.')

    def test_clean_old_prime_state(self):
        handler = pluginhandler.load_plugin(
            part_name='part1', plugin_name='nil')
        handler.makedirs()

        open(os.path.join(self.snap_dir, '1'), 'w').close()

        handler.mark_done('prime', None)

        self.assertTrue(os.path.exists(handler.code.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.code.partdir))

    def test_clean_prime_old_prime_state(self):
        handler = pluginhandler.load_plugin(
            part_name='part1', plugin_name='nil')
        handler.makedirs()

        primed_file = os.path.join(self.snap_dir, '1')
        open(primed_file, 'w').close()

        handler.mark_done('prime', None)

        with self.assertRaises(pluginhandler.MissingState) as raised:
            handler.clean(step='prime')

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'prime': Missing necessary state. "
            "This won't work until a complete clean has occurred.")

        self.assertTrue(os.path.isfile(primed_file))

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
                    part_name='test_part',
                    plugin_name='nil',
                    part_properties=properties,
                    project_options=snapcraft.ProjectOptions(),
                    part_schema=schema)
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
        handler1 = pluginhandler.load_plugin(part_name='part1',
                                             plugin_name='nil')
        handler1.makedirs()

        bindir = os.path.join(handler1.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()

        handler1.mark_done('build')

        # Now create part2 and get it through the "build" step.
        handler2 = pluginhandler.load_plugin(part_name='part2',
                                             plugin_name='nil')
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
        handler = pluginhandler.load_plugin(part_name='part1',
                                            plugin_name='nil')
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

        # Now clean the prime step for part1
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
        handler = pluginhandler.load_plugin(part_name='part1',
                                            plugin_name='nil')
        handler.makedirs()

        open(os.path.join(self.stage_dir, '1'), 'w').close()

        handler.mark_done('stage', None)

        self.assertTrue(os.path.exists(handler.code.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.code.partdir))

    def test_clean_stage_old_stage_state(self):
        handler = pluginhandler.load_plugin(part_name='part1',
                                            plugin_name='nil')
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

        patcher = patch.object(pluginhandler.PluginHandler, 'clean_prime')
        self.manager_mock.attach_mock(patcher.start(), 'clean_prime')
        self.addCleanup(patcher.stop)

        self.handler = pluginhandler.load_plugin(
            part_name='test_part', plugin_name='nil')

    def test_clean_with_hint(self):
        self.handler.clean(step='pull', hint='foo')

        # Verify the step cleaning order
        self.assertEqual(4, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_prime({}, 'foo'),
            call.clean_stage({}, 'foo'),
            call.clean_build('foo'),
            call.clean_pull('foo'),
        ])

    def test_clean_pull_order(self):
        self.handler.clean(step='pull')

        # Verify the step cleaning order
        self.assertEqual(4, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_prime({}, ''),
            call.clean_stage({}, ''),
            call.clean_build(''),
            call.clean_pull(''),
        ])

    def test_clean_build_order(self):
        self.handler.clean(step='build')

        # Verify the step cleaning order
        self.assertEqual(3, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_prime({}, ''),
            call.clean_stage({}, ''),
            call.clean_build(''),
        ])

    def test_clean_stage_order(self):
        self.handler.clean(step='stage')

        # Verify the step cleaning order
        self.assertEqual(2, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_prime({}, ''),
            call.clean_stage({}, ''),
        ])

    def test_clean_prime_order(self):
        self.handler.clean(step='prime')

        # Verify the step cleaning order
        self.assertEqual(1, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_prime({}, ''),
        ])


class CollisionTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        part1 = pluginhandler.load_plugin(part_name='part1', plugin_name='nil')
        part1.code.installdir = tmpdir + '/install1'
        os.makedirs(part1.installdir + '/a')
        open(part1.installdir + '/a/1', mode='w').close()
        with open(part1.installdir + '/file.pc', mode='w') as f:
            f.write('prefix={}\n'.format(part1.installdir))
            f.write('Name: File\n')

        part2 = pluginhandler.load_plugin(part_name='part2', plugin_name='nil')
        part2.code.installdir = tmpdir + '/install2'
        os.makedirs(part2.installdir + '/a')
        with open(part2.installdir + '/1', mode='w') as f:
            f.write('1')
        open(part2.installdir + '/2', mode='w').close()
        with open(part2.installdir + '/a/2', mode='w') as f:
            f.write('a/2')
        with open(part2.installdir + '/file.pc', mode='w') as f:
            f.write('prefix={}\n'.format(part2.installdir))
            f.write('Name: File\n')

        part3 = pluginhandler.load_plugin(part_name='part3', plugin_name='nil')
        part3.code.installdir = tmpdir + '/install3'
        os.makedirs(part3.installdir + '/a')
        os.makedirs(part3.installdir + '/b')
        with open(part3.installdir + '/1', mode='w') as f:
            f.write('2')
        with open(part2.installdir + '/2', mode='w') as f:
            f.write('1')
        open(part3.installdir + '/a/2', mode='w').close()

        part4 = pluginhandler.load_plugin(part_name='part4', plugin_name='nil')
        part4.code.installdir = tmpdir + '/install4'
        os.makedirs(part4.installdir)
        with open(part4.installdir + '/file.pc', mode='w') as f:
            f.write('prefix={}\n'.format(part4.installdir))
            f.write('Name: ConflictFile\n')

        self.part1 = part1
        self.part2 = part2
        self.part3 = part3
        self.part4 = part4

    def test_no_collisions(self):
        """No exception is expected as there are no collisions."""
        pluginhandler.check_for_collisions([self.part1, self.part2])

    def test_collisions_between_two_parts(self):
        with self.assertRaises(SnapcraftPartConflictError) as raised:
            pluginhandler.check_for_collisions(
                [self.part1, self.part2, self.part3])

        self.assertIn(
            "Parts 'part2' and 'part3' have the following file paths in "
            "common which have different contents:\n    1\n    a/2",
            raised.exception.__str__())

    def test_collisions_between_two_parts_pc_files(self):
        with self.assertRaises(SnapcraftPartConflictError) as raised:
            pluginhandler.check_for_collisions(
                [self.part1, self.part4])

        self.assertIn(
            "Parts 'part1' and 'part4' have the following file paths in "
            "common which have different contents:\n    file.pc",
            raised.exception.__str__())


class StagePackagesTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = patch.object(snapcraft.internal.repo.Ubuntu, 'get')
        setup_apt_mock = patcher.start()
        setup_apt_mock.side_effect = repo.PackageNotFoundError('non-existing')
        self.addCleanup(patcher.stop)

    def test_missing_stage_package_displays_nice_error(self):
        part_schema = {
            'stage-packages': {
                'minitems': 1,
                'uniqueItems': True,
                'default': [],
                'type': 'array',
                'items': {'type': 'string'}},
            'plugin': {'description': 'plugin name', 'type': 'string'}
        }

        part = pluginhandler.load_plugin(
            part_name='stage-test',
            plugin_name='nil',
            part_properties={'stage-packages': ['non-existing']},
            part_schema=part_schema)

        with self.assertRaises(RuntimeError) as raised:
            part.prepare_pull()

        self.assertEqual(
            str(raised.exception),
            "Error downloading stage packages for part 'stage-test': "
            "no such package 'non-existing'")


class FindDependenciesTestCase(tests.TestCase):

    @patch('magic.open')
    @patch('snapcraft.internal.libraries.get_dependencies')
    def test_find_dependencies(self, mock_dependencies, mock_magic):
        workdir = os.path.join(os.getcwd(), 'workdir')
        os.makedirs(workdir)

        linked_elf_path = os.path.join(workdir, 'linked')
        open(linked_elf_path, 'w').close()

        linked_elf_path_b = linked_elf_path.encode(sys.getfilesystemencoding())

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'dynamically linked interpreter /lib64/ld-linux-x86-64.so.2, '
            'for GNU/Linux 2.6.32, BuildID[sha1]=XYZ, stripped')

        mock_dependencies.return_value = ['/usr/lib/libDepends.so']

        dependencies = pluginhandler._find_dependencies(workdir, {'linked'})

        mock_ms.file.assert_called_once_with(linked_elf_path_b)
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

        dependencies = pluginhandler._find_dependencies(
            workdir, {'object_file.o'})

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

        statically_linked_elf_path_b = statically_linked_elf_path.encode(
            sys.getfilesystemencoding())

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = (
            'ELF 64-bit LSB executable, x86-64, version 1 (SYSV), '
            'statically linked, for GNU/Linux 2.6.32, '
            'BuildID[sha1]=XYZ, stripped')

        dependencies = pluginhandler._find_dependencies(
            workdir, {'statically-linked'})

        mock_ms.file.assert_called_once_with(statically_linked_elf_path_b)

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

        non_elf_path_b = non_elf_path.encode(sys.getfilesystemencoding())

        mock_ms = Mock()
        mock_magic.return_value = mock_ms
        mock_ms.load.return_value = 0
        mock_ms.file.return_value = 'JPEG image data, Exif standard: ...'

        dependencies = pluginhandler._find_dependencies(workdir, {'non-elf'})

        mock_ms.file.assert_called_once_with(non_elf_path_b)

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

        dependencies = pluginhandler._find_dependencies(workdir, {'symlinked'})

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
            pluginhandler._find_dependencies('.', set())

        self.assertEqual(
            raised.exception.__str__(), 'Cannot load magic header detection')


class SourcesTestCase(tests.TestCase):

    def test_do_not_follow_links(self):
        properties = dict(source='.')
        handler = pluginhandler.load_plugin(part_name='test-part',
                                            plugin_name='nil',
                                            part_properties=properties)

        # Create a file and a symlink to it
        open('file', mode='w').close()
        os.symlink('file', 'symlinkfile')

        # Create a directory and a symlink to it
        os.mkdir('dir')
        os.symlink('dir', 'symlinkdir')

        handler.pull()
        handler.build()

        # Make sure this is still a link
        build_file_path = os.path.join(
            handler.code.builddir, 'file')
        build_symlinkfile_path = os.path.join(
            handler.code.builddir, 'symlinkfile')

        self.assertTrue(os.path.isfile(build_file_path))
        self.assertTrue(os.path.islink(build_symlinkfile_path))

        build_dir_path = os.path.join(
            handler.code.builddir, 'dir')
        build_symlinkdir_path = os.path.join(
            handler.code.builddir, 'symlinkdir')

        self.assertTrue(os.path.isdir(build_dir_path))
        self.assertTrue(os.path.isdir(build_symlinkdir_path))

    def test_pull_ignores_snapcraft_files_in_source_dir(self):
        properties = dict(source='.')
        handler = pluginhandler.load_plugin(part_name='test-part',
                                            plugin_name='nil',
                                            part_properties=properties)

        open('my-snap.snap', 'w').close()
        open('my-snap', 'w').close()

        handler.pull()

        for file_ in common.SNAPCRAFT_FILES:
            self.assertFalse(
                os.path.exists(os.path.join(handler.sourcedir, file_)))
        self.assertFalse(
            os.path.exists(os.path.join(handler.sourcedir, 'my-snap.snap')),
            os.listdir(handler.sourcedir))

        # Make sure we don't filter things out incorrectly
        self.assertTrue(
            os.path.exists(os.path.join(handler.sourcedir, 'my-snap')),
            os.listdir(handler.sourcedir))

    def test_source_with_unrecognized_source_must_raise_exception(self):
        properties = dict(source='unrecognized://test_source')

        with self.assertRaises(ValueError) as raised:
            pluginhandler.load_plugin(part_name='test-part',
                                      plugin_name='nil',
                                      part_properties=properties)

        self.assertEqual(raised.exception.__str__(),
                         'no handler to manage source')


class CleanPullTestCase(tests.TestCase):

    def test_clean_pull_directory(self):
        handler = pluginhandler.load_plugin(part_name='test-part',
                                            plugin_name='nil')

        handler.pull()
        source_file = os.path.join(handler.sourcedir, 'source')
        open(source_file, 'w').close()

        handler.clean_pull()

        # The source directory should now be gone
        self.assertFalse(os.path.exists(handler.sourcedir))

    def test_clean_pull_symlink(self):
        real_source_directory = os.path.join(os.getcwd(), 'src')
        os.mkdir(real_source_directory)

        handler = pluginhandler.load_plugin(
            part_name='test-part',
            plugin_name='nil',
            part_properties={'source': 'src'},
            part_schema={'source': {'type': 'string'}})

        handler.pull()
        os.rmdir(handler.sourcedir)
        os.symlink(real_source_directory, handler.sourcedir)

        handler.clean_pull()

        # The source symlink should now be gone, but the real source should
        # still be there.
        self.assertFalse(os.path.exists(handler.sourcedir))
        self.assertTrue(os.path.isdir(real_source_directory))


class CleanBuildTestCase(tests.TestCase):

    def test_clean_build(self):
        handler = pluginhandler.load_plugin(part_name='test-part',
                                            plugin_name='nil')

        handler.build()

        source_file = os.path.join(handler.sourcedir, 'source')
        open(source_file, 'w').close()
        open(os.path.join(handler.code.build_basedir, 'built'), 'w').close()
        open(os.path.join(handler.code.installdir, 'installed'), 'w').close()

        handler.clean_build()

        # Make sure the source file hasn't been touched
        self.assertTrue(os.path.isfile(source_file))

        # Make sure the build directory is gone
        self.assertFalse(os.path.exists(handler.code.build_basedir))

        # Make sure the install directory is gone
        self.assertFalse(os.path.exists(handler.code.installdir))
