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
import shutil
import tempfile
from unittest.mock import (
    call,
    Mock,
    MagicMock,
    patch,
)
import yaml

import fixtures

from snapcraft import (
    common,
    pluginhandler,
    tests,
)
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

        files, dirs = pluginhandler._migratable_filesets('*', 'install')
        pluginhandler._migrate_files(files, dirs, 'install', 'stage')

        # Verify that the staged file is the one that was staged last
        with open('stage/foo', 'r') as f:
            self.assertEqual(f.read(), 'installed',
                             'Expected staging to allow overwriting of '
                             'already-staged files')

    @patch('snapcraft.pluginhandler._load_local')
    @patch('snapcraft.pluginhandler._get_plugin')
    def test_schema_not_found(self, plugin_mock, local_load_mock):
        mock_plugin = Mock()
        mock_plugin.schema.return_value = {}
        plugin_mock.return_value = mock_plugin
        local_load_mock.return_value = "not None"

        common.set_schemadir(os.path.join('', 'foo'))

        with self.assertRaises(FileNotFoundError):
            pluginhandler.PluginHandler('mock', 'mock-part', {})

    @patch('importlib.import_module')
    @patch('snapcraft.pluginhandler._load_local')
    @patch('snapcraft.pluginhandler._get_plugin')
    def test_non_local_plugins(self, plugin_mock,
                               local_load_mock, import_mock):
        mock_plugin = Mock()
        mock_plugin.schema.return_value = {}
        plugin_mock.return_value = mock_plugin
        local_load_mock.side_effect = ImportError()
        pluginhandler.PluginHandler('mock', 'mock-part', {})
        import_mock.assert_called_with('snapcraft.plugins.mock')
        local_load_mock.assert_called_with('x-mock')

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
            shutil.rmtree(common.get_partsdir())
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
            shutil.rmtree(common.get_partsdir())
            with self.subTest('{} step'.format(step)):
                part_dir = os.path.join(common.get_partsdir(), part_name)
                os.makedirs(part_dir)
                with open(os.path.join(part_dir, 'state'), 'w') as f:
                    f.write(step)

                handler = pluginhandler.load_plugin(part_name, 'nil')
                self.assertEqual(step, handler.last_step())

    @patch('snapcraft.repo.Ubuntu')
    def test_pull_state(self, ubuntu_mock):
        self.assertEqual(None, self.handler.last_step())

        self.handler.code.stage_packages.append('foo')
        bindir = os.path.join(self.handler.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()

        self.handler.pull()

        self.assertEqual('pull', self.handler.last_step())
        with open(self.handler._step_state_file('pull'), 'r') as f:
            state = yaml.load(f)

        self.assertTrue(state, 'Expected stage to save state YAML')
        self.assertTrue(type(state) is pluginhandler.PullState)
        self.assertTrue(type(state.stage_package_files) is set)
        self.assertTrue(type(state.stage_package_directories) is set)
        self.assertEqual(1, len(state.stage_package_files))
        self.assertTrue('bin/1' in state.stage_package_files)
        self.assertEqual(1, len(state.stage_package_directories))
        self.assertTrue('bin' in state.stage_package_directories)

    def test_build_state(self):
        self.assertEqual(None, self.handler.last_step())

        self.handler.build()

        self.assertEqual('build', self.handler.last_step())

    def test_build_old_pull_state_with_stage_packages(self):
        self.handler.code.stage_packages.append('foo')
        self.handler.mark_done('pull', None)
        with self.assertRaises(pluginhandler.MissingState) as raised:
            self.handler.build()

        self.assertEqual(
            str(raised.exception),
            'Failed to build: Missing necessary pull state. Please run pull '
            'again.')

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
        with open(self.handler._step_state_file('stage'), 'r') as f:
            state = yaml.load(f)

        self.assertTrue(state, 'Expected stage to save state YAML')
        self.assertTrue(type(state) is pluginhandler.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/2' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)

        self.assertEqual('stage', self.handler.last_step())

    def test_clean_stage_state(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(common.get_stagedir(), 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')

        self.handler.mark_done(
            'stage', pluginhandler.StageState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_stage({})

        self.assertEqual('build', self.handler.last_step())
        self.assertFalse(os.path.exists(bindir))

    def test_clean_stage_state_multiple_parts(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(common.get_stagedir(), 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()
        open(os.path.join(bindir, '3'), 'w').close()

        self.handler.mark_done('build')

        self.handler.mark_done(
            'stage', pluginhandler.StageState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_stage({})

        self.assertEqual('build', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertFalse(os.path.exists(os.path.join(bindir, '2')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '3')),
            "Expected 'bin/3' to remain as it wasn't staged by this part")

    def test_clean_stage_state_common_files(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(common.get_stagedir(), 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')

        self.handler.mark_done(
            'stage', pluginhandler.StageState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_stage({
            'other_part': pluginhandler.StageState({'bin/2'}, {'bin'})
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
            "Failed to clean step 'stage': Missing necessary state. Please "
            "run stage again.")

    def test_strip_state(self):
        self.assertEqual(None, self.handler.last_step())

        bindir = os.path.join(self.handler.code.installdir, 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('build')
        self.handler.stage()
        self.handler.strip()

        self.assertEqual('strip', self.handler.last_step())
        with open(self.handler._step_state_file('strip'), 'r') as f:
            state = yaml.load(f)

        self.assertTrue(type(state) is pluginhandler.StripState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertEqual(2, len(state.files))
        self.assertTrue('bin/1' in state.files)
        self.assertTrue('bin/1' in state.files)
        self.assertEqual(1, len(state.directories))
        self.assertTrue('bin' in state.directories)

    def test_clean_strip_state(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(common.get_snapdir(), 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'strip', pluginhandler.StripState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_strip({})

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(bindir))

    def test_clean_strip_state_multiple_parts(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(common.get_snapdir(), 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()
        open(os.path.join(bindir, '3'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'strip', pluginhandler.StripState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_strip({})

        self.assertEqual('stage', self.handler.last_step())
        self.assertFalse(os.path.exists(os.path.join(bindir, '1')))
        self.assertFalse(os.path.exists(os.path.join(bindir, '2')))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, '3')),
            "Expected 'bin/3' to remain as it wasn't stripped by this part")

    def test_clean_strip_state_common_files(self):
        self.assertEqual(None, self.handler.last_step())
        bindir = os.path.join(common.get_snapdir(), 'bin')
        os.makedirs(bindir)
        open(os.path.join(bindir, '1'), 'w').close()
        open(os.path.join(bindir, '2'), 'w').close()

        self.handler.mark_done('stage')

        self.handler.mark_done(
            'strip', pluginhandler.StripState({'bin/1', 'bin/2'}, {'bin'}))

        self.handler.clean_strip({
            'other_part': pluginhandler.StripState({'bin/2'}, {'bin'})
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
            "Failed to clean step 'strip': Missing necessary state. Please "
            "run strip again.")


class CleanTestCase(tests.TestCase):

    @patch('shutil.rmtree')
    @patch('os.path.exists')
    def test_clean_part_that_exists(self, mock_exists, mock_rmtree):
        mock_exists.return_value = True

        part_name = 'test_part'
        p = pluginhandler.load_plugin(part_name, 'nil')
        p.clean()

        partdir = os.path.join(
            os.path.abspath(os.curdir), 'parts', part_name)
        mock_exists.assert_called_once_with(partdir)
        mock_rmtree.assert_called_once_with(partdir)

    @patch('shutil.rmtree')
    @patch('os.path.exists')
    def test_clean_part_already_clean(self, mock_exists, mock_rmtree):
        mock_exists.return_value = False

        part_name = 'test_part'
        p = pluginhandler.load_plugin(part_name, 'nil')
        p.clean()

        partdir = os.path.join(
            os.path.abspath(os.curdir), 'parts', part_name)
        mock_exists.assert_called_once_with(partdir)
        self.assertFalse(mock_rmtree.called)

    def clear_common_directories(self):
        if os.path.exists(common.get_partsdir()):
            shutil.rmtree(common.get_partsdir())

        if os.path.exists(common.get_stagedir()):
            shutil.rmtree(common.get_stagedir())

        if os.path.exists(common.get_snapdir()):
            shutil.rmtree(common.get_snapdir())

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
            'nostara': {
                'fileset': ['-*/a'],
            },
        }

        for key, value in filesets.items():
            with self.subTest(key=key):
                self.clear_common_directories()

                handler = pluginhandler.load_plugin('test_part', 'nil', {
                    'snap': value['fileset']
                })
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

                self.assertTrue(os.listdir(common.get_snapdir()))

                handler.clean_strip({})

                self.assertFalse(os.listdir(common.get_snapdir()),
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
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '1')))

        # Verify that part2's file has been stripped
        self.assertTrue(
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '2')))

        # Now clean the strip step for part1
        handler1.clean_strip({})

        # Verify that part1's file is no longer stripped
        self.assertFalse(
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '1')),
            "Expected part1's stripped files to be cleaned")

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '2')),
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
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '1')))
        self.assertTrue(
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '2')))

        # Now update the `snap` fileset to only snap one of these files
        handler.code.options.snap = ['bin/1']

        # Now clean the strip step for part1
        handler.clean_strip({})

        # Verify that part1's file is no longer stripped
        self.assertFalse(
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '1')),
            'Expected bin/1 to be cleaned')
        self.assertFalse(
            os.path.exists(os.path.join(common.get_snapdir(), 'bin', '2')),
            'Expected bin/2 to be cleaned as well, even though the filesets '
            'changed since it was stripped.')

    def test_clean_old_strip_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        open(os.path.join(common.get_snapdir(), '1'), 'w').close()

        handler.mark_done('strip', None)

        self.assertTrue(os.path.exists(handler.code.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.code.partdir))

    def test_clean_strip_old_strip_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        stripped_file = os.path.join(common.get_snapdir(), '1')
        open(stripped_file, 'w').close()

        handler.mark_done('strip', None)

        with self.assertRaises(pluginhandler.MissingState) as raised:
            handler.clean(step='strip')

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'strip': Missing necessary state. Please "
            "run strip again.")

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
            'nostara': {
                'fileset': ['-*/a'],
            },
        }

        for key, value in filesets.items():
            with self.subTest(key=key):
                self.clear_common_directories()

                handler = pluginhandler.load_plugin('test_part', 'nil', {
                    'stage': value['fileset']
                })
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

                self.assertTrue(os.listdir(common.get_stagedir()))

                handler.clean_stage({})

                self.assertFalse(os.listdir(common.get_stagedir()),
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
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '1')))

        # Verify that part2's file has been staged
        self.assertTrue(
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '2')))

        # Now clean the stage step for part1
        handler1.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '1')),
            "Expected part1's staged files to be cleaned")

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '2')),
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
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '1')))
        self.assertTrue(
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '2')))

        # Now update the `stage` fileset to only snap one of these files
        handler.code.options.stage = ['bin/1']

        # Now clean the strip step for part1
        handler.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '1')),
            'Expected bin/1 to be cleaned')
        self.assertFalse(
            os.path.exists(os.path.join(common.get_stagedir(), 'bin', '2')),
            'Expected bin/2 to be cleaned as well, even though the filesets '
            'changed since it was staged.')

    def test_clean_old_stage_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        open(os.path.join(common.get_stagedir(), '1'), 'w').close()

        handler.mark_done('stage', None)

        self.assertTrue(os.path.exists(handler.code.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.code.partdir))

    def test_clean_stage_old_stage_state(self):
        handler = pluginhandler.load_plugin('part1', 'nil')
        handler.makedirs()

        staged_file = os.path.join(common.get_stagedir(), '1')
        open(staged_file, 'w').close()

        handler.mark_done('stage', None)

        with self.assertRaises(pluginhandler.MissingState) as raised:
            handler.clean(step='stage')

        self.assertEqual(
            str(raised.exception),
            "Failed to clean step 'stage': Missing necessary state. Please "
            "run stage again.")

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

    def test_clean_pull_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='pull')

        # Verify the step cleaning order
        self.assertEqual(4, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}),
            call.clean_stage({}),
            call.clean_build(),
            call.clean_pull(),
        ])

    def test_clean_build_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='build')

        # Verify the step cleaning order
        self.assertEqual(3, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}),
            call.clean_stage({}),
            call.clean_build(),
        ])

    def test_clean_stage_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='stage')

        # Verify the step cleaning order
        self.assertEqual(2, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}),
            call.clean_stage({}),
        ])

    def test_clean_strip_order(self):
        handler = pluginhandler.load_plugin('test_part', 'nil')
        handler.clean(step='strip')

        # Verify the step cleaning order
        self.assertEqual(1, len(self.manager_mock.mock_calls))
        self.manager_mock.assert_has_calls([
            call.clean_strip({}),
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
        stagedir = common.get_stagedir()

        replacements = (
            (
                'no replacement',
                'snapcraft_stage/usr/bin',
                'snapcraft_stage/usr/bin',
            ),
            (
                'replaced start',
                '$SNAPCRAFT_STAGE/usr/bin',
                '{}/usr/bin'.format(stagedir),
            ),
            (
                'replaced between',
                '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                '--with-swig {}/usr/swig'.format(stagedir),
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(pluginhandler._expand_env(subject), expected)

    def test_lists_with_string_replacements(self):
        stagedir = common.get_stagedir()

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
                    '{}/usr/bin'.format(stagedir),
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
                    '--with-swig {}/usr/swig'.format(stagedir),
                ],
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(pluginhandler._expand_env(subject), expected)

    def test_tuples_with_string_replacements(self):
        stagedir = common.get_stagedir()

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
                    '{}/usr/bin'.format(stagedir),
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
                    '--with-swig {}/usr/swig'.format(stagedir),
                ],
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(pluginhandler._expand_env(subject), expected)

    def test_dict_with_string_replacements(self):
        stagedir = common.get_stagedir()

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
                    '1': '{}/usr/bin'.format(stagedir),
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
                    '2': '--with-swig {}/usr/swig'.format(stagedir),
                },
            ),
        )

        for test_name, subject, expected in replacements:
            self.subTest(key=test_name)
            self.assertEqual(pluginhandler._expand_env(subject), expected)

    def test_string_replacement_with_complex_data(self):
        stagedir = common.get_stagedir()

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
                    '{}/file1'.format(stagedir),
                    'SNAPCRAFT_STAGE/really',
                ]
            },
            'configFlags': [
                '--with-python',
                '--with-swig {}/swig'.format(stagedir),
            ],
        }

        self.assertEqual(pluginhandler._expand_env(subject), expected)
