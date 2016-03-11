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
    Mock,
    patch,
)

import fixtures

from snapcraft import (
    common,
    pluginhandler,
    tests,
)


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
        os.makedirs(os.path.join(common.get_partsdir(), part_name))

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

    def test_pull_state(self):
        self.assertEqual(None, self.handler.last_step())

        self.handler.pull()

        self.assertEqual('pull', self.handler.last_step())

    def test_build_state(self):
        self.assertEqual(None, self.handler.last_step())

        self.handler.build()

        self.assertEqual('build', self.handler.last_step())

    def test_stage_state(self):
        self.assertEqual(None, self.handler.last_step())

        self.handler.stage()

        self.assertEqual('stage', self.handler.last_step())

    def test_strip_state(self):
        self.assertEqual(None, self.handler.last_step())

        self.handler.strip()

        self.assertEqual('strip', self.handler.last_step())


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
