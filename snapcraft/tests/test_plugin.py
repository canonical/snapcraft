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
import tempfile

import fixtures
from unittest.mock import (
    Mock,
    patch,
)

from snapcraft import (
    plugin,
    tests
)
from snapcraft.tests import mock_plugin


def get_test_plugin(name='mock', part_name='mock-part', properties=None):
    if properties is None:
        properties = {}
    return plugin.PluginHandler(name, part_name, properties)


class PluginTestCase(tests.TestCase):

    def test_init_unknown_plugin_must_log_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        get_test_plugin('test_unexisting_name')

        self.assertEqual(
            'Unknown plugin: test_unexisting_name\n', fake_logger.output)

    def test_is_dirty(self):
        p = get_test_plugin()
        p.statefile = tempfile.NamedTemporaryFile().name
        self.addCleanup(os.remove, p.statefile)
        p.code = Mock()
        # pull once
        p.pull()
        p.code.pull.assert_called()
        # pull again, not dirty no need to pull
        p.code.pull.reset_mock()
        p.pull()
        self.assertFalse(p.code.pull.called)

    def test_fileset_include_excludes(self):
        stage_set = [
            '-etc',
            'opt/something',
            '-usr/lib/*.a',
            'usr/bin',
            '\-everything',
            r'\\a',
        ]

        include, exclude = plugin._get_file_list(stage_set)

        self.assertEqual(include, ['opt/something', 'usr/bin',
                                   '-everything', r'\a'])
        self.assertEqual(exclude, ['etc', 'usr/lib/*.a'])

    def test_fileset_only_includes(self):
        stage_set = [
            'opt/something',
            'usr/bin',
        ]

        include, exclude = plugin._get_file_list(stage_set)

        self.assertEqual(include, ['opt/something', 'usr/bin'])
        self.assertEqual(exclude, [])

    def test_fileset_only_excludes(self):
        stage_set = [
            '-etc',
            '-usr/lib/*.a',
        ]

        include, exclude = plugin._get_file_list(stage_set)

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

                snap_files, snap_dirs = plugin.migratable_filesets(
                    filesets[key]['fileset'], srcdir)
                plugin._migrate_files(snap_files, snap_dirs, srcdir, dstdir)

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

    def test_non_local_plugins(self):
        """Ensure regular plugins are loaded from snapcraft only"""
        def mock_import_modules(module_name):
            # called with the full snapcraft path
            self.assertEqual(module_name, "snapcraft.plugins.mock")
            return mock_plugin
        with patch("importlib.import_module", side_effect=mock_import_modules):
            plugin.PluginHandler('mock', 'mock-part', {})

    def test_filesets_includes_without_relative_paths(self):
        with self.assertRaises(plugin.PluginError) as raised:
            plugin._get_file_list(['rel', '/abs/include'])

        self.assertEqual(
            "path '/abs/include' must be relative", str(raised.exception))

    def test_filesets_exlcudes_without_relative_paths(self):
        with self.assertRaises(plugin.PluginError) as raised:
            plugin._get_file_list(['rel', '-/abs/exclude'])

        self.assertEqual(
            "path '/abs/exclude' must be relative", str(raised.exception))

    def test_load_plugin_with_invalid_part_must_exit_with_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        with self.assertRaises(SystemExit) as raised:
            plugin.load_plugin('dummy-part', 'test_unexisting_name')

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            'Unknown plugin: test_unexisting_name\n'
            'Could not load part test_unexisting_name\n',
            fake_logger.output)


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

        p = get_test_plugin(part_name=part_name)
        p.makedirs()
        for d in dirs:
            self.assertTrue(os.path.exists(d), '{} does not exist'.format(d))
