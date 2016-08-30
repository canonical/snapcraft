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

import os

from unittest import mock

import snapcraft
from snapcraft import tests
from snapcraft.plugins import python2


def setup_directories(plugin):
    os.makedirs(plugin.sourcedir)
    os.makedirs(os.path.join(
        plugin.installdir, 'usr', 'lib', 'python2.7', 'dist-packages'))
    os.makedirs(os.path.join(
        plugin.installdir, 'usr', 'include', 'python2.7'))
    open('setup.py', 'w').close()


class Python2PluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            source = '.'
            requirements = ''
            constraints = ''
            python_packages = []
            process_dependency_links = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    @mock.patch.object(python2.Python2Plugin, 'run')
    @mock.patch.object(python2.Python2Plugin, 'run_output',
                       return_value='python2.7')
    def test_pip_relative_site_packages_symlink(self, run_output_mock,
                                                run_mock):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        setup_directories(plugin)
        plugin.pull()

        link = os.readlink(os.path.join(plugin.installdir, 'usr', 'lib',
                                        'python2.7', 'site-packages'))
        self.assertEqual(link, 'dist-packages',
                         'Expected site-packages to be a relative link to '
                         '"dist-packages", but it was a link to "{}"'
                         .format(link))

    def test_schema(self):
        schema = python2.Python2Plugin.schema()
        expected_requirements = {'type': 'string'}
        expected_constraints = {'type': 'string'}
        expected_python_packages = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {'type': 'string'},
            'default': [],
        }
        extend_pull = ['requirements', 'constraints', 'python-packages']

        self.assertDictEqual(expected_requirements,
                             schema['properties']['requirements'])
        self.assertDictEqual(expected_constraints,
                             schema['properties']['constraints'])
        self.assertDictEqual(expected_python_packages,
                             schema['properties']['python-packages'])
        self.assertTrue(
            set(extend_pull).issubset(set(schema['pull-properties']))
        )

    @mock.patch.object(snapcraft.common, 'get_python2_path')
    def test_env(self, mock_path):
        mock_path.side_effect = [
            '/testpath',
            EnvironmentError('test path does not exist'),
        ]

        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        expected_env = [
            'CPPFLAGS="-I/testpath/usr/include $CPPFLAGS"',
            'CFLAGS="-I/testpath/usr/include $CFLAGS"',
            'PYTHONPATH=/testpath',
        ]
        env = plugin.env('/testpath')
        self.assertListEqual(expected_env, env)

        env_missing_path = plugin.env('/testpath')
        self.assertTrue('PYTHONPATH=/testpath' not in env_missing_path)

    @mock.patch.object(python2.Python2Plugin, '_pip')
    def test_pull(self, mock_pip):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        plugin.pull()
        # mock_pip should not be called as there is no setup.py,
        # requirements or python-packages defined.
        self.assertFalse(mock_pip.called)

    @mock.patch.object(python2.Python2Plugin, 'run')
    @mock.patch.object(os.path, 'exists', return_value=False)
    def test_missing_setup_path(self, mock_path_exists, mock_run):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        setup_directories(plugin)
        plugin.pull()
        self.assertFalse(mock_run.called)

    @mock.patch.object(python2.Python2Plugin, 'run')
    def test_setup_pip(self, mock_run):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        easy_install = os.path.join(
            plugin.installdir, 'usr', 'bin', 'easy_install')
        prefix = os.path.join(plugin.installdir, 'usr')

        plugin._setup_pip()
        mock_run.assert_called_with(
            ['python2', easy_install, '--prefix', prefix, 'pip'])

    @mock.patch.object(python2.Python2Plugin, '_setup_pip')
    @mock.patch.object(python2.Python2Plugin, 'run')
    def test_pip(self, mock_run, mock_setup_pip):
        self.options.requirements = 'requirements.txt'
        self.options.constraints = 'constraints.txt'
        self.options.python_packages = ['test', 'packages']

        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        setup_directories(plugin)

        pip2 = os.path.join(plugin.installdir, 'usr', 'bin', 'pip2')
        include = os.path.join(
            plugin.installdir, 'usr', 'include', 'python2.7')
        target = os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2.7', 'site-packages')
        pip_install = ['python2', pip2, 'install',
                       '--global-option=build_ext',
                       '--global-option=-I{}'.format(include),
                       '--target', target]

        requirements_path = os.path.join(plugin.sourcedir, 'requirements.txt')
        constraints_path = os.path.join(plugin.sourcedir, 'constraints.txt')
        pip_install = pip_install + ['--constraint', constraints_path]
        calls = [
            mock.call(pip_install + ['--requirement', requirements_path]),
            mock.call(pip_install + ['--upgrade', 'test', 'packages']),
            mock.call(pip_install + ['.'], cwd=plugin.sourcedir)
        ]
        plugin.pull()
        mock_run.assert_has_calls(calls)

    def test_get_python2_include_missing_raises_exception(self):
        with self.assertRaises(EnvironmentError) as raised:
            python2._get_python2_include('/foo')
        self.assertEqual(str(raised.exception),
                         'python development headers not installed')

    def test_fileset_ignores(self):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        expected_fileset = [
            '-usr/bin/pip*',
            '-**/*.pth',
            '-**/*.pyc',
        ]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)

    @mock.patch.object(python2.Python2Plugin, 'run')
    def test_build_fixes_python_shebangs(self, run_mock):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'setup.py'), 'w').close()
        os.makedirs(os.path.join(plugin.installdir, 'bin'))
        os.makedirs(os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2', 'dist-packages'))
        os.makedirs(os.path.join(
            plugin.installdir, 'usr', 'include', 'python2'))

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                'path': os.path.join(plugin.installdir, 'example.py'),
                'contents': '#!/foo/bar/baz/python',
                'expected': '#!/usr/bin/env python',
            },
            {
                'path': os.path.join(plugin.installdir, 'bin/another_example'),
                'contents': '#!/foo/baz/python2',
                'expected': '#!/usr/bin/env python2',
            },
            {
                'path': os.path.join(plugin.installdir, 'foo'),
                'contents': 'foo',
                'expected': 'foo',
            }
        ]

        for file_info in files:
            with open(file_info['path'], 'w') as f:
                f.write(file_info['contents'])

        plugin.build()

        for file_info in files:
            with open(os.path.join(plugin.installdir,
                                   file_info['path']), 'r') as f:
                self.assertEqual(f.read(), file_info['expected'])

    @mock.patch.object(python2.Python2Plugin, 'run')
    def test_process_dependency_links(self, run_mock):
        self.options.process_dependency_links = True
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        setup_directories(plugin)
        plugin.pull()
        self.assertIn('--process-dependency-links', run_mock.call_args[0][0])
