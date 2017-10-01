# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import collections
import os
import tempfile
from glob import glob
from unittest import mock

import fixtures
from testtools.matchers import Equals, FileContains, HasLength

import snapcraft
from snapcraft import tests
from snapcraft.tests import fixture_setup
from snapcraft.plugins import python


def setup_directories(plugin, python_version):
    version = '2.7' if python_version == 'python2' else '3.5'
    os.makedirs(plugin.sourcedir)
    os.makedirs(plugin.builddir)
    python_home = os.path.join(plugin.installdir, 'usr')
    python_lib_path = os.path.join(python_home, 'lib', 'python' + version)
    python_include_path = os.path.join(
        python_home, 'include', 'python' + version)

    os.makedirs(os.path.join(python_lib_path, 'dist-packages'))
    os.makedirs(python_include_path)
    open(os.path.join(plugin.sourcedir, 'setup.py'), 'w').close()

    site_path = os.path.join(plugin.installdir, 'lib', 'python' + version,
                             'site-packages')
    os.makedirs(site_path)
    with open(os.path.join(python_lib_path, 'site.py'), 'w') as f:
        f.write('#!/usr/bin/python3\n'
                '# comment\n'
                'ENABLE_USER_SITE = None\n'
                'USER_SITE = None\n'
                'USER_BASE = None\n'
                '# comment\n')


def fake_empty_pip_list(*args, **kwargs):
    if 'list' in args[0]:
        return '{}'
    else:
        return ''


class BasePythonPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            source = '.'
            requirements = ''
            constraints = ''
            python_version = 'python3'
            python_packages = []
            process_dependency_links = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('subprocess.check_call')
        self.mock_call = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('subprocess.check_output')
        self.mock_call_output = patcher.start()
        self.addCleanup(patcher.stop)


class PythonPluginTestCase(BasePythonPluginTestCase):

    def test_schema(self):
        schema = python.PythonPlugin.schema()
        expected_requirements = {'type': 'string'}
        expected_constraints = {'type': 'string'}
        expected_python_packages = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {'type': 'string'},
            'default': [],
        }

        self.assertDictEqual(expected_requirements,
                             schema['properties']['requirements'])
        self.assertDictEqual(expected_constraints,
                             schema['properties']['constraints'])
        self.assertDictEqual(expected_python_packages,
                             schema['properties']['python-packages'])

    def test_get_pull_properties(self):
        expected_pull_properties = [
            'requirements',
            'constraints',
            'python-packages',
            'python-version',
        ]
        resulting_pull_properties = python.PythonPlugin.get_pull_properties()

        self.assertThat(resulting_pull_properties,
                        HasLength(len(expected_pull_properties)))

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_env(self):
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        expected_env = []
        env = plugin.env('/testpath')
        self.assertListEqual(expected_env, env)

        env_missing_path = plugin.env('/testpath')
        self.assertTrue('PYTHONPATH=/testpath' not in env_missing_path)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    @mock.patch.object(os.path, 'exists', return_value=False)
    def test_missing_setup_path(
            self, mock_path_exists, mock_run, mock_run_output):
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)
        plugin.pull()
        self.assertFalse(mock_run.called)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_pull_with_nothing(self, mock_run, mock_run_output):
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.pull()
        mock_run.assert_has_calls([])

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_pull_with_requirements(self, mock_run, mock_run_output):
        self.options.requirements = 'requirements.txt'
        self.options.constraints = 'constraints.txt'
        self.options.python_packages = ['test', 'packages']

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        requirements_path = os.path.join(plugin.sourcedir, 'requirements.txt')
        constraints_path = os.path.join(plugin.sourcedir, 'constraints.txt')

        pip_command = [
            os.path.join(plugin.installdir, 'usr', 'bin', 'python3'),
            '-m', 'pip']

        pip_download = ['download',
                        '--disable-pip-version-check',
                        '--dest', plugin._python_package_dir,
                        '--constraint', constraints_path]

        calls = [
            mock.call(pip_command + pip_download +
                      ['--requirement', requirements_path, '.',
                       'test', 'packages'],
                      cwd=plugin.sourcedir, env=mock.ANY),
        ]
        plugin.pull()
        mock_run.assert_has_calls(calls)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_pull_without_requirements(self, mock_run, mock_run_output):
        self.options.requirements = ''
        self.options.constraints = 'constraints.txt'
        self.options.python_packages = ['test', 'packages']

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        constraints_path = os.path.join(plugin.sourcedir, 'constraints.txt')

        pip_command = [
            os.path.join(plugin.installdir, 'usr', 'bin', 'python3'),
            '-m', 'pip']

        pip_download = ['download',
                        '--disable-pip-version-check',
                        '--dest', plugin._python_package_dir,
                        '--constraint', constraints_path]

        calls = [
            mock.call(pip_command + pip_download + ['.', 'test', 'packages'],
                      cwd=plugin.sourcedir, env=mock.ANY),
        ]
        plugin.pull()
        mock_run.assert_has_calls(calls)

    @mock.patch.object(python.PythonPlugin, 'run')
    def test_clean_pull(self, mock_run):
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)

        # Pretend pip downloaded packages
        os.makedirs(os.path.join(plugin.partdir, 'packages'))
        plugin.clean_pull()
        self.assertFalse(
            os.path.isdir(os.path.join(plugin.partdir, 'packages')))

    @mock.patch.object(python.PythonPlugin, 'run_output')
    @mock.patch.object(python.PythonPlugin, 'run')
    @mock.patch.object(python.snapcraft.BasePlugin, 'build')
    def test_build(self, mock_base_build, mock_run, mock_run_output):
        self.options.requirements = 'requirements.txt'
        self.options.constraints = 'constraints.txt'
        self.options.python_packages = ['test', 'packages']

        class TempDir(tempfile.TemporaryDirectory):

            def __enter__(self):
                project_whl_path = os.path.join(self.name, 'project.whl')
                open(project_whl_path, 'w').close()
                return super().__enter__()

        patcher = mock.patch('tempfile.TemporaryDirectory',
                             new=mock.Mock(wraps=TempDir))
        patcher.start()
        self.addCleanup(patcher.stop)

        mock_run_output.return_value = (
            '[{"name": "yaml", "version": "1.2"},'
            ' {"name": "extras", "version": "1.0"}]')

        self.useFixture(fixture_setup.CleanEnvironment())
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        for file_name in (self.options.requirements, self.options.constraints):
            path = os.path.join(plugin.sourcedir, file_name)
            open(path, 'w').close()

        def build_side_effect():
            open(os.path.join(plugin.builddir, 'setup.py'), 'w').close()
            os.mkdir(os.path.join(plugin.builddir, 'dist'))
            open(os.path.join(
                plugin.builddir, 'dist', 'package.tar'), 'w').close()

        mock_base_build.side_effect = build_side_effect

        requirements_path = os.path.join(plugin.sourcedir, 'requirements.txt')
        constraints_path = os.path.join(plugin.sourcedir, 'constraints.txt')

        pip_command = [
            os.path.join(plugin.installdir, 'usr', 'bin', 'python3'),
            '-m', 'pip']

        pip_wheel = ['wheel',
                     '--disable-pip-version-check', '--no-index',
                     '--find-links', plugin._python_package_dir,
                     '--constraint', constraints_path,
                     '--wheel-dir', mock.ANY]

        pip_install = ['install', '--user', '--no-compile',
                       '--disable-pip-version-check', '--no-index',
                       '--find-links', plugin._python_package_dir,
                       '--constraint', constraints_path]

        calls = [
            mock.call(pip_command + pip_wheel +
                      ['--requirement', requirements_path, '.',
                       'test', 'packages'],
                      cwd=plugin.builddir, env=mock.ANY),
            mock.call(tests.ContainsList(pip_command + pip_install +
                      ['project.whl']), env=mock.ANY),
        ]
        plugin.build()
        mock_run.assert_has_calls(calls)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_pip_with_url(self, mock_run, mock_run_output):
        self.options.requirements = 'https://test.com/requirements.txt'
        self.options.constraints = 'http://test.com/constraints.txt'

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        pip_command = [
            os.path.join(plugin.installdir, 'usr', 'bin', 'python3'),
            '-m', 'pip']

        pip_download = ['download',
                        '--disable-pip-version-check',
                        '--dest', plugin._python_package_dir,
                        '--constraint', 'http://test.com/constraints.txt']

        calls = [
            mock.call(pip_command + pip_download +
                      ['--requirement', 'https://test.com/requirements.txt',
                       '.'],
                      cwd=plugin.sourcedir, env=mock.ANY),
        ]
        plugin.pull()
        mock_run.assert_has_calls(calls)

    def test_fileset_ignores(self):
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        expected_fileset = [
            '-bin/pip*',
            '-bin/easy_install*',
            '-bin/wheel',
            '-**/__pycache__',
            '-**/*.pyc',
            '-lib/python*/site-packages/*/RECORD'
        ]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_build_fixes_python_shebangs(self, run_mock, mock_run_output):
        if self.options.python_version == 'python2':
            py_version_short = 'python2'
        elif self.options.python_version == 'python3':
            py_version_short = 'python3'

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        os.makedirs(os.path.join(plugin.installdir, 'bin'))

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
                'contents': '#!/foo/baz/' + py_version_short,
                'expected': '#!/usr/bin/env ' + py_version_short,
            },
            {
                'path': os.path.join(plugin.installdir, 'foo'),
                'contents': 'foo',
                'expected': 'foo',
            },
            {
                'path': os.path.join(plugin.installdir, 'bar'),
                'contents': 'bar\n#!/usr/bin/python3',
                'expected': 'bar\n#!/usr/bin/python3',
            }
        ]

        for file_info in files:
            with open(file_info['path'], 'w') as f:
                f.write(file_info['contents'])

        plugin.build()

        for file_info in files:
            with open(os.path.join(plugin.installdir,
                                   file_info['path']), 'r') as f:
                self.assertThat(f.read(), Equals(file_info['expected']))

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_process_dependency_links(self, run_mock, mock_run_output):
        self.options.process_dependency_links = True
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)
        plugin.pull()
        self.assertIn('--process-dependency-links', run_mock.call_args[0][0])

    @mock.patch.object(os, 'stat')
    @mock.patch.object(os.path, 'exists', return_value=False)
    def test_replicate_owner_mode_missing_path(self, mock_path_exists,
                                               mock_os_stat):
        python._replicate_owner_mode('/nonexistant_path')
        self.assertFalse(mock_os_stat.called)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    @mock.patch.object(python.snapcraft.BasePlugin, 'build')
    def test_build_creates_correct_sitecustomize(
            self, mock_base_build, mock_run, mock_run_output):
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        expected_sitecustomize = (
            'import site\n'
            'import os\n'
            '\n'
            'snap_dir = os.getenv("SNAP")\n'
            'snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")\n'
            'snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")\n'
            '\n'
            'for d in (snap_dir, snapcraft_stage_dir, '
            'snapcraft_part_install):\n'
            '    if d:\n'
            '        site_dir = os.path.join(d, '
            '"lib/python3.5/site-packages")\n'
            '        site.addsitedir(site_dir)\n'
            '\n'
            'if snap_dir:\n'
            '    site.ENABLE_USER_SITE = False')

        site_path = glob(os.path.join(
            plugin.installdir, 'usr', 'lib', 'python*', 'sitecustomize.py'))[0]
        self.assertThat(site_path, FileContains(expected_sitecustomize))

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_use_staged_python(self, run_mock, run_output_mock):
        self.useFixture(fixture_setup.CleanEnvironment())

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)

        setup_directories(plugin, self.options.python_version)
        # Create the necessary hints to detect a staged python
        staged_python_bin = os.path.join(
            plugin.project.stage_dir, 'usr', 'bin', 'python3')
        os.makedirs(os.path.dirname(staged_python_bin))
        open(staged_python_bin, 'w').close()
        staged_python_include = os.path.join(
            plugin.project.stage_dir, 'usr', 'include', 'python3.7')
        os.makedirs(staged_python_include)
        plugin.pull()

        pip_command = [
            os.path.join(plugin.project.stage_dir, 'usr', 'bin', 'python3'),
            '-m', 'pip', 'download', '--disable-pip-version-check',
            '--dest', os.path.join(plugin.partdir, 'packages'), '.'
        ]
        cwd = plugin.sourcedir
        env = {
            'PYTHONUSERBASE': plugin.installdir,
            'PYTHONHOME': os.path.join(plugin.project.stage_dir, 'usr'),
            'PATH': '{}/usr/bin:$PATH'.format(plugin.installdir),
            'CPPFLAGS': '-I{}'.format(os.path.join(
                plugin.project.stage_dir, 'usr', 'include', 'python3.7'))
        }
        run_mock.assert_called_once_with(pip_command, cwd=cwd, env=env)

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_use_staged_python_extra_cppflags(self, run_mock, run_output_mock):
        self.useFixture(fixture_setup.CleanEnvironment())
        # Add some extra CPPFLAGS into the environment
        self.useFixture(fixtures.EnvironmentVariable(
            'CPPFLAGS', '-I/opt/include'))

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)

        setup_directories(plugin, self.options.python_version)
        # Create the necessary hints to detect a staged python
        staged_python_bin = os.path.join(
            plugin.project.stage_dir, 'usr', 'bin', 'python3')
        os.makedirs(os.path.dirname(staged_python_bin))
        open(staged_python_bin, 'w').close()
        staged_python_include = os.path.join(
            plugin.project.stage_dir, 'usr', 'include', 'python3.7')
        os.makedirs(staged_python_include)

        plugin.pull()

        pip_command = [
            os.path.join(plugin.project.stage_dir, 'usr', 'bin', 'python3'),
            '-m', 'pip', 'download', '--disable-pip-version-check',
            '--dest', os.path.join(plugin.partdir, 'packages'), '.'
        ]
        cwd = plugin.sourcedir
        env = {
            'PYTHONUSERBASE': plugin.installdir,
            'PYTHONHOME': os.path.join(plugin.project.stage_dir, 'usr'),
            'PATH': '{}/usr/bin:$PATH'.format(plugin.installdir),
            'CPPFLAGS': '-I{} -I/opt/include'.format(os.path.join(
                plugin.project.stage_dir, 'usr', 'include', 'python3.7'))
        }
        run_mock.assert_called_once_with(pip_command, cwd=cwd, env=env)

    @mock.patch.object(python.PythonPlugin, 'run_output')
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_get_manifest_with_python_packages(self, _, mock_run_output):
        def run_output_side_effect(*args, **kwargs):
            if 'list' in args[0]:
                return ('[{"name": "testpackage1", "version": "1.0"},'
                        '{"name": "testpackage2", "version": "1.2"}]')
            else:
                return ''
        mock_run_output.side_effect = run_output_side_effect

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)
        plugin.build()
        self.assertThat(
            plugin.get_manifest(), Equals(
                collections.OrderedDict(
                    {'python-packages':
                     ['testpackage1=1.0', 'testpackage2=1.2']})))

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_get_manifest_with_local_requirements(self, _, mock_run_output):
        self.options.requirements = 'requirements.txt'
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)
        requirements_path = os.path.join(plugin.sourcedir, 'requirements.txt')
        with open(requirements_path, 'w') as requirements_file:
            requirements_file.write('testpackage1==1.0\n')
            requirements_file.write('testpackage2==1.2')

        plugin.build()

        self.assertThat(
            plugin.get_manifest()['requirements-contents'],
            Equals('testpackage1==1.0\ntestpackage2==1.2'))

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_get_manifest_with_local_constraints(self, _, mock_run_output):
        self.options.constraints = 'constraints.txt'

        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)
        constraints_path = os.path.join(plugin.sourcedir, 'constraints.txt')
        with open(constraints_path, 'w') as constraints_file:
            constraints_file.write('testpackage1==1.0\n')
            constraints_file.write('testpackage2==1.2')

        plugin.build()

        self.assertThat(
            plugin.get_manifest()['constraints-contents'],
            Equals('testpackage1==1.0\ntestpackage2==1.2'))


class PythonPluginWithURLTestCase(
        BasePythonPluginTestCase, tests.FakeFileHTTPServerBasedTestCase):

    def setUp(self):
        super().setUp()
        self.source = 'http://{}:{}/{}'.format(
            *self.server.server_address, 'testfile.txt')

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_get_manifest_with_requirements_url(self, _, mock_run_output):
        self.options.requirements = self.source
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        self.assertThat(
            plugin.get_manifest()['requirements-contents'],
            Equals('Test fake file'))

    @mock.patch.object(
        python.PythonPlugin, 'run_output', side_effect=fake_empty_pip_list)
    @mock.patch.object(python.PythonPlugin, 'run')
    def test_get_manifest_with_constraints_url(self, _, mock_run_output):
        self.options.constraints = self.source
        plugin = python.PythonPlugin('test-part', self.options,
                                     self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        self.assertThat(
            plugin.get_manifest()['constraints-contents'],
            Equals('Test fake file'))
