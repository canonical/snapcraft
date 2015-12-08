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
import os.path
import tempfile
import subprocess

from unittest import mock

from snapcraft import tests
from snapcraft.plugins import catkin


class _IOError(IOError):
    errno = os.errno.EACCES


class CatkinPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class props:
            rosdistro = 'indigo'
            catkin_packages = ['my_package']
            source_subdir = None

        self.properties = props()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.plugins.catkin._Rosdep')
        self.rosdep_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def verify_rosdep_setup(self, plugin):
        self.rosdep_mock.assert_has_calls([
            mock.call(self.properties.rosdistro,
                      os.path.join(plugin.sourcedir, 'src'),
                      os.path.join(plugin.partdir, 'rosdep'),
                      plugin.PLUGIN_STAGE_SOURCES),
            mock.call().setup()])

    def test_pull_debian_dependencies(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # Fake rosdep to setup the following dependecy/resolver tree:
        # my_package:
        #   - catkin = ros-indigo-catkin
        #   - roscpp = ros-indigo-roscpp
        #   - mylib = mylib-dev
        def get(package_name):
            if package_name == 'my_package':
                return ['catkin', 'roscpp', 'mylib']

            return mock.DEFAULT

        def resolve(dependency_name):
            return {
                'catkin': 'ros-indigo-catkin',
                'roscpp': 'ros-indigo-roscpp',
                'mylib': 'mylib-dev',
            }.get(dependency_name)

        self.rosdep_mock.return_value.get_dependencies.side_effect = get
        self.rosdep_mock.return_value.resolve_dependency.side_effect = resolve

        plugin.pull()

        # Verify that rosdep was setup
        self.verify_rosdep_setup(plugin)

        # Verify that rosdep was called to obtain dependencies
        self.rosdep_mock.return_value.get_dependencies.assert_called_with(
            'my_package')

        # Verify that rosdep was called to resolve dependencies into Ubuntu
        # packages.
        self.assertEqual(
            self.rosdep_mock.return_value.resolve_dependency.call_count, 3)
        self.rosdep_mock.return_value.resolve_dependency.assert_has_calls([
            mock.call('catkin'),
            mock.call('roscpp'),
            mock.call('mylib')], any_order=True)

        # Verify that the dependencies were installed
        self.ubuntu_mock.assert_has_calls([
            mock.call().get([
                'ros-indigo-catkin',
                'ros-indigo-roscpp',
                'g++',  # This should be installed as a side effect of roscpp
                'mylib-dev']),
            mock.call().unpack(plugin.installdir)])

    def test_pull_local_dependencies(self):
        self.properties.catkin_packages.append('package_2')

        plugin = catkin.CatkinPlugin('test-part', self.properties)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # Fake rosdep to setup the following dependecy/resolver tree:
        # my_package:
        #   - package_2 = local dependency
        #
        # package_2:
        #   - No dependencies
        def get(package_name):
            if package_name == 'my_package':
                return ['package_2']

            return ''

        self.rosdep_mock.return_value.get_dependencies.side_effect = get

        plugin.pull()

        # Verify that rosdep was setup
        self.verify_rosdep_setup(plugin)

        # Verify that rosdep was called to obtain dependencies
        self.assertEqual(
            self.rosdep_mock.return_value.get_dependencies.call_count, 2)
        self.rosdep_mock.return_value.get_dependencies.assert_has_calls([
            mock.call('my_package'),
            mock.call('package_2')], any_order=True)

        # Verify that rosdep was NOT called to resolve dependencies
        self.rosdep_mock.return_value.resolve_dependency.assert_not_called()

        # Verify that the dependency tree is what we expect
        self.assertEqual(plugin.package_local_deps, {
            'my_package': {'package_2'},
            'package_2': set(),
        })

        # Verify that no .deb packages were installed
        self.assertTrue(mock.call().unpack(plugin.installdir) not in
                        self.ubuntu_mock.mock_calls)

    def test_pull_missing_local_dependency(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # Fake rosdep to setup the following dependecy/resolver tree:
        # my_package:
        #   - package_2 = local dependency
        #                 (which isn't included in the catkin_packages)
        def get(package_name):
            if package_name == 'my_package':
                return ['package_2']

            return ''

        self.rosdep_mock.return_value.get_dependencies.side_effect = get
        self.rosdep_mock.return_value.resolve_dependency.return_value = None

        with self.assertRaises(RuntimeError) as raised:
            plugin.pull()

        self.assertEqual(str(raised.exception),
                         'Package "package_2" isn\'t a valid system '
                         'dependency. Did you forget to add it to '
                         'catkin-packages? If not, add the Ubuntu package '
                         'containing it to stage-packages until you can get '
                         'it into the rosdep database.')

    def test_valid_catkin_workspace_src(self):
        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it contains a 'src' directory, this is a valid Catkin workspace.
        try:
            plugin = catkin.CatkinPlugin('test-part', self.properties)
            os.makedirs(os.path.join(plugin.sourcedir, 'src'))
            plugin.pull()
        except FileNotFoundError:
            self.fail('Unexpectedly raised an exception when the Catkin '
                      'workspace was valid')

    def test_invalid_catkin_workspace_no_src(self):
        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and `source-subdir` wasn't
        # specified, this should fail.
        with self.assertRaises(FileNotFoundError) as raised:
            plugin = catkin.CatkinPlugin('test-part', self.properties)
            plugin.pull()

        self.assertEqual(
            str(raised.exception),
            'Unable to find package path: "{}"'.format(os.path.join(
                plugin.sourcedir, 'src')))

    def test_valid_catkin_workspace_subdir(self):
        self.properties.source_subdir = 'foo'

        # sourcedir is expected to be the root of the Catkin workspace.
        # Normally this would mean it contained a `src` directory, but it can
        # be remapped via the `source-subdir` key.
        try:
            plugin = catkin.CatkinPlugin('test-part', self.properties)
            os.makedirs(os.path.join(plugin.sourcedir,
                        self.properties.source_subdir))
            plugin.pull()
        except FileNotFoundError:
            self.fail('Unexpectedly raised an exception when the Catkin '
                      'src was remapped in a valid manner')

    def test_invalid_catkin_workspace_invalid_subdir(self):
        self.properties.source_subdir = 'foo'

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and source_subdir wasn't
        # specified, this should fail.
        with self.assertRaises(FileNotFoundError) as raised:
            plugin = catkin.CatkinPlugin('test-part', self.properties)
            plugin.pull()

        self.assertEqual(
            str(raised.exception),
            'Unable to find package path: "{}"'.format(os.path.join(
                plugin.sourcedir, self.properties.source_subdir)))

    def test_invalid_catkin_workspace_subdir_same_as_source(self):
        self.properties.source_subdir = '.'

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # source_subdir was specified to be the same as the root, this should
        # fail.
        with self.assertRaises(RuntimeError) as raised:
            plugin = catkin.CatkinPlugin('test-part', self.properties)
            plugin.pull()

        self.assertEqual(str(raised.exception),
                         'source-subdir cannot be the root of the Catkin '
                         'workspace')

    @mock.patch.object(catkin.CatkinPlugin, '_rosrun')
    @mock.patch.object(catkin.CatkinPlugin, 'run')
    def test_build(self, run_mock, rosrun_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))
        os.makedirs(plugin.rosdir)

        # Place _setup_util.py with a bad shebang so we can verify we fixed it
        with open(os.path.join(plugin.rosdir, '_setup_util.py'), 'w') as f:
            f.write('#!/foo/bar/baz/python')

        # Pretend 'my_package' has no dependencies
        def get(package_name):
            return ''

        self.rosdep_mock.return_value.get_dependencies.side_effect = get

        plugin.build()

        # Matching like this for order independence (otherwise it would be
        # quite fragile)
        class check_build_command():
            def __eq__(self, args):
                command = ' '.join(args)
                return (
                    args[0] == 'catkin_make_isolated' and
                    '--install' in command and
                    '--pkg my_package' in command and
                    '--directory {}'.format(plugin.builddir) in command and
                    '--install-space {}'.format(plugin.rosdir) in command)

        rosrun_mock.assert_called_once_with(check_build_command())

        with open('{}/opt/ros/{}/_setup_util.py'.format(
                plugin.installdir, plugin.options.rosdistro), 'r') as f:
            self.assertEqual(f.read(), '#!/usr/bin/env python',
                             'The shebang was not replaced as expected')

    def test_build_with_subdir_without_src_copies_subdir_into_src(self):
        self.properties.catkin_packages = []
        self.properties.source_subdir = 'src_subdir'

        plugin = catkin.CatkinPlugin('test-part', self.properties)

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.sourcedir = tmpdir.name
        subdir = os.path.join(plugin.sourcedir, plugin.options.source_subdir)
        os.mkdir(subdir)
        open(os.path.join(subdir, 'file'), 'w').close()

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.builddir = tmpdir.name

        plugin._provision_builddir()

        self.assertTrue(
            os.path.exists(os.path.join(plugin.builddir, 'src', 'file')))

    def test_build_without_subdir_or_src_copies_sourcedir_into_src(self):
        self.properties.catkin_packages = []

        plugin = catkin.CatkinPlugin('test-part', self.properties)

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.sourcedir = tmpdir.name
        subdir = os.path.join(plugin.sourcedir, 'src_subdir')
        os.mkdir(subdir)
        open(os.path.join(subdir, 'file'), 'w').close()

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.builddir = tmpdir.name

        plugin._provision_builddir()

        self.assertTrue(os.path.exists(
            os.path.join(plugin.builddir, 'src', 'src_subdir', 'file')))


class RosdepTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.rosdep = catkin._Rosdep('ros_distro', 'package_path',
                                     'rosdep_path', 'sources')

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('subprocess.check_output')
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_setup(self):
        # Return something other than a Mock to ease later assertions
        self.check_output_mock.return_value = b''

        self.rosdep.setup()

        # Verify that only rosdep was installed (no other .debs)
        self.assertEqual(self.ubuntu_mock.call_count, 1)
        self.assertEqual(self.ubuntu_mock.return_value.get.call_count, 1)
        self.assertEqual(self.ubuntu_mock.return_value.unpack.call_count, 1)
        self.ubuntu_mock.assert_has_calls([
            mock.call(self.rosdep._rosdep_path, sources='sources'),
            mock.call().get(['python-rosdep']),
            mock.call().unpack(self.rosdep._rosdep_install_path)])

        # Verify that rosdep was initialized and updated
        self.assertEqual(self.check_output_mock.call_count, 2)
        self.check_output_mock.assert_has_calls([
            mock.call(['rosdep', 'init'], env=mock.ANY),
            mock.call(['rosdep', 'update'], env=mock.ANY)
        ])

    def test_setup_initialization_failure(self):
        def run(args, **kwargs):
            if args == ['rosdep', 'init']:
                raise subprocess.CalledProcessError(1, 'foo', 'bar')

            return mock.DEFAULT

        self.check_output_mock.side_effect = run

        with self.assertRaises(RuntimeError) as raised:
            self.rosdep.setup()

        self.assertEqual(str(raised.exception),
                         'Error initializing rosdep database: bar')

    def test_setup_update_failure(self):
        def run(args, **kwargs):
            if args == ['rosdep', 'update']:
                raise subprocess.CalledProcessError(1, 'foo', 'bar')

            return mock.DEFAULT

        self.check_output_mock.side_effect = run

        with self.assertRaises(RuntimeError) as raised:
            self.rosdep.setup()

        self.assertEqual(str(raised.exception),
                         'Error updating rosdep database: bar')

    def test_get_dependencies(self):
        self.check_output_mock.return_value = b'foo\nbar\nbaz'

        self.assertEqual(self.rosdep.get_dependencies('foo'),
                         ['foo', 'bar', 'baz'])

        self.check_output_mock.assert_called_with(['rosdep', 'keys', 'foo'],
                                                  env=mock.ANY)

    def test_get_dependencies_no_dependencies(self):
        self.check_output_mock.return_value = b''

        self.assertEqual(self.rosdep.get_dependencies('foo'), [])

    def test_get_dependencies_invalid_package(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo')

        with self.assertRaises(FileNotFoundError) as raised:
            self.rosdep.get_dependencies('bar')

        self.assertEqual(str(raised.exception),
                         'Unable to find Catkin package "bar"')

    def test_resolve_dependency(self):
        self.check_output_mock.return_value = b'#apt\nmylib-dev'

        self.assertEqual(self.rosdep.resolve_dependency('foo'), 'mylib-dev')

        self.check_output_mock.assert_called_with(
            ['rosdep', 'resolve', 'foo', '--rosdistro', 'ros_distro'],
            env=mock.ANY)

    def test_resolve_invalid_dependency(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo')

        self.assertEqual(self.rosdep.resolve_dependency('bar'), None)

    def test_resolve_dependency_weird_output(self):
        self.check_output_mock.return_value = b'mylib-dev'

        with self.assertRaises(RuntimeError) as raised:
            self.rosdep.resolve_dependency('')

        self.assertEqual(str(raised.exception),
                         'Unexpected rosdep resolve output:\nmylib-dev')

    def test_run(self):
        rosdep = self.rosdep
        rosdep._run(['qux'])

        class check_env():
            def __eq__(self, env):
                rosdep_sources_path = rosdep._rosdep_sources_path
                return (
                    env['PATH'] == os.path.join(rosdep._rosdep_install_path,
                                                'usr', 'bin') and
                    env['PYTHONPATH'] == os.path.join(
                        rosdep._rosdep_install_path, 'usr', 'lib', 'python2.7',
                        'dist-packages') and
                    env['ROSDEP_SOURCE_PATH'] == rosdep_sources_path and
                    env['ROS_HOME'] == rosdep._rosdep_cache_path and
                    env['ROS_PACKAGE_PATH'] == rosdep._ros_package_path)

        self.check_output_mock.assert_called_with(mock.ANY, env=check_env())
