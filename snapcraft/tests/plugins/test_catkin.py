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

import builtins
import os
import os.path
import subprocess
import shutil

from unittest import mock
import testtools
from testtools.matchers import (
    Contains,
    Equals,
    HasLength,
    MatchesRegex,
)

import snapcraft
from snapcraft.plugins import catkin
from snapcraft import (
    repo,
    tests,
)
from snapcraft.internal import errors


class _CompareContainers():
    def __init__(self, test, expected):
        self.test = test
        self.expected = expected

    def __eq__(self, container):
        self.test.assertEqual(len(container), len(self.expected),
                              'Expected {} items to be in container, '
                              'got {}'.format(len(self.expected),
                                              len(container)))

        for expectation in self.expected:
            self.test.assertTrue(expectation in container,
                                 'Expected "{}" to be in container'
                                 .format(expectation))

        return True


class CatkinPluginBaseTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class props:
            rosdistro = 'indigo'
            catkin_packages = ['my_package']
            source_space = 'src'
            source_subdir = None
            include_roscore = False
            underlay = None

        self.properties = props()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            'snapcraft.plugins.catkin._find_system_dependencies')
        self.dependencies_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.plugins.catkin._Rosdep')
        self.rosdep_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.plugins.catkin._Catkin')
        self.catkin_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def verify_rosdep_setup(self, rosdistro, package_path, rosdep_path,
                            sources):
        self.rosdep_mock.assert_has_calls([
            mock.call(rosdistro, package_path, rosdep_path, sources,
                      self.project_options),
            mock.call().setup()])


class CatkinPluginTestCase(CatkinPluginBaseTestCase):

    def test_schema(self):
        schema = catkin.CatkinPlugin.schema()

        # Check rosdistro property
        properties = schema['properties']
        self.assertTrue('rosdistro' in properties,
                        'Expected "rosdistro" to be included in properties')

        rosdistro = properties['rosdistro']
        self.assertTrue('type' in rosdistro,
                        'Expected "type" to be included in "rosdistro"')
        self.assertTrue('default' in rosdistro,
                        'Expected "default" to be included in "rosdistro"')

        rosdistro_type = rosdistro['type']
        self.assertEqual(rosdistro_type, 'string',
                         'Expected "rosdistro" "type" to be "string", but it '
                         'was "{}"'.format(rosdistro_type))

        rosdistro_default = rosdistro['default']
        self.assertEqual(rosdistro_default, 'indigo',
                         'Expected "rosdistro" "default" to be "indigo", but '
                         'it was "{}"'.format(rosdistro_default))

        # Check catkin-packages property
        self.assertTrue('catkin-packages' in properties,
                        'Expected "catkin-packages" to be included in '
                        'properties')

        catkin_packages = properties['catkin-packages']
        self.assertTrue('type' in catkin_packages,
                        'Expected "type" to be included in "catkin-packages"')
        self.assertTrue('default' in catkin_packages,
                        'Expected "default" to be included in '
                        '"catkin-packages"')
        self.assertTrue('minitems' in catkin_packages,
                        'Expected "minitems" to be included in '
                        '"catkin-packages"')
        self.assertTrue('uniqueItems' in catkin_packages,
                        'Expected "uniqueItems" to be included in '
                        '"catkin-packages"')
        self.assertTrue('items' in catkin_packages,
                        'Expected "items" to be included in "catkin-packages"')

        catkin_packages_type = catkin_packages['type']
        self.assertEqual(catkin_packages_type, 'array',
                         'Expected "catkin-packages" "type" to be "aray", but '
                         'it was "{}"'.format(catkin_packages_type))

        catkin_packages_default = catkin_packages['default']
        self.assertEqual(catkin_packages_default, [],
                         'Expected "catkin-packages" "default" to be [], but '
                         'it was {}'.format(catkin_packages_default))

        catkin_packages_minitems = catkin_packages['minitems']
        self.assertEqual(catkin_packages_minitems, 1,
                         'Expected "catkin-packages" "minitems" to be 1, but '
                         'it was {}'.format(catkin_packages_minitems))

        self.assertTrue(catkin_packages['uniqueItems'])

        catkin_packages_items = catkin_packages['items']
        self.assertTrue('type' in catkin_packages_items,
                        'Expected "type" to be included in "catkin-packages" '
                        '"items"')

        catkin_packages_items_type = catkin_packages_items['type']
        self.assertEqual(catkin_packages_items_type, 'string',
                         'Expected "catkin-packages" "item" "type" to be '
                         '"string", but it was "{}"'
                         .format(catkin_packages_items_type))

        # Check source-space property
        self.assertTrue('source-space' in properties,
                        'Expected "source-space" to be included in properties')

        source_space = properties['source-space']
        self.assertTrue('type' in source_space,
                        'Expected "type" to be included in "source-space"')
        self.assertTrue('default' in source_space,
                        'Expected "default" to be included in "source-space"')

        source_space_type = source_space['type']
        self.assertEqual(source_space_type, 'string',
                         'Expected "source-space" "type" to be "string", but '
                         'it was "{}"'.format(source_space_type))

        source_space_default = source_space['default']
        self.assertEqual(source_space_default, 'src',
                         'Expected "source-space" "default" to be "src", but '
                         'it was "{}"'.format(source_space_default))

        # Check include-roscore property
        self.assertTrue('include-roscore' in properties,
                        'Expected "include-roscore" to be included in '
                        'properties')

        include_roscore = properties['include-roscore']
        self.assertTrue('type' in include_roscore,
                        'Expected "type" to be included in "include-roscore"')
        self.assertTrue('default' in include_roscore,
                        'Expected "default" to be included in '
                        '"include-roscore"')

        include_roscore_type = include_roscore['type']
        self.assertEqual(include_roscore_type, 'boolean',
                         'Expected "include-roscore" "type" to be "boolean", '
                         'but it was "{}"'.format(include_roscore_type))

        include_roscore_default = include_roscore['default']
        self.assertEqual(include_roscore_default, 'true',
                         'Expected "include-roscore" "default" to be "true", '
                         'but it was "{}"'.format(include_roscore_default))

        # Check underlay property
        self.assertThat(properties, Contains('underlay'),
                        'Expected "underlay" to be included in properties')

        underlay = properties['underlay']
        self.assertThat(underlay, Contains('type'),
                        'Expected "type" to be included in "underlay"')
        self.assertThat(underlay, Contains('properties'),
                        'Expected "properties" to be included in "underlay"')
        self.assertThat(underlay, Contains('required'),
                        'Expected "required" to be included in "underlay"')

        underlay_type = underlay['type']
        self.assertThat(underlay_type, Equals('object'),
                        'Expected "underlay" "type" to be "object", but it '
                        'was "{}"'.format(underlay_type))

        underlay_required = underlay['required']
        self.assertThat(underlay_required, HasLength(2))
        self.assertThat(underlay_required, Contains('build-path'))
        self.assertThat(underlay_required, Contains('run-path'))

        underlay_properties = underlay['properties']
        self.assertThat(underlay_properties, Contains('build-path'))
        self.assertThat(underlay_properties, Contains('run-path'))

        underlay_build_path = underlay_properties['build-path']
        self.assertThat(underlay_build_path, Contains('type'))
        self.assertThat(underlay_build_path['type'], Equals('string'))

        underlay_run_path = underlay_properties['run-path']
        self.assertThat(underlay_run_path, Contains('type'))
        self.assertThat(underlay_run_path['type'], Equals('string'))

        # Check required
        self.assertTrue('catkin-packages' in schema['required'],
                        'Expected "catkin-packages" to be included in '
                        '"required"')

    def test_get_pull_properties(self):
        expected_pull_properties = ['rosdistro', 'catkin-packages',
                                    'source-space', 'include-roscore',
                                    'underlay']
        resulting_pull_properties = catkin.CatkinPlugin.get_pull_properties()

        self.assertThat(resulting_pull_properties,
                        HasLength(len(expected_pull_properties)))

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_invalid_rosdistro(self):
        self.properties.rosdistro = 'invalid'
        raised = self.assertRaises(
            RuntimeError,
            catkin.CatkinPlugin,
            'test-part', self.properties,
            self.project_options)

        self.assertEqual(str(raised),
                         "Unsupported rosdistro: 'invalid'. The supported ROS "
                         "distributions are 'indigo', 'jade', and 'kinetic'")

    def test_get_stage_sources_indigo(self):
        self.properties.rosdistro = 'indigo'
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        self.assertTrue('trusty' in plugin.PLUGIN_STAGE_SOURCES)

    def test_get_stage_sources_jade(self):
        self.properties.rosdistro = 'jade'
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        self.assertTrue('trusty' in plugin.PLUGIN_STAGE_SOURCES)

    def test_get_stage_sources_kinetic(self):
        self.properties.rosdistro = 'kinetic'
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        self.assertTrue('xenial' in plugin.PLUGIN_STAGE_SOURCES)

    @mock.patch('snapcraft.plugins.catkin._Compilers')
    def test_pull_invalid_dependency(self, compilers_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        self.dependencies_mock.return_value = ['foo']

        mock_instance = self.ubuntu_mock.return_value
        mock_instance.get.side_effect = repo.PackageNotFoundError('foo')

        raised = self.assertRaises(
            RuntimeError,
            plugin.pull)

        self.assertEqual(str(raised),
                         'Failed to fetch system dependencies: The Ubuntu '
                         "package 'foo' was not found.")

    def test_pull_unable_to_resolve_roscore(self):
        self.properties.include_roscore = True
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # No system dependencies
        self.dependencies_mock.return_value = set()

        self.rosdep_mock.return_value.resolve_dependency.return_value = None

        raised = self.assertRaises(RuntimeError, plugin.pull)

        self.assertEqual(str(raised),
                         'Unable to determine system dependency for roscore')

    @mock.patch.object(catkin.CatkinPlugin, '_generate_snapcraft_setup_sh')
    def test_pull_invalid_underlay(self, generate_setup_mock):
        self.properties.underlay = {
            'build-path': 'test-build-path',
            'run-path': 'test-run-path'
        }
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # No system dependencies
        self.dependencies_mock.return_value = set()

        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError, plugin.pull)

        self.assertThat(
            str(raised),
            MatchesRegex(
                '.*Requested underlay.*does not point to a valid directory'))

    def test_clean_pull(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        self.dependencies_mock.return_value = {'foo', 'bar', 'baz'}

        plugin.pull()
        os.makedirs(plugin._rosdep_path)

        plugin.clean_pull()
        self.assertFalse(os.path.exists(plugin._rosdep_path))

    def test_valid_catkin_workspace_src(self):
        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it contains a 'src' directory, this is a valid Catkin workspace.
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))
        # An exception will be raised if pull can't handle the valid workspace.
        plugin.pull()

    def test_invalid_catkin_workspace_no_src(self):
        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and `source-space` is 'src', this
        # should fail.
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        raised = self.assertRaises(FileNotFoundError, plugin.pull)

        self.assertEqual(
            str(raised),
            'Unable to find package path: "{}"'.format(os.path.join(
                plugin.sourcedir, 'src')))

    def test_valid_catkin_workspace_source_space(self):
        self.properties.source_space = 'foo'

        # sourcedir is expected to be the root of the Catkin workspace.
        # Normally this would mean it contained a `src` directory, but it can
        # be remapped via the `source-space` key.
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir,
                                 self.properties.source_space))
        # An exception will be raised if pull can't handle the source space.
        plugin.pull()

    def test_invalid_catkin_workspace_invalid_source_space(self):
        self.properties.source_space = 'foo'

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # it does not contain a `src` folder and source_space wasn't
        # specified, this should fail.
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        raised = self.assertRaises(FileNotFoundError, plugin.pull)

        self.assertEqual(
            str(raised),
            'Unable to find package path: "{}"'.format(os.path.join(
                plugin.sourcedir, self.properties.source_space)))

    def test_invalid_catkin_workspace_source_space_same_as_source(self):
        self.properties.source_space = '.'

        # sourcedir is expected to be the root of the Catkin workspace. Since
        # source_space was specified to be the same as the root, this should
        # fail.
        raised = self.assertRaises(
            RuntimeError,
            catkin.CatkinPlugin,
            'test-part', self.properties,
            self.project_options)

        self.assertEqual(str(raised),
                         'source-space cannot be the root of the Catkin '
                         'workspace')

    @mock.patch('snapcraft.plugins.catkin._Compilers')
    @mock.patch.object(catkin.CatkinPlugin, 'run')
    @mock.patch.object(catkin.CatkinPlugin, '_run_in_bash')
    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='foo')
    @mock.patch.object(catkin.CatkinPlugin, '_prepare_build')
    @mock.patch.object(catkin.CatkinPlugin, '_finish_build')
    def test_build(self, finish_build_mock, prepare_build_mock,
                   run_output_mock, bashrun_mock, run_mock, compilers_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        plugin.build()

        prepare_build_mock.assert_called_once_with()

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
                    '--install-space {}'.format(plugin.rosdir) in command and
                    '--source-space {}'.format(os.path.join(
                        plugin.builddir,
                        plugin.options.source_space)) in command)

        bashrun_mock.assert_called_with(check_build_command(), env=mock.ANY)

        self.assertFalse(
            self.dependencies_mock.called,
            'Dependencies should have been discovered in the pull() step')

        finish_build_mock.assert_called_once_with()

    @mock.patch('snapcraft.plugins.catkin._Compilers')
    @mock.patch.object(catkin.CatkinPlugin, 'run')
    @mock.patch.object(catkin.CatkinPlugin, '_run_in_bash')
    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='foo')
    @mock.patch.object(catkin.CatkinPlugin, '_prepare_build')
    @mock.patch.object(catkin.CatkinPlugin, '_finish_build')
    def test_build_multiple(self, finish_build_mock, prepare_build_mock,
                            run_output_mock, bashrun_mock, run_mock,
                            compilers_mock):
        self.properties.catkin_packages.append('package_2')

        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        plugin.build()

        class check_pkg_arguments():
            def __init__(self, test):
                self.test = test

            def __eq__(self, args):
                index = args.index('--pkg')
                packages = args[index+1:index+3]
                self.test.assertIn('my_package', packages)
                self.test.assertIn('package_2', packages)
                return True

        bashrun_mock.assert_called_with(
            check_pkg_arguments(self), env=mock.ANY)

        self.assertFalse(
            self.dependencies_mock.called,
            'Dependencies should have been discovered in the pull() step')

        finish_build_mock.assert_called_once_with()

    @mock.patch('snapcraft.plugins.catkin._Compilers')
    @mock.patch.object(catkin.CatkinPlugin, 'run')
    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='foo')
    def test_build_runs_in_bash(self, run_output_mock, run_mock,
                                compilers_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['/bin/bash', mock.ANY], cwd=mock.ANY, env=mock.ANY)
        ])

    def test_use_in_snap_python_rewrites_shebangs(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.rosdir, 'bin'))

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                'path': os.path.join(plugin.rosdir, '_setup_util.py'),
                'contents': '#!/foo/bar/baz/python',
                'expected': '#!/usr/bin/env python',
            },
            {
                'path': os.path.join(plugin.rosdir, 'bin/catkin'),
                'contents': '#!/foo/baz/python',
                'expected': '#!/usr/bin/env python',
            },
            {
                'path': os.path.join(plugin.rosdir, 'foo'),
                'contents': 'foo',
                'expected': 'foo',
            }
        ]

        for file_info in files:
            with open(file_info['path'], 'w') as f:
                f.write(file_info['contents'])

        plugin._use_in_snap_python()

        for file_info in files:
            with open(os.path.join(plugin.rosdir,
                                   file_info['path']), 'r') as f:
                self.assertThat(f.read(), Equals(file_info['expected']))

    @mock.patch.object(catkin.CatkinPlugin, 'run')
    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='foo')
    def test_use_in_snap_python_skips_binarys(self, run_output_mock, run_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(plugin.rosdir)

        # Place a file to be discovered by _use_in_snap_python().
        open(os.path.join(plugin.rosdir, 'foo'), 'w').close()

        file_mock = mock.mock_open()
        with mock.patch.object(builtins, 'open', file_mock):
            # Reading a binary file may throw a UnicodeDecodeError. Make sure
            # that's handled.
            file_mock.return_value.read.side_effect = UnicodeDecodeError(
                'foo', b'bar', 1, 2, 'baz')
            # An exception will be raised if the function can't handle the
            # binary file.
            plugin._use_in_snap_python()

    def test_use_in_snap_python_rewrites_10_ros_sh(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.rosdir, 'etc', 'catkin', 'profile.d'))

        ros_profile = os.path.join(plugin.rosdir, 'etc', 'catkin', 'profile.d',
                                   '10.ros.sh')

        # Place 10.ros.sh with an absolute path to python
        with open(ros_profile, 'w') as f:
            f.write('/usr/bin/python foo')

        plugin._use_in_snap_python()

        # Verify that the absolute path in 10.ros.sh was rewritten correctly
        with open(ros_profile, 'r') as f:
            self.assertEqual(f.read(), 'python foo',
                             'The absolute path to python was not replaced as '
                             'expected')

    @mock.patch.object(catkin.CatkinPlugin, '_source_setup_sh',
                       return_value='test-source-setup')
    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='bar')
    def test_run_environment(self, run_mock, source_setup_sh_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)

        python_path = os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2.7', 'dist-packages')
        os.makedirs(python_path)

        # Joining and re-splitting to get hacked script in there as well
        environment = '\n'.join(plugin.env(plugin.installdir)).split('\n')

        self.assertThat(environment, Contains(
            'PYTHONPATH={}:$PYTHONPATH'.format(python_path)))

        self.assertThat(environment, Contains(
            'ROS_MASTER_URI=http://localhost:11311'))

        self.assertThat(environment, Contains('ROS_HOME=$SNAP_USER_DATA/ros'))

        self.assertThat(environment, Contains('LC_ALL=C.UTF-8'))

        source_setup_sh_mock.assert_called_with(plugin.installdir, None)
        self.assertThat(environment, Contains('test-source-setup'))

    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='bar')
    def test_run_environment_with_underlay(self, run_mock):
        self.properties.underlay = {
            'build-path': 'test-build-path',
            'run-path': 'test-run-path',
        }
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)

        python_path = os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2.7', 'dist-packages')
        os.makedirs(python_path)

        # Joining and re-splitting to get hacked script in there as well
        environment = '\n'.join(plugin.env(plugin.installdir)).split('\n')

        self.assertThat(environment, Contains(
            'PYTHONPATH={}:$PYTHONPATH'.format(python_path)))

        self.assertThat(environment, Contains(
            'ROS_MASTER_URI=http://localhost:11311'))

        self.assertThat(environment, Contains('ROS_HOME=$SNAP_USER_DATA/ros'))

        self.assertThat(environment, Contains('LC_ALL=C.UTF-8'))

        self.assertThat(environment, Contains('. {}'.format(
            os.path.join(plugin.rosdir, 'snapcraft-setup.sh'))))

    @mock.patch.object(catkin.CatkinPlugin, '_source_setup_sh',
                       return_value='test-source-setup')
    def test_generate_snapcraft_sh(self, source_setup_sh_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)

        plugin._generate_snapcraft_setup_sh('test-root', None)
        source_setup_sh_mock.assert_called_with('test-root', None)
        with open(os.path.join(plugin.rosdir, 'snapcraft-setup.sh'), 'r') as f:
            self.assertThat(f.read(), Equals('test-source-setup'))

    def test_source_setup_sh(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)

        # Make sure $@ is zeroed, then setup.sh sourced, then $@ is restored
        rosdir = os.path.join(
            'test-root', 'opt', 'ros', self.properties.rosdistro)
        setup_path = os.path.join(rosdir, 'setup.sh')
        lines_of_interest = [
            'set --',
            'if [ -f {} ]; then'.format(setup_path),
            '_CATKIN_SETUP_DIR={} . {}'.format(rosdir, setup_path),
            'fi',
            'eval "set -- $BACKUP_ARGS"',
        ]
        actual_lines = []

        for line in plugin._source_setup_sh('test-root', None).split('\n'):
            line = line.strip()
            if line in lines_of_interest:
                actual_lines.append(line)

        self.assertThat(
            actual_lines, Equals(lines_of_interest),
            "Expected ROS's setup.sh to be sourced after args were zeroed, "
            "followed by the args being restored.")

    def test_generate_snapcraft_sh_with_underlay(self):
        self.properties.underlay = {
            'build-path': 'test-build-underlay',
            'run-path': 'test-run-underlay'
        }
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)

        # Make sure $@ is zeroed, then setup.sh sourced, then $@ is restored
        underlay_setup_path = os.path.join(
            'test-underlay', 'setup.sh')
        rosdir = os.path.join(
            'test-root', 'opt', 'ros', self.properties.rosdistro)
        setup_path = os.path.join(rosdir, 'setup.sh')
        lines_of_interest = [
            'set --',
            'if [ -f {} ]; then'.format(underlay_setup_path),
            '_CATKIN_SETUP_DIR={} . {}'.format(
                'test-underlay', underlay_setup_path),
            'if [ -f {} ]; then'.format(setup_path),
            'set -- --extend',
            '_CATKIN_SETUP_DIR={} . {}'.format(rosdir, setup_path),
            'fi',
            'fi',
            'eval "set -- $BACKUP_ARGS"',
        ]
        actual_lines = []

        plugin._generate_snapcraft_setup_sh('test-root', 'test-underlay')
        with open(os.path.join(plugin.rosdir, 'snapcraft-setup.sh'), 'r') as f:
            for line in f:
                line = line.strip()
                if line in lines_of_interest:
                    actual_lines.append(line)

        self.assertThat(
            actual_lines, Equals(lines_of_interest),
            "Expected ROS's setup.sh to be sourced after args were zeroed, "
            "followed by the args being restored.")

    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='bar')
    def test_run_environment_no_python(self, run_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)

        python_path = os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2.7', 'dist-packages')

        environment = plugin.env(plugin.installdir)

        self.assertFalse(
            'PYTHONPATH={}'.format(python_path) in environment, environment)

    @mock.patch.object(catkin.CatkinPlugin, '_use_in_snap_python')
    def test_prepare_build(self, use_python_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.rosdir, 'test'))

        # Place a few .cmake files with incorrect paths, and some files that
        # shouldn't be changed.
        files = [
            {
                'path': 'fooConfig.cmake',
                'contents': '"/usr/lib/foo"',
                'expected': '"{}/usr/lib/foo"'.format(plugin.installdir),
            },
            {
                'path': 'bar.cmake',
                'contents': '"/usr/lib/bar"',
                'expected': '"/usr/lib/bar"',
            },
            {
                'path': 'test/bazConfig.cmake',
                'contents': '"/test/baz;/usr/lib/baz"',
                'expected': '"{0}/test/baz;{0}/usr/lib/baz"'.format(
                    plugin.installdir),
            },
            {
                'path': 'test/quxConfig.cmake',
                'contents': 'qux',
                'expected': 'qux',
            },
            {
                'path': 'test/installedConfig.cmake',
                'contents': '"{}/foo"'.format(plugin.installdir),
                'expected': '"{}/foo"'.format(plugin.installdir),
            }
        ]

        for file_info in files:
            path = os.path.join(plugin.rosdir, file_info['path'])
            with open(path, 'w') as f:
                f.write(file_info['contents'])

        plugin._prepare_build()

        self.assertTrue(use_python_mock.called)

        for file_info in files:
            path = os.path.join(plugin.rosdir, file_info['path'])
            with open(path, 'r') as f:
                self.assertThat(f.read(), Equals(file_info['expected']))


class PullTestCase(CatkinPluginBaseTestCase):

    scenarios = [
        ('no underlay', {
            'underlay': None,
            'expected_underlay_path': None,
        }),
        ('underlay', {
            'underlay': {
                'build-path': 'test-build-path',
                'run-path': 'test-run-path',
            },
            'expected_underlay_path': 'test-build-path'
        })
    ]

    def setUp(self):
        super().setUp()

        # Make the underlay a valid workspace
        if self.underlay:
            os.makedirs(self.underlay['build-path'])
            open(os.path.join(
                self.underlay['build-path'], 'setup.sh'), 'w').close()
            self.properties.underlay = self.underlay

    @mock.patch.object(catkin.CatkinPlugin, '_generate_snapcraft_setup_sh')
    def test_pull_debian_dependencies(self, generate_setup_mock):
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        self.dependencies_mock.return_value = {'foo', 'bar', 'baz'}

        plugin.pull()

        self.verify_rosdep_setup(
            self.properties.rosdistro,
            os.path.join(plugin.sourcedir, 'src'),
            os.path.join(plugin.partdir, 'rosdep'),
            plugin.PLUGIN_STAGE_SOURCES)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path)
        else:
            generate_setup_mock.assert_not_called()

        # Verify that dependencies were found as expected. TODO: Would really
        # like to use ANY here instead of verifying explicit arguments, but
        # Python issue #25195 won't let me.
        self.assertEqual(1, self.dependencies_mock.call_count)
        self.assertEqual({'my_package'},
                         self.dependencies_mock.call_args[0][0])

        # Verify that the dependencies were installed
        self.ubuntu_mock.return_value.get.assert_called_with(
            _CompareContainers(self, ['foo', 'bar', 'baz']))
        self.ubuntu_mock.return_value.unpack.assert_called_with(
            plugin.installdir)

    @mock.patch.object(catkin.CatkinPlugin, '_generate_snapcraft_setup_sh')
    def test_pull_local_dependencies(self, generate_setup_mock):
        self.properties.catkin_packages.append('package_2')

        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # No system dependencies (only local)
        self.dependencies_mock.return_value = set()

        plugin.pull()

        self.verify_rosdep_setup(
            self.properties.rosdistro,
            os.path.join(plugin.sourcedir, 'src'),
            os.path.join(plugin.partdir, 'rosdep'),
            plugin.PLUGIN_STAGE_SOURCES)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path)
        else:
            generate_setup_mock.assert_not_called()

        # Verify that dependencies were found as expected. TODO: Would really
        # like to use ANY here instead of verifying explicit arguments, but
        # Python issue #25195 won't let me.
        self.assertEqual(1, self.dependencies_mock.call_count)
        self.assertEqual({'my_package', 'package_2'},
                         self.dependencies_mock.call_args[0][0])

        # Verify that no .deb packages were installed
        self.assertTrue(mock.call().unpack(plugin.installdir) not in
                        self.ubuntu_mock.mock_calls)

    @mock.patch.object(catkin.CatkinPlugin, '_generate_snapcraft_setup_sh')
    def test_pull_with_roscore(self, generate_setup_mock):
        self.properties.include_roscore = True
        plugin = catkin.CatkinPlugin('test-part', self.properties,
                                     self.project_options)
        os.makedirs(os.path.join(plugin.sourcedir, 'src'))

        # No system dependencies
        self.dependencies_mock.return_value = set()

        def resolve(package_name):
            if package_name == 'ros_core':
                return ['ros-core-dependency']

        self.rosdep_mock.return_value.resolve_dependency = resolve

        plugin.pull()

        self.verify_rosdep_setup(
            self.properties.rosdistro,
            os.path.join(plugin.sourcedir, 'src'),
            os.path.join(plugin.partdir, 'rosdep'),
            plugin.PLUGIN_STAGE_SOURCES)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                plugin.installdir, self.expected_underlay_path)
        else:
            generate_setup_mock.assert_not_called()

        # Verify that roscore was installed
        self.ubuntu_mock.return_value.get.assert_called_with(
            {'ros-core-dependency'})
        self.ubuntu_mock.return_value.unpack.assert_called_with(
            plugin.installdir)


class FinishBuildTestCase(CatkinPluginBaseTestCase):

    scenarios = [
        ('no underlay', {
            'underlay': None,
            'expected_underlay_path': None,
        }),
        ('underlay', {
            'underlay': {
                'build-path': 'test-build-path',
                'run-path': 'test-run-path',
            },
            'expected_underlay_path': 'test-run-path'
        })
    ]

    def setUp(self):
        super().setUp()

        self.properties.underlay = self.underlay
        self.plugin = catkin.CatkinPlugin(
            'test-part', self.properties, self.project_options)

    @mock.patch.object(catkin.CatkinPlugin, '_generate_snapcraft_setup_sh')
    @mock.patch.object(catkin.CatkinPlugin, '_use_in_snap_python')
    def test_finish_build_cmake_paths(self, use_python_mock,
                                      generate_setup_mock):
        os.makedirs(os.path.join(self.plugin.rosdir, 'test'))

        # Place a few .cmake files with incorrect paths, and some files that
        # shouldn't be changed.
        files = [
            {
                'path': 'fooConfig.cmake',
                'contents': '"{}/usr/lib/foo"'.format(self.plugin.installdir),
                'expected': '"$ENV{SNAPCRAFT_STAGE}/usr/lib/foo"',
            },
            {
                'path': 'bar.cmake',
                'contents': '"/usr/lib/bar"',
                'expected': '"/usr/lib/bar"',
            },
            {
                'path': 'test/bazConfig.cmake',
                'contents': '"{0}/test/baz;{0}/usr/lib/baz"'.format(
                    self.plugin.installdir),
                'expected': '"$ENV{SNAPCRAFT_STAGE}/test/baz;'
                            '$ENV{SNAPCRAFT_STAGE}/usr/lib/baz"',
            },
            {
                'path': 'test/quxConfig.cmake',
                'contents': 'qux',
                'expected': 'qux',
            },
            {
                'path': 'test/installedConfig.cmake',
                'contents': '"$ENV{SNAPCRAFT_STAGE}/foo"',
                'expected': '"$ENV{SNAPCRAFT_STAGE}/foo"',
            }
        ]

        for file_info in files:
            path = os.path.join(self.plugin.rosdir, file_info['path'])
            with open(path, 'w') as f:
                f.write(file_info['contents'])

        self.plugin._finish_build()

        self.assertTrue(use_python_mock.called)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                '$SNAP', self.expected_underlay_path)
        else:
            generate_setup_mock.assert_not_called()

        for file_info in files:
            path = os.path.join(self.plugin.rosdir, file_info['path'])
            with open(path, 'r') as f:
                self.assertThat(f.read(), Equals(file_info['expected']))

    @mock.patch.object(catkin.CatkinPlugin, '_generate_snapcraft_setup_sh')
    @mock.patch.object(catkin.CatkinPlugin, 'run')
    @mock.patch.object(catkin.CatkinPlugin, 'run_output', return_value='foo')
    @mock.patch.object(catkin.CatkinPlugin, '_use_in_snap_python')
    def test_finish_build_cmake_prefix_path(self, use_python_mock,
                                            run_output_mock, run_mock,
                                            generate_setup_mock):
        setup_file = os.path.join(self.plugin.rosdir, '_setup_util.py')
        os.makedirs(os.path.dirname(setup_file))

        with open(setup_file, 'w') as f:
            f.write("CMAKE_PREFIX_PATH = '{0}/{1};{0}\n".format(
                self.plugin.rosdir, self.plugin.options.rosdistro))

        self.plugin._finish_build()

        self.assertTrue(use_python_mock.called)

        # This shouldn't be called unless there's an underlay
        if self.properties.underlay:
            generate_setup_mock.assert_called_once_with(
                '$SNAP', self.expected_underlay_path)
        else:
            generate_setup_mock.assert_not_called()

        expected = 'CMAKE_PREFIX_PATH = []\n'

        with open(setup_file, 'r') as f:
            self.assertEqual(
                f.read(), expected,
                'The absolute path to python or the CMAKE_PREFIX_PATH '
                'was not replaced as expected')


class FindSystemDependenciesTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        self.rosdep_mock = mock.MagicMock()
        self.rosdep_mock.get_dependencies.return_value = ['bar']

        self.catkin_mock = mock.MagicMock()
        exception = catkin.CatkinPackageNotFoundError('foo')
        self.catkin_mock.find.side_effect = exception

    def test_find_system_dependencies_system_only(self):
        self.rosdep_mock.resolve_dependency.return_value = ['baz']

        self.assertThat(catkin._find_system_dependencies(
            {'foo'}, self.rosdep_mock, self.catkin_mock), Equals({'baz'}))

        self.rosdep_mock.get_dependencies.assert_called_once_with('foo')
        self.rosdep_mock.resolve_dependency.assert_called_once_with('bar')
        self.catkin_mock.find.assert_called_once_with('bar')

    def test_find_system_dependencies_local_only(self):
        self.assertThat(catkin._find_system_dependencies(
            {'foo', 'bar'}, self.rosdep_mock, self.catkin_mock),
            HasLength(0))

        self.rosdep_mock.get_dependencies.assert_has_calls(
            [mock.call('foo'), mock.call('bar')], any_order=True)
        self.rosdep_mock.resolve_dependency.assert_not_called()
        self.catkin_mock.find.assert_not_called()

    def test_find_system_dependencies_satisfied_in_stage(self):
        self.catkin_mock.find.side_effect = None
        self.catkin_mock.find.return_value = 'baz'

        self.assertThat(catkin._find_system_dependencies(
            {'foo'}, self.rosdep_mock, self.catkin_mock), HasLength(0))

        self.rosdep_mock.get_dependencies.assert_called_once_with('foo')
        self.catkin_mock.find.assert_called_once_with('bar')
        self.rosdep_mock.resolve_dependency.assert_not_called()

    def test_find_system_dependencies_mixed(self):
        self.rosdep_mock.get_dependencies.return_value = ['bar', 'baz', 'qux']
        self.rosdep_mock.resolve_dependency.return_value = ['quux']

        def _fake_find(package_name):
            if package_name == 'qux':
                return package_name
            raise catkin.CatkinPackageNotFoundError(package_name)

        self.catkin_mock.find.side_effect = _fake_find
        self.assertThat(catkin._find_system_dependencies(
            {'foo', 'bar'}, self.rosdep_mock, self.catkin_mock),
            Equals({'quux'}))

        self.rosdep_mock.get_dependencies.assert_has_calls(
            [mock.call('foo'), mock.call('bar')], any_order=True)
        self.rosdep_mock.resolve_dependency.assert_called_once_with('baz')
        self.catkin_mock.find.assert_has_calls(
            [mock.call('baz'), mock.call('qux')], any_order=True)

    def test_find_system_dependencies_missing_local_dependency(self):
        # Setup a dependency on a non-existing package, and it doesn't resolve
        # to a system dependency.'
        exception = catkin.SystemDependencyNotFoundError('foo')
        self.rosdep_mock.resolve_dependency.side_effect = exception

        raised = self.assertRaises(
            RuntimeError,
            catkin._find_system_dependencies,
            {'foo'}, self.rosdep_mock, self.catkin_mock)

        self.assertEqual(raised.args[0],
                         "Package 'bar' isn't a valid system dependency. Did "
                         "you forget to add it to catkin-packages? If not, "
                         "add the Ubuntu package containing it to "
                         "stage-packages until you can get it into the rosdep "
                         "database.")


class RosdepTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.project = snapcraft.ProjectOptions()
        self.rosdep = catkin._Rosdep('kinetic', 'package_path',
                                     'rosdep_path', 'sources',
                                     self.project)

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
            mock.call(self.rosdep._rosdep_path, sources='sources',
                      project_options=self.project),
            mock.call().get(['python-rosdep']),
            mock.call().unpack(self.rosdep._rosdep_install_path)])

        # Verify that rosdep was initialized and updated
        self.assertEqual(self.check_output_mock.call_count, 2)
        self.check_output_mock.assert_has_calls([
            mock.call(['rosdep', 'init'], env=mock.ANY),
            mock.call(['rosdep', 'update'], env=mock.ANY)
        ])

    def test_setup_can_run_multiple_times(self):
        self.rosdep.setup()

        # Make sure running setup() again doesn't have problems with the old
        # environment
        # An exception will be raised if setup can't be called twice.
        self.rosdep.setup()

    def test_setup_initialization_failure(self):
        def run(args, **kwargs):
            if args == ['rosdep', 'init']:
                raise subprocess.CalledProcessError(1, 'foo', b'bar')

        self.check_output_mock.side_effect = run

        raised = self.assertRaises(RuntimeError, self.rosdep.setup)

        self.assertEqual(str(raised),
                         'Error initializing rosdep database:\nbar')

    def test_setup_update_failure(self):
        def run(args, **kwargs):
            if args == ['rosdep', 'update']:
                raise subprocess.CalledProcessError(1, 'foo', b'bar')

            return mock.DEFAULT

        self.check_output_mock.side_effect = run

        raised = self.assertRaises(RuntimeError, self.rosdep.setup)

        self.assertEqual(str(raised),
                         'Error updating rosdep database:\nbar')

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

        raised = self.assertRaises(
            FileNotFoundError,
            self.rosdep.get_dependencies, 'bar')

        self.assertEqual(str(raised),
                         'Unable to find Catkin package "bar"')

    def test_resolve_dependency(self):
        self.check_output_mock.return_value = b'#apt\nmylib-dev'

        self.assertEqual(self.rosdep.resolve_dependency('foo'), ['mylib-dev'])

        self.check_output_mock.assert_called_with(
            ['rosdep', 'resolve', 'foo', '--rosdistro', 'kinetic', '--os',
             'ubuntu:xenial'],
            env=mock.ANY)

    def test_resolve_invalid_dependency(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo')

        raised = self.assertRaises(
            catkin.SystemDependencyNotFoundError,
            self.rosdep.resolve_dependency, 'bar')

        self.assertEqual(str(raised),
                         "'bar' does not resolve to a system dependency")

    def test_resolve_no_dependency(self):
        self.check_output_mock.return_value = b'#apt'

        self.assertEqual(self.rosdep.resolve_dependency('bar'), [])

    def test_resolve_multiple_dependencies(self):
        self.check_output_mock.return_value = b'#apt\nlib1 lib2'

        self.assertEqual(self.rosdep.resolve_dependency('foo'),
                         ['lib1', 'lib2'])

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


class CompilersTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.project = snapcraft.ProjectOptions()
        self.compilers = catkin._Compilers(
            'compilers_path', 'sources', self.project)

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('subprocess.check_output')
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # Pretend we have gcc v5
        os.makedirs(os.path.join(
            self.compilers._compilers_install_path, 'usr', 'include', 'c++',
            '5'))
        os.makedirs(os.path.join(
            self.compilers._compilers_install_path, 'usr', 'include',
            self.project.arch_triplet, 'c++', '5'))

    def test_setup(self):
        # Return something other than a Mock to ease later assertions
        self.check_output_mock.return_value = b''

        self.compilers.setup()

        # Verify that both gcc and g++ were installed (no other .debs)
        self.assertEqual(self.ubuntu_mock.call_count, 1)
        self.assertEqual(self.ubuntu_mock.return_value.get.call_count, 1)
        self.assertEqual(self.ubuntu_mock.return_value.unpack.call_count, 1)
        self.ubuntu_mock.assert_has_calls([
            mock.call(self.compilers._compilers_path, sources='sources',
                      project_options=self.project),
            mock.call().get(['gcc', 'g++']),
            mock.call().unpack(self.compilers._compilers_install_path)])

    def test_setup_can_run_multiple_times(self):
        self.compilers.setup()

        # Make sure running setup() again doesn't have problems with the old
        # environment. An exception will be raised if setup can't be called
        # twice.
        self.compilers.setup()

    def test_environment(self):
        # Setup a few valid library paths
        library_path1 = os.path.join(
            self.compilers._compilers_install_path, 'usr', 'lib')
        library_path2 = os.path.join(
            self.compilers._compilers_install_path, 'usr', 'lib',
            self.project.arch_triplet)
        for path in (library_path1, library_path2):
            os.makedirs(path)

        environment = self.compilers.environment

        self.assertThat(environment, Contains('LD_LIBRARY_PATH'))
        self.expectThat(
            environment['LD_LIBRARY_PATH'], Contains(library_path1))
        self.expectThat(
            environment['LD_LIBRARY_PATH'], Contains(library_path2))

        self.assertThat(environment, Contains('PATH'))
        self.expectThat(environment['PATH'], Contains(os.path.join(
            self.compilers._compilers_install_path, 'usr', 'bin')))

    def test_c_compiler_path(self):
        self.assertThat(
            self.compilers.c_compiler_path,
            Equals(os.path.join(self.compilers._compilers_install_path, 'usr',
                   'bin', 'gcc')))

    def test_cxx_compiler_path(self):
        self.assertThat(
            self.compilers.cxx_compiler_path,
            Equals(os.path.join(self.compilers._compilers_install_path, 'usr',
                   'bin', 'g++')))

    def test_cflags(self):
        self.assertThat(self.compilers.cflags, Equals(''))

    def test_cxxflags(self):
        cxx_include_dir = os.path.join(
            self.compilers._compilers_install_path, 'usr', 'include')
        self.assertThat(
            self.compilers.cxxflags, Contains('-I{}'.format(cxx_include_dir)))
        self.assertThat(
            self.compilers.cxxflags,
            Contains('-I{}'.format(os.path.join(cxx_include_dir, 'c++', '5'))))
        self.assertThat(
            self.compilers.cxxflags,
            Contains('-I{}'.format(os.path.join(
                cxx_include_dir, self.project.arch_triplet, 'c++', '5'))))

    def test_cxxflags_without_include_path_raises(self):
        shutil.rmtree(self.compilers._compilers_install_path)
        with testtools.ExpectedException(
                RuntimeError, 'Unable to determine gcc version: nothing.*'):
            self.compilers.cxxflags

    def test_ldflags(self):
        self.assertThat(self.compilers.ldflags, Equals(''))


class CatkinFindTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        self.project = snapcraft.ProjectOptions()
        self.catkin = catkin._Catkin(
            'kinetic', 'workspace_path', 'catkin_path', 'sources',
            self.project)

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('subprocess.check_output')
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_setup(self):
        # Return something other than a Mock to ease later assertions
        self.check_output_mock.return_value = b''

        self.catkin.setup()

        # Verify that only rospack was installed (no other .debs)
        self.assertThat(self.ubuntu_mock.call_count, Equals(1))
        self.assertThat(
            self.ubuntu_mock.return_value.get.call_count, Equals(1))
        self.assertThat(
            self.ubuntu_mock.return_value.unpack.call_count, Equals(1))
        self.ubuntu_mock.assert_has_calls([
            mock.call(self.catkin._catkin_path, sources='sources',
                      project_options=self.project),
            mock.call().get(['ros-kinetic-catkin']),
            mock.call().unpack(self.catkin._catkin_install_path)])

    def test_setup_can_run_multiple_times(self):
        self.catkin.setup()

        # Make sure running setup() again doesn't have problems with the old
        # environment. An exception will be raised if setup() can't be called
        # twice.
        self.catkin.setup()

    def test_find(self):
        self.check_output_mock.return_value = b'bar'

        self.assertThat(self.catkin.find('foo'), Equals('bar'))

        self.assertTrue(self.check_output_mock.called)
        positional_args = self.check_output_mock.call_args[0][0]
        self.assertThat(
            ' '.join(positional_args),
            Contains('catkin_find --first-only foo'))

    def test_find_non_existing_package(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(
            1, 'foo')

        with testtools.ExpectedException(
                catkin.CatkinPackageNotFoundError,
                "Unable to find Catkin package 'foo'"):
            self.catkin.find('foo')

        self.assertTrue(self.check_output_mock.called)
        positional_args = self.check_output_mock.call_args[0][0]
        self.assertThat(
            ' '.join(positional_args),
            Contains('catkin_find --first-only foo'))
