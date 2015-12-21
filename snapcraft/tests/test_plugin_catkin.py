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

import fixtures
import os
import os.path
import tempfile

from unittest import mock

from snapcraft import tests
from snapcraft.plugins import catkin


class _IOError(IOError):
    errno = os.errno.EACCES


class CatkinPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class props:
            rosversion = 'foo'
            catkin_packages = ['my_package']

        self.properties = props()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_pull_debian_dependencies(self):
        plugin = catkin.CatkinPlugin('test-part', self.properties)

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.sourcedir = tmpdir.name

        # Create ROS package directory
        os.mkdir(os.path.join(plugin.sourcedir, "my_package"))

        # Now create my_package's package.xml:
        with open(os.path.join(plugin.sourcedir, "my_package",
                               "package.xml"), 'w') as f:
            f.write("""<?xml version="1.0"?>
                      <package>
                        <buildtool_depend>buildtool_depend</buildtool_depend>
                        <build_depend>build_depend</build_depend>
                        <run_depend>run_depend</run_depend>
                      </package>""")

        plugin.pull()

        self.ubuntu_mock.assert_has_calls([
            mock.call().get([
                'ros-foo-buildtool-depend',
                'ros-foo-build-depend',
                'ros-foo-run-depend']),
            mock.call().unpack(plugin.installdir)])

    def test_pull_local_dependencies(self):
        self.properties.catkin_packages.append('package_2')

        plugin = catkin.CatkinPlugin('test-part', self.properties)

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.sourcedir = tmpdir.name

        # Create ROS package directory for both packages
        os.mkdir(os.path.join(plugin.sourcedir, "my_package"))
        os.mkdir(os.path.join(plugin.sourcedir, "package_2"))

        # Now create my_package's package.xml, specifying that is depends upon
        # package_2:
        with open(os.path.join(plugin.sourcedir, "my_package",
                               "package.xml"), 'w') as f:
            f.write("""<?xml version="1.0"?>
                      <package>
                        <build_depend>package_2</build_depend>
                      </package>""")

        # Finally, create package_2's package.xml, specifying that is has no
        # dependencies:
        with open(os.path.join(plugin.sourcedir, "package_2",
                               "package.xml"), 'w') as f:
            f.write("""<?xml version="1.0"?>
                      <package>
                      </package>""")

        plugin.pull()

        self.assertTrue('my_package' in plugin.package_local_deps,
                        'Expected "my_package" to be in the dependencies')
        self.assertEqual(plugin.package_local_deps['my_package'],
                         {'package_2'},
                         'Expected "my_package" to depend upon "package_2"')
        self.assertFalse(self.ubuntu_mock.called,
                         "Ubuntu packages were unexpectedly pulled down")

    def test_log_warning_when_unable_to_find_a_catkin_package(self):
        fake_logger = fixtures.FakeLogger()
        self.useFixture(fake_logger)

        plugin = catkin.CatkinPlugin('test-part', self.properties)

        with self.assertRaises(FileNotFoundError) as raised:
            plugin._find_package_deps()

        self.assertEqual(raised.exception.args[0],
                         'unable to find "package.xml" for "my_package"')

    @mock.patch('snapcraft.plugins.catkin.open', create=True)
    def test_exception_raised_when_package_definition_cannot_be_read(
            self, mock_open):
        mock_open.side_effect = _IOError()
        plugin = catkin.CatkinPlugin('test-part', self.properties)
        with self.assertRaises(IOError) as raised:
            plugin._find_package_deps()

        self.assertEqual(raised.exception.errno, os.errno.EACCES)
        xml_to_open = os.path.join(
            os.path.abspath(os.curdir), 'parts', 'test-part', 'src',
            'my_package', 'package.xml')
        mock_open.assert_called_once_with(xml_to_open, 'r')

    def test_build_with_subdir_without_src_copies_subdir_into_src(self):
        class Options:
            catkin_packages = []
            source_subdir = 'src_subdir'

        plugin = catkin.CatkinPlugin('test-part', Options())

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
        class Options:
            catkin_packages = []

        plugin = catkin.CatkinPlugin('test-part', Options())

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
