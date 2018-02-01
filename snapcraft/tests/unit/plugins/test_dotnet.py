# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import subprocess
import tarfile
from unittest import mock

from testtools.matchers import (
    Contains,
    DirExists,
    Equals,
    FileExists,
    Not)

import snapcraft
from snapcraft import file_utils
from snapcraft.internal import sources
from snapcraft.plugins import dotnet
from snapcraft.tests import unit


def _setup_dirs(plugin):
    os.makedirs(plugin.builddir)
    open(os.path.join(plugin.builddir, 'test-app.xxproj'), 'w').close()
    os.makedirs(plugin.installdir)
    executable_path = os.path.join(plugin.installdir, 'test-app')
    open(executable_path, 'w').close()


class DotNetPluginTestCase(unit.TestCase):

    def test_schema(self):
        schema = dotnet.DotNetPlugin.schema()
        self.assertThat(schema, Not(Contains('required')))

    def test_get_pull_properties(self):
        expected_pull_properties = []
        self.assertThat(
            dotnet.DotNetPlugin.get_pull_properties(),
            Equals(expected_pull_properties))

    def test_get_build_properties(self):
        expected_build_properties = []
        self.assertThat(
            dotnet.DotNetPlugin.get_build_properties(),
            Equals(expected_build_properties))


class DotNetProjectBaseTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            build_attributes = []

        self.options = Options()
        self.project = snapcraft.ProjectOptions()

        # Only amd64 is supported for now.
        patcher = mock.patch(
            'snapcraft.ProjectOptions.deb_arch',
            new_callable=mock.PropertyMock,
            return_value='amd64')
        patcher.start()
        self.addCleanup(patcher.stop)

        original_check_call = subprocess.check_call
        patcher = mock.patch('subprocess.check_call')
        self.mock_check_call = patcher.start()
        self.addCleanup(patcher.stop)

        def side_effect(cmd, *args, **kwargs):
            if cmd[2].endswith('dotnet'):
                pass
            else:
                original_check_call(cmd, *args, **kwargs)

        self.mock_check_call.side_effect = side_effect


class DotNetProjectTestCase(DotNetProjectBaseTestCase):

    def test_init_with_non_amd64_architecture(self):
        with mock.patch(
                'snapcraft.ProjectOptions.deb_arch',
                new_callable=mock.PropertyMock,
                return_value='non-amd64'):
            error = self.assertRaises(
                NotImplementedError,
                dotnet.DotNetPlugin,
                'test-part', self.options, self.project)
        self.assertThat(
            str(error),
            Equals("This plugin does not support architecture 'non-amd64'"))

    def test_pull_sdk(self):
        with tarfile.open('test-sdk.tar', 'w') as test_sdk_tar:
            open('test-sdk', 'w').close()
            test_sdk_tar.add('test-sdk')
        with mock.patch.dict(
                    dotnet._SDKS_AMD64['2.0.0'],
                    {'checksum': 'sha256/{}'.format(
                        file_utils.calculate_hash(
                            'test-sdk.tar', algorithm='sha256'))}):
            plugin = dotnet.DotNetPlugin(
                'test-part', self.options, self.project)

        with mock.patch.object(
                sources.Tar, 'download', return_value='test-sdk.tar'):
            plugin.pull()

        self.assertThat(
            os.path.join('parts', 'test-part', 'dotnet', 'sdk', 'test-sdk'),
            FileExists())

    def test_clean_pull_removes_dotnet_dir(self):
        dotnet_dir = os.path.join('parts', 'test-part', 'dotnet', 'sdk')
        os.makedirs(dotnet_dir)
        plugin = dotnet.DotNetPlugin(
            'test-part', self.options, self.project)
        plugin.clean_pull()
        self.assertThat(dotnet_dir, Not(DirExists()))

    def test_build_changes_executable_permissions(self):
        plugin = dotnet.DotNetPlugin(
            'test-part', self.options, self.project)
        _setup_dirs(plugin)
        plugin.build()

        self.assertThat(
            os.stat(os.path.join(
                plugin.installdir, 'test-app')).st_mode & 0o777,
            Equals(0o755))


class DotNetProjectBuildCommandsTestCase(DotNetProjectBaseTestCase):

    scenarios = [
        ('Debug', dict(configuration='Debug', build_attributes=['debug'])),
        ('Release', dict(configuration='Release', build_attributes=[]))]

    def test_build_commands(self):
        self.options.build_attributes = self.build_attributes
        plugin = dotnet.DotNetPlugin(
            'test-part', self.options, self.project)
        _setup_dirs(plugin)
        plugin.build()

        part_dir = os.path.join(self.path, 'parts', 'test-part')
        dotnet_command = os.path.join(part_dir, 'dotnet', 'sdk', 'dotnet')
        self.assertThat(
            self.mock_check_call.mock_calls, Equals([
                mock.call([
                    mock.ANY, mock.ANY, dotnet_command,
                    'build', '-c', self.configuration], cwd=mock.ANY),
                mock.call([
                    mock.ANY, mock.ANY, dotnet_command,
                    'publish', '-c', self.configuration,
                    '-o', plugin.installdir,
                    '--self-contained', '-r', 'linux-x64'], cwd=mock.ANY)]))
