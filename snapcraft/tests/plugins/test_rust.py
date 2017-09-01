# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Marius Gripsgard (mariogrip@ubuntu.com)
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
from testtools.matchers import DirExists, Equals, FileExists, Not

import snapcraft
from snapcraft import tests
from snapcraft.plugins import rust


class RustPluginCrossCompileTestCase(tests.TestCase):

    scenarios = [
        ('armv7l', dict(deb_arch='armhf',
                        target='armv7-unknown-linux-gnueabihf')),
        ('aarch64', dict(deb_arch='arm64',
                         target='aarch64-unknown-linux-gnu')),
        ('i386', dict(deb_arch='i386',
                      target='i686-unknown-linux-gnu')),
        ('x86_64', dict(deb_arch='amd64',
                        target='x86_64-unknown-linux-gnu')),
        ('ppc64le', dict(deb_arch='ppc64el',
                         target='powerpc64le-unknown-linux-gnu')),
    ]

    def setUp(self):
        super().setUp()

        class Options:
            makefile = None
            make_parameters = []
            rust_features = []
            rust_revision = ''
            rust_channel = ''
            source_subdir = ''

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions(
            target_deb_arch=self.deb_arch)

        patcher = mock.patch('snapcraft.internal.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.ProjectOptions.is_cross_compiling')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.dict(os.environ, {})
        self.env_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch('snapcraft.internal.sources._script.Script.download')
    def test_cross_compile(self, mock_download):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.enable_cross_compilation()
        self.assertThat(plugin._target, Equals(self.target))

        plugin.pull()
        mock_download.assert_called_once_with()
        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls([
            mock.call(
                [plugin._rustup,
                 '--prefix={}'.format(os.path.join(plugin._rustpath)),
                 '--disable-sudo',
                 '--save',
                 '--with-target={}'.format(self.target)],
                cwd=os.path.join(plugin.partdir, 'build')),
            mock.call(
                [plugin._cargo, 'fetch',
                 '--manifest-path',
                 os.path.join(plugin.sourcedir, 'Cargo.toml')],
                cwd=os.path.join(plugin.partdir, 'build')),
        ])

        plugin.build()
        self.assertThat(os.path.join(plugin._cargo_dir, 'config'),
                        FileExists())

        self.assertThat(self.run_mock.call_count, Equals(3))
        self.run_mock.assert_has_calls([
            mock.call(
                [plugin._cargo, 'install',
                 '-j{}'.format(plugin.project.parallel_build_count),
                 '--root', plugin.installdir,
                 '--path', plugin.builddir],
                cwd=os.path.join(plugin.partdir, 'build'),
                env=plugin._build_env())
        ])

        plugin.clean_build()
        self.assertThat(plugin._cargo_dir, Not(DirExists()))
        plugin.clean_pull()
        self.assertThat(plugin._rustpath, Not(DirExists()))
        # Cleaning again shouldn't raise an exception
        plugin.clean_build()
        plugin.clean_pull()


class RustPluginTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch.dict(os.environ, {})
        self.env_mock = patcher.start()
        self.addCleanup(patcher.stop)

        class Options:
            makefile = None
            make_parameters = []
            rust_features = []
            rust_revision = ''
            rust_channel = ''
            source_subdir = ''

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        schema = rust.RustPlugin.schema()

        properties = schema['properties']
        self.assertTrue('rust-channel' in properties,
                        'Expected "rust-channel" to be included in properties')
        self.assertTrue('rust-revision' in properties,
                        'Expected "rust-revision to be included in properties')

        rust_channel = properties['rust-channel']
        self.assertTrue('type' in rust_channel,
                        'Expected "type" to be included in "rust-channel"')

        rust_channel_type = rust_channel['type']
        self.assertThat(rust_channel_type, Equals('string'),
                        'Expected "rust-channel" "type" to be "string", '
                        'but it was "{}"'.format(rust_channel_type))

        rust_revision = properties['rust-revision']
        self.assertTrue('type' in rust_revision,
                        'Expected "type" to be included in "rust-revision"')

        rust_revision_type = rust_revision['type']
        self.assertThat(rust_revision_type, Equals('string'),
                        'Expected "rust-revision" "type" to be "string", '
                        'but it was "{}"'.format(rust_revision_type))

    @mock.patch.object(rust.RustPlugin, 'run')
    def test_build(self, run_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(1))
        run_mock.assert_has_calls([
            mock.call(
                [plugin._cargo, 'install',
                 '-j{}'.format(plugin.project.parallel_build_count),
                 '--root', plugin.installdir,
                 '--path', plugin.builddir],
                env=plugin._build_env())
        ])

    @mock.patch.object(rust.RustPlugin, 'run')
    def test_build_with_conditional_compilation(self, run_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        plugin.options.rust_features = ['conditional-compilation']
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(1))
        run_mock.assert_has_calls([
            mock.call(
                [plugin._cargo, 'install',
                 '-j{}'.format(plugin.project.parallel_build_count),
                 '--root', plugin.installdir,
                 '--path', plugin.builddir,
                 '--features', 'conditional-compilation'],
                env=plugin._build_env())
        ])

    @mock.patch.object(rust.sources, 'Script')
    @mock.patch.object(rust.RustPlugin, 'run')
    def test_pull(self, run_mock, script_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.options.rust_revision = []
        plugin.options.rust_channel = []

        plugin.pull()

        self.assertThat(run_mock.call_count, Equals(2))

        rustdir = os.path.join(plugin.partdir, 'rust')
        run_mock.assert_has_calls([mock.call([
            os.path.join(rustdir, 'rustup.sh'), '--prefix={}'.format(rustdir),
            '--disable-sudo', '--save']),
            mock.call([plugin._cargo, 'fetch',
                       '--manifest-path',
                       os.path.join(plugin.sourcedir, 'Cargo.toml')])])

    @mock.patch.object(rust.sources, 'Script')
    @mock.patch.object(rust.RustPlugin, 'run')
    def test_pull_with_channel(self, run_mock, script_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.options.rust_revision = ''
        plugin.options.rust_channel = 'nightly'

        plugin.pull()

        self.assertThat(run_mock.call_count, Equals(2))

        rustdir = os.path.join(plugin.partdir, 'rust')
        run_mock.assert_has_calls([mock.call([
            os.path.join(rustdir, 'rustup.sh'), '--prefix={}'.format(rustdir),
            '--disable-sudo', '--save',
            '--channel=nightly']),
            mock.call([plugin._cargo, 'fetch',
                       '--manifest-path',
                       os.path.join(plugin.sourcedir, 'Cargo.toml')])])

    @mock.patch.object(rust.sources, 'Script')
    @mock.patch.object(rust.RustPlugin, 'run')
    def test_pull_with_revision(self, run_mock, script_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.options.rust_revision = '1.13.0'
        plugin.options.rust_channel = ''

        plugin.pull()

        self.assertThat(run_mock.call_count, Equals(2))

        rustdir = os.path.join(plugin.partdir, 'rust')
        run_mock.assert_has_calls([mock.call([
            os.path.join(rustdir, 'rustup.sh'), '--prefix={}'.format(rustdir),
            '--disable-sudo', '--save',
            '--revision=1.13.0']),
            mock.call([
                plugin._cargo, 'fetch',
                '--manifest-path', os.path.join(plugin.sourcedir, 'Cargo.toml')
            ])])

    @mock.patch.object(rust.sources, 'Script')
    @mock.patch.object(rust.RustPlugin, 'run')
    def test_pull_with_source_and_source_subdir(self, run_mock, script_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.options.source_subdir = 'test-subdir'

        plugin.pull()

        run_mock.assert_has_calls([
            mock.ANY,
            mock.call([
                plugin._cargo, 'fetch',
                '--manifest-path',
                os.path.join(plugin.sourcedir, 'test-subdir', 'Cargo.toml')
            ])])

    @mock.patch('snapcraft.ProjectOptions.deb_arch', 'fantasy-arch')
    def test_cross_compiling_unsupported_arch_raises_exception(self):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)

        self.assertRaises(NotImplementedError, plugin.enable_cross_compilation)
