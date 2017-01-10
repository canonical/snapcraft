# Copyright (C) 2016, 2017 Marius Gripsgard (mariogrip@ubuntu.com)
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
from snapcraft.plugins import rust


class RustPluginTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            makefile = None
            make_parameters = []
            rust_features = []

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
        self.assertEqual(rust_channel_type, 'string',
                         'Expected "rust-channel" "type" to be "string", '
                         'but it was "{}"'.format(rust_channel_type))

        rust_revision = properties['rust-revision']
        self.assertTrue('type' in rust_revision,
                        'Expected "type" to be included in "rust-revision"')

        rust_revision_type = rust_revision['type']
        self.assertEqual(rust_revision_type, 'string',
                         'Expected "rust-revision" "type" to be "string", '
                         'but it was "{}"'.format(rust_revision_type))

    @mock.patch.object(rust.RustPlugin, 'run')
    def test_build(self, run_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertEqual(1, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(
                [plugin._cargo, 'install',
                 '-j{}'.format(plugin.project.parallel_build_count),
                 '--root', plugin.installdir],
                env=plugin._build_env())
        ])

    @mock.patch.object(rust.RustPlugin, 'run')
    def test_build_with_conditional_compilation(self, run_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        plugin.options.rust_features = ['conditional-compilation']
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertEqual(1, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(
                [plugin._cargo, 'install',
                 '-j{}'.format(plugin.project.parallel_build_count),
                 '--root', plugin.installdir,
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

        self.assertEqual(2, run_mock.call_count)

        rustdir = os.path.join(plugin.partdir, 'rust')
        run_mock.assert_has_calls([mock.call([
            os.path.join(rustdir, 'rustup.sh'), '--prefix={}'.format(rustdir),
            '--disable-sudo', '--save']),
            mock.call([plugin._cargo, "fetch"])])

    @mock.patch.object(rust.sources, 'Script')
    @mock.patch.object(rust.RustPlugin, 'run')
    def test_pull_with_channel(self, run_mock, script_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.options.rust_revision = ''
        plugin.options.rust_channel = 'nightly'

        plugin.pull()

        self.assertEqual(2, run_mock.call_count)

        rustdir = os.path.join(plugin.partdir, 'rust')
        run_mock.assert_has_calls([mock.call([
            os.path.join(rustdir, 'rustup.sh'), '--prefix={}'.format(rustdir),
            '--disable-sudo', '--save', '--channel=nightly']),
            mock.call([plugin._cargo, "fetch"])])

    @mock.patch.object(rust.sources, 'Script')
    @mock.patch.object(rust.RustPlugin, 'run')
    def test_pull_with_revision(self, run_mock, script_mock):
        plugin = rust.RustPlugin('test-part', self.options,
                                 self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.options.rust_revision = '1.13.0'
        plugin.options.rust_channel = ''

        plugin.pull()

        self.assertEqual(2, run_mock.call_count)

        rustdir = os.path.join(plugin.partdir, 'rust')
        run_mock.assert_has_calls([mock.call([
            os.path.join(rustdir, 'rustup.sh'), '--prefix={}'.format(rustdir),
            '--disable-sudo', '--save', '--revision=1.13.0']),
            mock.call([plugin._cargo, "fetch"])])
