# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import ruby
from tests import unit


class RubyPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options(snapcraft.ProjectOptions):
            source = "."
            ruby_version = "2.4.2"
            gems = []
            use_bundler = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        schema = ruby.RubyPlugin.schema()
        expected_use_bundler = {"type": "boolean", "default": False}
        expected_ruby_version = {"type": "string", "default": "2.4.2"}
        expected_gems = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        self.assertThat(schema["properties"], HasLength(3))
        self.assertDictEqual(expected_use_bundler, schema["properties"]["use-bundler"])
        self.assertDictEqual(
            expected_ruby_version, schema["properties"]["ruby-version"]
        )
        self.assertDictEqual(expected_gems, schema["properties"]["gems"])

    def test_get_pull_properties(self):
        expected_pull_properties = ["ruby-version", "gems", "use-bundler"]
        resulting_pull_properties = ruby.RubyPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_snap_fileset(self):
        expected_fileset = ["-include/", "-share/"]

        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        fileset = plugin.snap_fileset()

        self.assertThat(fileset, Equals(expected_fileset))

    def test_env_without_ruby(self):
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        env = plugin.env("dummy-path")
        self.assertThat(env, Equals([]))

    def test_env_with_ruby(self):
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        part_dir = os.path.join(self.path, "test-part-path")

        os.makedirs(os.path.join(part_dir, "lib", "ruby", "gems", "test-version"))
        libdir = os.path.join("test-part-path", "lib", "ruby", "test-version")
        arch_libdir = os.path.join(libdir, "foo-linux-bar")
        real_arch_libdir = os.path.join(self.path, arch_libdir)
        os.makedirs(real_arch_libdir)
        open(os.path.join(real_arch_libdir, "rbconfig.rb"), "w").close()
        env = plugin.env("test-part-path")

        expected_env = {
            'GEM_HOME="test-part-path/lib/ruby/gems/test-version"',
            'RUBYLIB="{}:{}"'.format(libdir, arch_libdir),
            'GEM_PATH="test-part-path/lib/ruby/gems/test-version"',
        }
        self.assertThat(set(env), Equals(expected_env))

    def test_env_with_multiple_ruby(self):
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        part_dir = os.path.join(self.path, "test-part-path")

        os.makedirs(os.path.join(part_dir, "lib", "ruby", "gems", "test-version1"))
        os.makedirs(os.path.join(part_dir, "lib", "ruby", "gems", "test-version2"))

        error = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            plugin.env,
            "test-part-path",
        )

        self.assertThat(
            str(error), Equals("Expected a single Ruby version, but found 2")
        )

    def test_env_with_rbconfigs(self):
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        part_dir = os.path.join(self.path, "test-part-path")

        os.makedirs(os.path.join(part_dir, "lib", "ruby", "gems", "test-version"))
        libdir = os.path.join("test-part-path", "lib", "ruby", "test-version")

        for arch in ("foo-linux-bar1", "foo-linux-bar2"):
            arch_libdir1 = os.path.join(libdir, arch)
            real_arch_libdir1 = os.path.join(self.path, arch_libdir1)
            os.makedirs(real_arch_libdir1)
            open(os.path.join(real_arch_libdir1, "rbconfig.rb"), "w").close()

        error = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            plugin.env,
            "test-part-path",
        )

        self.assertThat(
            str(error), Equals("Expected a single rbconfig.rb, but found 2")
        )

    def test_pull_downloads_ruby(self):
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)

        self.assertThat(
            plugin._ruby_tar.source,
            Equals("https://cache.ruby-lang.org/pub/ruby/ruby-2.4.2.tar.gz"),
        )
        self.assertThat(
            plugin._ruby_tar.source_dir,
            Equals(os.path.join(self.path, "parts", "test-part", "ruby")),
        )

        with mock.patch.multiple(
            plugin, _ruby_install=mock.DEFAULT, _gem_install=mock.DEFAULT
        ):
            # We don't want to wait for the download to finish. It is already
            # tested in the Tar object tests.
            with mock.patch.object(plugin._ruby_tar, "download") as mock_download:
                plugin.pull()

        mock_download.assert_called_once_with()

    def test_pull_installs_ruby(self):
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        with mock.patch.multiple(
            plugin, _ruby_tar=mock.DEFAULT, _gem_install=mock.DEFAULT
        ) as mocks:
            with mock.patch("snapcraft.internal.common.run") as mock_run:
                plugin.pull()

        ruby_expected_dir = os.path.join(self.path, "parts", "test-part", "ruby")
        mocks["_ruby_tar"].provision.assert_called_with(
            ruby_expected_dir, clean_target=False, keep_tarball=True
        )
        mock_run.assert_has_calls(
            [
                mock.call(
                    ["./configure", "--disable-install-rdoc", "--prefix=/"],
                    cwd=ruby_expected_dir,
                    env=mock.ANY,
                ),
                mock.call(
                    ["make", "-j{}".format(plugin.parallel_build_count)],
                    cwd=ruby_expected_dir,
                    env=mock.ANY,
                ),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    cwd=ruby_expected_dir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_pull_installs_gems_without_bundler(self):
        self.options.gems = ["test-gem-1", "test-gem-2"]
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        with mock.patch.multiple(
            plugin, _ruby_tar=mock.DEFAULT, _ruby_install=mock.DEFAULT
        ):
            with mock.patch("snapcraft.internal.common.run") as mock_run:
                plugin.pull()

        test_part_dir = os.path.join(self.path, "parts", "test-part")
        mock_run.assert_called_with(
            [
                os.path.join(test_part_dir, "install", "bin", "ruby"),
                os.path.join(test_part_dir, "install", "bin", "gem"),
                "install",
                "--env-shebang",
                "test-gem-1",
                "test-gem-2",
            ],
            cwd=os.path.join(test_part_dir, "build"),
            env=mock.ANY,
        )

    def test_pull_with_bundler(self):
        self.options.gems = ["test-gem-1", "test-gem-2"]
        plugin = ruby.RubyPlugin("test-part", self.options, self.project_options)
        self.options.use_bundler = True

        with mock.patch.multiple(
            plugin, _ruby_tar=mock.DEFAULT, _ruby_install=mock.DEFAULT
        ):
            with mock.patch("snapcraft.internal.common.run") as mock_run:
                plugin.pull()
        test_part_dir = os.path.join(self.path, "parts", "test-part")
        mock_run.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(test_part_dir, "install", "bin", "ruby"),
                        os.path.join(test_part_dir, "install", "bin", "gem"),
                        "install",
                        "--env-shebang",
                        "test-gem-1",
                        "test-gem-2",
                        "bundler",
                    ],
                    cwd=os.path.join(test_part_dir, "build"),
                    env=mock.ANY,
                ),
                mock.call(
                    [
                        os.path.join(test_part_dir, "install", "bin", "ruby"),
                        os.path.join(test_part_dir, "install", "bin", "bundle"),
                        "install",
                    ],
                    cwd=os.path.join(test_part_dir, "build"),
                    env=mock.ANY,
                ),
            ]
        )
