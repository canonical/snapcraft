# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018, 2020 Canonical Ltd
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

from snapcraft.internal import errors
from snapcraft.plugins.v1 import plainbox_provider
from tests import fixture_setup, unit

from . import PluginsV1BaseTestCase


class PlainboxProviderPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        """Test validity of the Scons Plugin schema"""
        schema = plainbox_provider.PlainboxProviderPlugin.schema()

        # Verify the presence of all properties
        properties = schema["properties"]
        self.assertThat(properties, Equals({}))

    def test_get_pull_properties(self):
        expected_pull_properties = []
        resulting_pull_properties = (
            plainbox_provider.PlainboxProviderPlugin.get_pull_properties()
        )

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = []
        resulting_build_properties = (
            plainbox_provider.PlainboxProviderPlugin.get_build_properties()
        )

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


class PlainboxProviderPluginTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class Options:
            source = "."

        self.options = Options()

        patcher = mock.patch.object(plainbox_provider.PlainboxProviderPlugin, "run")
        self.mock_run = patcher.start()
        self.addCleanup(patcher.stop)

    def test_build(self):
        plugin = plainbox_provider.PlainboxProviderPlugin(
            "test-part", self.options, self.project
        )

        os.makedirs(plugin.sourcedir)

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                "path": os.path.join(plugin.installdir, "baz"),
                "contents": "#!/foo/bar/baz/python3",
                "expected": "#!/usr/bin/env python3",
            },
            {
                "path": os.path.join(plugin.installdir, "bin", "foobar"),
                "contents": "#!/foo/baz/python3.5",
                "expected": "#!/usr/bin/env python3.5",
            },
            {
                "path": os.path.join(plugin.installdir, "foo"),
                "contents": "foo",
                "expected": "foo",
            },
            {
                "path": os.path.join(plugin.installdir, "bar"),
                "contents": "bar\n#!/usr/bin/python3",
                "expected": "bar\n#!/usr/bin/python3",
            },
        ]

        for file_info in files:
            os.makedirs(os.path.dirname(file_info["path"]), exist_ok=True)
            with open(file_info["path"], "w") as f:
                f.write(file_info["contents"])

        plugin.build()

        env = os.environ.copy()
        env["PROVIDERPATH"] = ""
        calls = [
            mock.call(["python3", "manage.py", "validate"], env=env),
            mock.call(["python3", "manage.py", "build"]),
            mock.call(["python3", "manage.py", "i18n"]),
            mock.call(
                [
                    "python3",
                    "manage.py",
                    "install",
                    "--layout=relocatable",
                    "--prefix=/providers/test-part",
                    "--root={}".format(plugin.installdir),
                ]
            ),
        ]
        self.mock_run.assert_has_calls(calls)

        for file_info in files:
            with open(os.path.join(plugin.installdir, file_info["path"]), "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    def test_build_with_provider_stage_dir(self):
        self.useFixture(fixture_setup.CleanEnvironment())

        plugin = plainbox_provider.PlainboxProviderPlugin(
            "test-part", self.options, self.project
        )

        os.makedirs(plugin.sourcedir)
        provider_path = os.path.join(
            self.project.stage_dir, "providers", "test-provider"
        )
        os.makedirs(provider_path)

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                "path": os.path.join(plugin.installdir, "baz"),
                "contents": "#!/foo/bar/baz/python3",
                "expected": "#!/usr/bin/env python3",
            },
            {
                "path": os.path.join(plugin.installdir, "bin", "foobar"),
                "contents": "#!/foo/baz/python3.5",
                "expected": "#!/usr/bin/env python3.5",
            },
            {
                "path": os.path.join(plugin.installdir, "foo"),
                "contents": "foo",
                "expected": "foo",
            },
            {
                "path": os.path.join(plugin.installdir, "bar"),
                "contents": "bar\n#!/usr/bin/python3",
                "expected": "bar\n#!/usr/bin/python3",
            },
        ]

        for file_info in files:
            os.makedirs(os.path.dirname(file_info["path"]), exist_ok=True)
            with open(file_info["path"], "w") as f:
                f.write(file_info["contents"])

        plugin.build()

        calls = [
            mock.call(
                ["python3", "manage.py", "validate"],
                env={"PROVIDERPATH": provider_path},
            ),
            mock.call(["python3", "manage.py", "build"]),
            mock.call(["python3", "manage.py", "i18n"]),
            mock.call(
                [
                    "python3",
                    "manage.py",
                    "install",
                    "--layout=relocatable",
                    "--prefix=/providers/test-part",
                    "--root={}".format(plugin.installdir),
                ]
            ),
        ]
        self.mock_run.assert_has_calls(calls)

        for file_info in files:
            with open(os.path.join(plugin.installdir, file_info["path"]), "r") as f:
                self.assertThat(f.read(), Equals(file_info["expected"]))

    def test_fileset_ignores(self):
        plugin = plainbox_provider.PlainboxProviderPlugin(
            "test-part", self.options, self.project
        )

        expected_fileset = [
            "-usr/lib/python*/sitecustomize.py",
            "-etc/python*/sitecustomize.py",
        ]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)


class PlainboxProviderPluginUnsupportedBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            source = "dir"

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            plainbox_provider.PlainboxProviderPlugin,
            "test-part",
            self.options,
            self.project,
        )
