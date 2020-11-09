# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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

import fixtures
import pytest
from testtools.matchers import DirExists, Equals, HasLength, Not

from snapcraft.internal import errors
from snapcraft.plugins.v1 import conda
from tests import unit

from . import PluginsV1BaseTestCase


class CondaPluginBaseTest(PluginsV1BaseTestCase):

    deb_arch = None

    def setUp(self):
        super().setUp()

        if self.project.deb_arch != "amd64":
            self.skipTest("architecture is not supported by conda plugin")

        self.project._snap_meta.name = "conda-snap"

        self.fake_check_call = fixtures.MockPatch("subprocess.check_call")
        self.useFixture(self.fake_check_call)


class CondaPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = conda.CondaPlugin.schema()

        properties = schema["properties"]
        for expected in [
            "conda-packages",
            "conda-python-version",
            "conda-miniconda-version",
        ]:
            self.assertTrue(
                expected in properties,
                "Expected {!r} to be included in properties".format(expected),
            )

        # Check conda-packages
        conda_packages = properties["conda-packages"]
        for expected in ["type", "default", "minitems", "uniqueItems", "items"]:
            self.assertTrue(
                expected in conda_packages,
                "Expected {!r} to be included in 'conda-packages'".format(expected),
            )

        conda_packages_type = conda_packages["type"]
        self.assertThat(
            conda_packages_type,
            Equals("array"),
            'Expected "conda-packages" "type" to be "array", but '
            'it was "{}"'.format(conda_packages_type),
        )

        conda_packages_default = conda_packages["default"]
        self.assertThat(
            conda_packages_default,
            Equals([]),
            'Expected "conda-packages" "default" to be '
            '"d[]", but it was "{}"'.format(conda_packages_default),
        )

        conda_packages_minitems = conda_packages["minitems"]
        self.assertThat(
            conda_packages_minitems,
            Equals(1),
            'Expected "conda-packages" "minitems" to be 1, but '
            "it was {}".format(conda_packages_minitems),
        )

        self.assertTrue(conda_packages["uniqueItems"])

        conda_packages_items = conda_packages["items"]
        self.assertTrue(
            "type" in conda_packages_items,
            'Expected "type" to be included in "conda-packages" ' '"items"',
        )

        conda_packages_items_type = conda_packages_items["type"]
        self.assertThat(
            conda_packages_items_type,
            Equals("string"),
            'Expected "conda-packages" "item" "type" to be '
            '"string", but it was "{}"'.format(conda_packages_items_type),
        )

        # Check conda-python-version
        conda_channel = properties["conda-python-version"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in conda_channel,
                "Expected {!r} to be included in 'conda-python-version'".format(
                    expected
                ),
            )

        conda_channel_type = conda_channel["type"]
        self.assertThat(
            conda_channel_type,
            Equals("string"),
            'Expected "conda-python-version" "type" to be "string", but '
            'it was "{}"'.format(conda_channel_type),
        )

        conda_channel_default = conda_channel["default"]
        self.assertThat(
            conda_channel_default,
            Equals(""),
            'Expected "conda-python-version" "default" to be '
            '"latest/stable", but it was "{}"'.format(conda_channel_default),
        )

        # Check conda-miniconda-version
        conda_channel = properties["conda-miniconda-version"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in conda_channel,
                "Expected {!r} to be included in 'conda-miniconda-version'".format(
                    expected
                ),
            )

        conda_channel_type = conda_channel["type"]
        self.assertThat(
            conda_channel_type,
            Equals("string"),
            'Expected "conda-miniconda-version" "type" to be "string", but '
            'it was "{}"'.format(conda_channel_type),
        )

        conda_channel_default = conda_channel["default"]
        self.assertThat(
            conda_channel_default,
            Equals("latest"),
            'Expected "conda-miniconda-version" "default" to be '
            '"latest", but it was "{}"'.format(conda_channel_default),
        )

        # Check required properties
        self.assertIn("required", schema)
        self.assertThat(schema["required"], Equals(["conda-packages"]))

    def test_get_pull_properties(self):
        expected_pull_properties = ["conda-miniconda-version"]
        resulting_pull_properties = conda.CondaPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ["conda-packages", "conda-python-version"]
        resulting_build_properties = conda.CondaPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


@pytest.mark.parametrize(
    "miniconda_version,expected_url,expected_checksum",
    [
        (
            "latest",
            "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh",
            None,
        ),
        (
            "4.6.14",
            "https://repo.anaconda.com/miniconda/Miniconda3-4.6.14-Linux-x86_64.sh",
            "md5/718259965f234088d785cad1fbd7de03",
        ),
        (
            "4.5.14",
            "https://repo.anaconda.com/miniconda/Miniconda3-4.5.14-Linux-x86_64.sh",
            None,
        ),
    ],
)
def test_get_miniconda_source(
    project, miniconda_version, expected_url, expected_checksum
):
    class Options:
        conda_miniconda_version = miniconda_version

    plugin = conda.CondaPlugin("test-part", Options(), project)
    source_script = plugin._get_miniconda_script()

    assert source_script.source == expected_url
    assert source_script.source_checksum == expected_checksum


class CondaPluginTest(CondaPluginBaseTest):
    def test_pull(self):
        class Options:
            conda_miniconda_version = "latest"

        fake_source_script = fixtures.MockPatch("snapcraft.internal.sources.Script")
        self.useFixture(fake_source_script)

        plugin = conda.CondaPlugin("test-part", Options(), self.project)

        def download(filepath):
            self.assertThat(
                filepath, Equals(os.path.join(plugin.partdir, "miniconda.sh"))
            )
            os.makedirs(plugin.partdir)
            open(filepath, "w").close()

        fake_source_script.mock().download.side_effect = download

        plugin.pull()

        fake_source_script.mock().download.assert_called_once_with(filepath=mock.ANY)
        self.fake_check_call.mock.assert_not_called()

    def test_build_with_defaults_but_no_conda_home(self):
        class Options:
            conda_packages = ["pkg1", "pkg2"]
            conda_python_version = ""

        plugin = conda.CondaPlugin("test-part", Options(), self.project)

        plugin.build()

        self.fake_check_call.mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(plugin.partdir, "miniconda.sh"),
                        "-bfp",
                        plugin._conda_home,
                    ]
                ),
                mock.call(
                    [
                        os.path.join(plugin._conda_home, "bin", "conda"),
                        "create",
                        "--prefix",
                        plugin.installdir,
                        "--yes",
                        "pkg1",
                        "pkg2",
                    ],
                    env=dict(CONDA_TARGET_PREFIX_OVERRIDE="/snap/conda-snap/current"),
                ),
            ]
        )

    def test_build_with_defaults(self):
        class Options:
            conda_packages = ["pkg1", "pkg2"]
            conda_python_version = ""

        plugin = conda.CondaPlugin("test-part", Options(), self.project)

        # Fake _conda_home
        conda_cmd = os.path.join(plugin._conda_home, "bin", "conda")
        os.makedirs(os.path.dirname(conda_cmd))
        open(conda_cmd, "w").close()

        plugin.build()

        self.fake_check_call.mock.assert_called_once_with(
            [
                os.path.join(plugin._conda_home, "bin", "conda"),
                "create",
                "--prefix",
                plugin.installdir,
                "--yes",
                "pkg1",
                "pkg2",
            ],
            env=dict(CONDA_TARGET_PREFIX_OVERRIDE="/snap/conda-snap/current"),
        )

    def test_build_with_specific_python(self):
        class Options:
            conda_packages = ["pkg1", "pkg2"]
            conda_python_version = "3.7"

        plugin = conda.CondaPlugin("test-part", Options(), self.project)

        # Fake _conda_home
        conda_cmd = os.path.join(plugin._conda_home, "bin", "conda")
        os.makedirs(os.path.dirname(conda_cmd))
        open(conda_cmd, "w").close()

        plugin.build()

        self.fake_check_call.mock.assert_called_once_with(
            [
                os.path.join(plugin._conda_home, "bin", "conda"),
                "create",
                "--prefix",
                plugin.installdir,
                "--yes",
                "python=3.7",
                "pkg1",
                "pkg2",
            ],
            env=dict(CONDA_TARGET_PREFIX_OVERRIDE="/snap/conda-snap/current"),
        )

    def test_clean_pull(self):
        class Options:
            pass

        plugin = conda.CondaPlugin("test-part", Options(), self.project)

        os.makedirs(plugin._conda_home)

        plugin.clean_pull()

        self.assertThat(plugin._conda_home, Not(DirExists()))

    def test_clean_build(self):
        class Options:
            pass

        plugin = conda.CondaPlugin("test-part", Options(), self.project)

        os.makedirs(plugin._conda_home)

        plugin.clean_build()

        # Make sure that _conda_home is still there after clean_build
        self.assertThat(plugin._conda_home, DirExists())


class CondaPluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            pass

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            conda.CondaPlugin,
            "test-part",
            self.options,
            self.project,
        )
