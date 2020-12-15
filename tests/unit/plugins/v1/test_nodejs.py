# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017-2020 Canonical Ltd
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
import json
import os
import pathlib
import tarfile
from unittest import mock

import fixtures
import pytest
from testtools.matchers import Equals, FileExists, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import nodejs
from tests import fixture_setup, unit

from . import PluginsV1BaseTestCase


class NodePluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class Options:
            source = "."
            nodejs_version = nodejs._NODEJS_VERSION
            nodejs_package_manager = "npm"
            nodejs_yarn_version = ""
            source = "."

        self.options = Options()

        # always have a package.json stub under source
        open("package.json", "w").close()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.internal.common.run_output")
        self.run_output_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.run_output_mock.return_value = '{"dependencies": []}'

        patcher = mock.patch("snapcraft.sources.Tar")
        self.tar_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.nodejs_url = nodejs.get_nodejs_release(
            nodejs._NODEJS_VERSION, self.project.deb_arch
        )

        self.useFixture(fixture_setup.CleanEnvironment())

    def create_assets(
        self,
        plugin,
        package_name="test-nodejs",
        single_bin=False,
        skip_package_json=False,
    ):
        for directory in (plugin.sourcedir, plugin.builddir):
            os.makedirs(directory)
            if not skip_package_json:
                with open(os.path.join(directory, "package.json"), "w") as json_file:
                    json.dump(
                        dict(
                            name=package_name, bin=dict(run="index.js"), version="1.0"
                        ),
                        json_file,
                    )
            package_name = package_name.lstrip("@").replace("/", "-")
            packed_tar_file = os.path.join(directory, "{}-1.0.tgz".format(package_name))
            open(os.path.join(directory, "index.js"), "w").close()
        tarfile.TarFile(packed_tar_file, "w").close()

        def provision(directory, **kwargs):
            if directory == os.path.join(plugin.builddir, "package"):
                package_dir = os.path.join(plugin.builddir, "package")
                open(os.path.join(package_dir, "index.js"), "w").close()

        self.tar_mock().provision.side_effect = provision

        # Create a fake node bin
        os.makedirs(os.path.join(plugin._npm_dir, "bin"))
        open(os.path.join(plugin._npm_dir, "bin", "node"), "w").close()


class NodejsPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = nodejs.NodePlugin.schema()
        properties = schema["properties"]

        # Check nodejs-version
        self.assertTrue(
            "nodejs-version" in properties,
            'Expected "nodejs-version" to be included in ' "properties",
        )
        node_channel = properties["nodejs-version"]

        self.assertTrue(
            "type" in node_channel, 'Expected "type" to be included in "nodejs-version"'
        )
        self.assertThat(
            node_channel["type"],
            Equals("string"),
            'Expected "nodejs-version" "type" to be '
            '"string", but it was '
            '"{}"'.format(node_channel["type"]),
        )
        self.assertTrue(
            "default" in node_channel,
            'Expected "default" to be included in "nodejs-version"',
        )
        self.assertThat(
            node_channel["default"],
            Equals(nodejs._NODEJS_VERSION),
            'Expected "nodejs-version" "default" to be '
            '"{}", but it was '
            '"{}"'.format(nodejs._NODEJS_VERSION, node_channel["default"]),
        )

        # Check nodejs-yarn-version
        self.assertTrue(
            "nodejs-yarn-version" in properties,
            'Expected "nodejs-yarn-version" to be included in ' "properties",
        )
        node_channel = properties["nodejs-yarn-version"]

        self.assertTrue(
            "type" in node_channel,
            'Expected "type" to be included in "nodejs-yarn-version"',
        )
        self.assertThat(
            node_channel["type"],
            Equals("string"),
            'Expected "nodejs-yarn-version" "type" to be '
            '"string", but it was '
            '"{}"'.format(node_channel["type"]),
        )
        self.assertTrue(
            "default" in node_channel,
            'Expected "default" to be included in "nodejs-yarn-version"',
        )
        self.assertThat(
            node_channel["default"],
            Equals(""),
            'Expected "nodejs-yarn-version" "default" to be '
            '"", but it was '
            '"{}"'.format(node_channel["default"]),
        )

        # Check nodejs-package-manager
        self.assertTrue(
            "nodejs-package-manager" in properties,
            'Expected "nodejs-package-manager" to be included in ' "properties",
        )
        nodejs_package_manager = properties["nodejs-package-manager"]

        self.assertTrue(
            "type" in nodejs_package_manager,
            'Expected "type" to be included in "nodejs-package-manager"',
        )
        self.assertThat(
            nodejs_package_manager["type"],
            Equals("string"),
            'Expected "nodejs-package-manager" "type" to be '
            '"string", but it was '
            '"{}"'.format(nodejs_package_manager["type"]),
        )
        self.assertTrue(
            "default" in nodejs_package_manager,
            'Expected "default" to be included in "nodejs-package-manager"',
        )
        self.assertThat(
            nodejs_package_manager["default"],
            Equals("yarn"),
            'Expected "nodejs-package-manager" "default" to be '
            '"yarn", but it was '
            '"{}"'.format(nodejs_package_manager["default"]),
        )

    def test_get_pull_properties(self):
        expected_pull_properties = [
            "nodejs-version",
            "nodejs-package-manager",
            "nodejs-yarn-version",
        ]
        resulting_pull_properties = nodejs.NodePlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = []
        resulting_build_properties = nodejs.NodePlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


class NodePluginTest(NodePluginBaseTest):
    def get_npm_cmd(self, plugin):
        return os.path.join(plugin._npm_dir, "bin", "npm")

    def get_yarn_cmd(self, plugin):
        return os.path.join(plugin._npm_dir, "bin", "yarn")

    def test_pull_with_npm(self):
        self.options.nodejs_package_manager = "npm"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.pull()

        expected_tar_calls = [
            mock.call(self.nodejs_url, plugin._npm_dir),
            mock.call().download(),
            mock.call().provision(
                plugin._npm_dir, clean_target=False, keep_tarball=True
            ),
        ]

        self.tar_mock.assert_has_calls(expected_tar_calls)

    def test_pull_with_yarn(self):
        self.options.nodejs_package_manager = "yarn"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.pull()

        expected_tar_calls = [
            mock.call(self.nodejs_url, plugin._npm_dir),
            mock.call().download(),
            mock.call("https://yarnpkg.com/latest.tar.gz", plugin._npm_dir),
            mock.call().download(),
            mock.call().provision(
                plugin._npm_dir, clean_target=False, keep_tarball=True
            ),
            mock.call().provision(
                plugin._npm_dir, clean_target=False, keep_tarball=True
            ),
        ]

        self.tar_mock.assert_has_calls(expected_tar_calls)

    def test_build_with_npm(self):
        self.options.nodejs_package_manager = "npm"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.build()

        self.assertThat(os.path.join(plugin.installdir, "bin", "run"), FileExists())

        expected_env = dict(PATH=os.path.join(plugin._npm_dir, "bin"))

        expected_run_calls = [
            mock.call(
                [self.get_npm_cmd(plugin), "install", "--unsafe-perm"],
                cwd=plugin.builddir,
                env=expected_env,
            ),
            mock.call(
                [self.get_npm_cmd(plugin), "pack"],
                cwd=plugin.builddir,
                env=expected_env,
            ),
            mock.call(
                [
                    self.get_npm_cmd(plugin),
                    "install",
                    "--unsafe-perm",
                    "--offline",
                    "--prod",
                ],
                cwd=os.path.join(plugin.builddir, "package"),
                env=expected_env,
            ),
        ]
        expected_tar_calls = [
            mock.call("test-nodejs-1.0.tgz", plugin.builddir),
            mock.call().provision(os.path.join(plugin.builddir, "package")),
        ]

        self.run_mock.assert_has_calls(expected_run_calls)
        self.tar_mock.assert_has_calls(expected_tar_calls)

        expected_tar_calls = [
            mock.call("test-nodejs-1.0.tgz", plugin.builddir),
            mock.call().provision(os.path.join(plugin.builddir, "package")),
        ]
        self.tar_mock.assert_has_calls(expected_tar_calls)

    def test_build_with_yarn(self):
        self.options.nodejs_package_manager = "yarn"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.build()

        self.assertThat(os.path.join(plugin.installdir, "bin", "run"), FileExists())

        expected_env = dict(PATH=os.path.join(plugin._npm_dir, "bin"))
        cmd = [self.get_yarn_cmd(plugin)]
        expected_run_calls = [
            mock.call(cmd + ["install"], cwd=plugin.builddir, env=expected_env),
            mock.call(
                cmd + ["pack", "--filename", "test-nodejs-1.0.tgz"],
                cwd=plugin.builddir,
                env=expected_env,
            ),
            mock.call(
                cmd + ["install", "--offline", "--prod"],
                cwd=os.path.join(plugin.builddir, "package"),
                env=expected_env,
            ),
        ]
        expected_tar_calls = [
            mock.call("test-nodejs-1.0.tgz", plugin.builddir),
            mock.call().provision(os.path.join(plugin.builddir, "package")),
        ]

        self.run_mock.assert_has_calls(expected_run_calls)
        self.tar_mock.assert_has_calls(expected_tar_calls)

        expected_tar_calls = [
            mock.call("test-nodejs-1.0.tgz", plugin.builddir),
            mock.call().provision(os.path.join(plugin.builddir, "package")),
        ]
        self.tar_mock.assert_has_calls(expected_tar_calls)

    def test_build_yarn_with_proxy(self):
        self.useFixture(
            fixtures.EnvironmentVariable("http_proxy", "http://localhost:3132")
        )
        self.useFixture(
            fixtures.EnvironmentVariable("https_proxy", "http://localhost:3133")
        )
        self.options.nodejs_package_manager = "yarn"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        plugin.build()

        self.assertThat(os.path.join(plugin.installdir, "bin", "run"), FileExists())

        cmd = [
            self.get_yarn_cmd(plugin),
            "--proxy",
            "http://localhost:3132",
            "--https-proxy",
            "http://localhost:3133",
        ]

        expected_run_calls = [
            mock.call(cmd + ["install"], cwd=plugin.builddir, env=mock.ANY),
            mock.call(
                cmd + ["pack", "--filename", "test-nodejs-1.0.tgz"],
                cwd=plugin.builddir,
                env=mock.ANY,
            ),
            mock.call(
                cmd + ["install", "--offline", "--prod"],
                cwd=os.path.join(plugin.builddir, "package"),
                env=mock.ANY,
            ),
        ]

        self.run_mock.assert_has_calls(expected_run_calls)

    def test_build_scoped_name_with_npm(self):
        self.options.nodejs_package_manager = "npm"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin, package_name="@org/name")

        plugin.build()

        expected_run_calls = [
            mock.call(
                [self.get_npm_cmd(plugin), "install", "--unsafe-perm"],
                cwd=os.path.join(plugin.builddir),
                env=mock.ANY,
            ),
            mock.call(
                [self.get_npm_cmd(plugin), "pack"], cwd=plugin.builddir, env=mock.ANY
            ),
            mock.call(
                [
                    self.get_npm_cmd(plugin),
                    "install",
                    "--unsafe-perm",
                    "--offline",
                    "--prod",
                ],
                cwd=os.path.join(plugin.builddir, "package"),
                env=mock.ANY,
            ),
        ]

        self.run_mock.assert_has_calls(expected_run_calls)

        expected_tar_calls = [
            mock.call("org-name-1.0.tgz", plugin.builddir),
            mock.call().provision(os.path.join(plugin.builddir, "package")),
        ]
        self.tar_mock.assert_has_calls(expected_tar_calls)

    def test_build_scoped_name_with_yarn(self):
        self.options.nodejs_package_manager = "yarn"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin, package_name="@org/name")

        plugin.build()

        cmd = [self.get_yarn_cmd(plugin)]
        expected_run_calls = [
            mock.call(cmd + ["install"], cwd=plugin.builddir, env=mock.ANY),
            mock.call(
                cmd + ["pack", "--filename", "org-name-1.0.tgz"],
                cwd=plugin.builddir,
                env=mock.ANY,
            ),
            mock.call(
                cmd + ["install", "--offline", "--prod"],
                cwd=os.path.join(plugin.builddir, "package"),
                env=mock.ANY,
            ),
        ]

        self.run_mock.assert_has_calls(expected_run_calls)

        expected_tar_calls = [
            mock.call("org-name-1.0.tgz", plugin.builddir),
            mock.call().provision(os.path.join(plugin.builddir, "package")),
        ]
        self.tar_mock.assert_has_calls(expected_tar_calls)


@pytest.fixture(params=["npm", "yarn"])
def nodejs_plugin(project, request):
    """Return an instance of NodejsPlugin setup varying bases and pkg managers."""

    class Options:
        source = "."
        nodejs_version = nodejs._NODEJS_VERSION
        nodejs_package_manager = request.param
        nodejs_yarn_version = ""
        source = "."

    return nodejs.NodePlugin("test-part", Options(), project)


def _create_assets(nodejs_plugin, mock_tar, package_name="test-nodejs"):
    for directory in (nodejs_plugin.sourcedir, nodejs_plugin.builddir):
        directory_path = pathlib.Path(directory)
        directory_path.mkdir(parents=True)

        package_json_path = directory_path / "package.json"
        with package_json_path.open("w") as json_file:
            json.dump(
                dict(name=package_name, bin=dict(run="node.js"), version="1.0"),
                json_file,
            )

        package_name = package_name.lstrip("@").replace("/", "-")
        packed_tar_path = directory_path / f"{package_name}-1.0.tgz"

        (directory_path / "node.js").touch()

    tarfile.TarFile(packed_tar_path, "w").close()

    def provision(directory, **kwargs):
        if directory == os.path.join(nodejs_plugin.builddir, "package"):
            package_dir = os.path.join(nodejs_plugin.builddir, "package")
            open(os.path.join(package_dir, "node.js"), "w").close()

    # Create a fake node bin
    bin_path = pathlib.Path(nodejs_plugin._npm_dir) / "bin"
    bin_path.mkdir(parents=True)
    (bin_path / "node").touch()

    nodejs_bin_path = pathlib.Path(nodejs_plugin.installdir) / "node.js"
    nodejs_bin_path.parent.mkdir(parents=True)
    nodejs_bin_path.touch()


@pytest.fixture()
def nodejs_plugin_with_assets(mock_tar, nodejs_plugin):
    """Return an instance of NodejsPlugin with artifacts setup using nodejs_plugin."""
    _create_assets(nodejs_plugin, mock_tar)

    return nodejs_plugin


class TestManifest:
    scenarios = [
        (
            "simple",
            dict(
                ls_output=(
                    '{"dependencies": {'
                    '   "testpackage1": {"version": "1.0"},'
                    '   "testpackage2": {"version": "1.2"}}}'
                ),
                expected_dependencies=["testpackage1=1.0", "testpackage2=1.2"],
            ),
        ),
        (
            "nested",
            dict(
                ls_output=(
                    '{"dependencies": {'
                    '   "testpackage1": {'
                    '      "version": "1.0",'
                    '      "dependencies": {'
                    '        "testpackage2": {"version": "1.2"}}}}}'
                ),
                expected_dependencies=["testpackage1=1.0", "testpackage2=1.2"],
            ),
        ),
        (
            "missing",
            dict(
                ls_output=(
                    '{"dependencies": {'
                    '   "testpackage1": {"version": "1.0"},'
                    '   "testpackage2": {"version": "1.2"},'
                    '   "missing": {"noversion": "dummy"}}}'
                ),
                expected_dependencies=["testpackage1=1.0", "testpackage2=1.2"],
            ),
        ),
        ("none", dict(ls_output="{}", expected_dependencies=[])),
    ]

    def test_get_manifest_with_node_packages(
        self,
        mock_run,
        mock_run_output,
        nodejs_plugin_with_assets,
        ls_output,
        expected_dependencies,
    ):
        mock_run_output.return_value = ls_output
        plugin = nodejs_plugin_with_assets

        plugin.build()

        if plugin.options.nodejs_package_manager == "yarn":
            expected_dependencies = []
        assert plugin.get_manifest() == collections.OrderedDict(
            {"node-packages": expected_dependencies}
        )


class NodePluginYarnLockManifestTest(NodePluginBaseTest):
    def test_get_manifest_with_yarn_lock_file(self):
        self.options.nodejs_package_manager = "yarn"
        plugin = nodejs.NodePlugin("test-part", self.options, self.project)

        self.create_assets(plugin)

        with open(os.path.join(plugin.builddir, "yarn.lock"), "w") as yarn_lock_file:
            yarn_lock_file.write("test yarn lock contents")

        plugin.build()

        expected_manifest = collections.OrderedDict()
        expected_manifest["yarn-lock-contents"] = "test yarn lock contents"
        expected_manifest["node-packages"] = []

        self.assertThat(plugin.get_manifest(), Equals(expected_manifest))


class TestNodeBin:
    scenarios = [
        (
            "dict",
            dict(
                package_json=dict(
                    name="package-foo",
                    bin=dict(run1="bin0/run1bin", run2="bin1/run2bin"),
                ),
                expected_bins=["run1", "run2"],
            ),
        ),
        (
            "single",
            dict(
                package_json=dict(name="package-foo", bin="bin0/run1bin"),
                expected_bins=["package-foo"],
            ),
        ),
        (
            "single, scoped package",
            dict(
                package_json=dict(name="@org/package-foo", bin="bin1/run1bin"),
                expected_bins=["package-foo"],
            ),
        ),
    ]

    def test_bins(self, tmp_work_path, package_json, expected_bins):
        if type(package_json["bin"]) == dict:
            binaries = package_json["bin"].values()
        else:
            binaries = [package_json["bin"]]

        for binary in binaries:
            os.makedirs(os.path.dirname(binary), exist_ok=True)
            open(binary, "w").close()

        nodejs._create_bins(package_json, ".")
        binary_paths = (os.path.join("bin", b) for b in expected_bins)

        for binary in binary_paths:
            assert os.path.exists(binary)


class TestNodeRelease:
    scenarios = [
        (
            "amd64",
            dict(
                deb_arch="amd64",
                engine="4.4.4",
                expected_url=(
                    "https://nodejs.org/dist/v4.4.4/node-v4.4.4-linux-x64.tar.gz"
                ),
            ),
        ),
        (
            "i386",
            dict(
                deb_arch="i386",
                engine="4.4.4",
                expected_url=(
                    "https://nodejs.org/dist/v4.4.4/node-v4.4.4-linux-x86.tar.gz"
                ),
            ),
        ),
        (
            "armhf",
            dict(
                deb_arch="armhf",
                engine="4.4.4",
                expected_url=(
                    "https://nodejs.org/dist/v4.4.4/node-v4.4.4-linux-armv7l.tar.gz"
                ),
            ),
        ),
        (
            "aarch64",
            dict(
                deb_arch="arm64",
                engine="4.4.4",
                expected_url=(
                    "https://nodejs.org/dist/v4.4.4/node-v4.4.4-linux-arm64.tar.gz"
                ),
            ),
        ),
        (
            "s390x",
            dict(
                deb_arch="s390x",
                engine="4.4.4",
                expected_url=(
                    "https://nodejs.org/dist/v4.4.4/node-v4.4.4-linux-s390x.tar.gz"
                ),
            ),
        ),
    ]

    def test_get_nodejs_release(self, deb_arch, engine, expected_url):
        assert nodejs.get_nodejs_release(engine, deb_arch) == expected_url


class NodePluginUnsupportedArchTest(NodePluginBaseTest):
    @mock.patch("snapcraft.project.Project.deb_arch", "ppcel64")
    def test_unsupported_arch_raises_exception(self):
        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError,
            nodejs.NodePlugin,
            "test-part",
            self.options,
            self.project,
        )

        self.assertThat(
            raised.__str__(), Equals("architecture not supported (ppcel64)")
        )


def test_missing_package_json(mock_run, mock_run_output, nodejs_plugin_with_assets):
    (pathlib.Path(nodejs_plugin_with_assets.sourcedir) / "package.json").unlink()

    with pytest.raises(nodejs.NodejsPluginMissingPackageJsonError):
        nodejs_plugin_with_assets.pull()
