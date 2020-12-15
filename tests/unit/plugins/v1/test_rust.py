# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Marius Gripsgard (mariogrip@ubuntu.com)
# Copyright (C) 2016-2020 Canonical Ltd
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
import os
import subprocess
from pathlib import Path
from unittest import mock

import pytest
import toml
from testtools.matchers import Contains, Equals, FileExists, Not

import snapcraft
from snapcraft.internal import errors, meta
from snapcraft.plugins.v1 import rust
from tests import fixture_setup, unit

from . import PluginsV1BaseTestCase


class RustPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.CleanEnvironment())

        class Options:
            makefile = None
            make_parameters = []
            rust_features = []
            rust_revision = ""
            rust_channel = ""
            source_subdir = ""

        self.options = Options()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.internal.common.run_output")
        patcher.start()
        self.addCleanup(patcher.stop)

        original_exists = os.path.exists

        def exists_mock(*args, **kwargs):
            if args[0].endswith("rustup"):
                return False
            else:
                return original_exists(args[0])

        patcher = mock.patch("os.path.exists", side_effect=exists_mock)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.plugin = rust.RustPlugin("test-part", self.options, self.project)
        os.makedirs(self.plugin.sourcedir)
        os.makedirs(self.plugin.builddir)
        open(Path(self.plugin.builddir, "Cargo.toml"), "w").write("")


class RustPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = rust.RustPlugin.schema()

        properties = schema["properties"]
        self.assertTrue(
            "rust-channel" in properties,
            'Expected "rust-channel" to be included in properties',
        )
        self.assertTrue(
            "rust-revision" in properties,
            'Expected "rust-revision to be included in properties',
        )

        # rust-channel
        rust_channel = properties["rust-channel"]
        self.assertTrue(
            "type" in rust_channel, 'Expected "type" to be included in "rust-channel"'
        )
        rust_channel_type = rust_channel["type"]
        self.assertThat(
            rust_channel_type,
            Equals("string"),
            'Expected "rust-channel" "type" to be "string", '
            'but it was "{}"'.format(rust_channel_type),
        )
        self.assertTrue(
            "enum" in rust_channel, 'Expected "enum" to be included in "rust-channel"'
        )
        rust_channel_enum = rust_channel["enum"]
        self.assertThat(
            rust_channel_enum,
            Equals(["stable", "beta", "nightly"]),
            'Expected "rust-channel" "enum" to be "["stable", "beta", "nightly"]", '
            'but it was "{}"'.format(rust_channel_enum),
        )

        # rust-revision
        rust_revision = properties["rust-revision"]
        self.assertTrue(
            "type" in rust_revision, 'Expected "type" to be included in "rust-revision"'
        )
        rust_revision_type = rust_revision["type"]
        self.assertThat(
            rust_revision_type,
            Equals("string"),
            'Expected "rust-revision" "type" to be "string", '
            'but it was "{}"'.format(rust_revision_type),
        )


@pytest.fixture
def options():
    class Options:
        makefile = None
        make_parameters = []
        rust_features = []
        rust_revision = ""
        rust_channel = "stable"
        source_subdir = ""

    return Options()


class TestRustPluginCrossCompile:

    scenarios = [
        ("armv7l", dict(deb_arch="armhf", target="armv7-unknown-linux-gnueabihf")),
        ("aarch64", dict(deb_arch="arm64", target="aarch64-unknown-linux-gnu")),
        ("i386", dict(deb_arch="i386", target="i686-unknown-linux-gnu")),
        ("x86_64", dict(deb_arch="amd64", target="x86_64-unknown-linux-gnu")),
        ("ppc64le", dict(deb_arch="ppc64el", target="powerpc64le-unknown-linux-gnu")),
        ("s390x", dict(deb_arch="s390x", target="s390x-unknown-linux-gnu")),
    ]

    @mock.patch("snapcraft.internal.sources._script.Script.download")
    def test_cross_compile(
        self,
        mock_download,
        monkeypatch,
        tmp_work_path,
        mock_run,
        mock_run_output,
        options,
        deb_arch,
        target,
    ):
        monkeypatch.setattr(snapcraft.project.Project, "is_cross_compiling", True)
        project = snapcraft.project.Project(target_deb_arch=deb_arch)
        project._snap_meta = meta.snap.Snap(name="test-snap", base="core18")

        plugin = rust.RustPlugin("test-part", options, project)

        with mock.patch("os.path.exists", return_value=False):
            plugin.pull()

        assert mock_run.call_count == 4
        mock_run.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                        "--default-toolchain",
                        "none",
                    ],
                    env=mock.ANY,
                ),
                mock.call([plugin._rustup_cmd, "install", "stable"], env=mock.ANY),
                mock.call(
                    [
                        plugin._rustup_cmd,
                        "target",
                        "add",
                        "--toolchain",
                        "stable",
                        target,
                    ],
                    env=mock.ANY,
                ),
                mock.call(
                    [
                        plugin._cargo_cmd,
                        "+stable",
                        "fetch",
                        "--manifest-path",
                        os.path.join(plugin.sourcedir, "Cargo.toml"),
                    ],
                    env=mock.ANY,
                ),
            ]
        )

        mock_run.reset_mock()
        cargo_path = Path(plugin.builddir) / "Cargo.toml"
        cargo_path.parent.mkdir(parents=True)
        cargo_path.touch()

        plugin.build()

        assert os.path.exists(os.path.join(plugin.builddir, ".cargo", "config"))
        mock_run.assert_called_once_with(
            [
                plugin._cargo_cmd,
                "+stable",
                "install",
                "--path",
                plugin.builddir,
                "--root",
                plugin.installdir,
                "--force",
                "--target",
                target,
            ],
            cwd=plugin.builddir,
            env=plugin._build_env(),
        )


class RustPluginTest(RustPluginBaseTest):
    def setUp(self):
        super().setUp()

        if self.project.deb_arch == "s390x":
            self.skipTest("architecture is not supported by rust")

    @mock.patch.object(rust.sources, "Script")
    def test_pull(self, script_mock):
        self.plugin.options.rust_revision = []
        self.plugin.options.rust_channel = []

        self.plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(3))

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(self.plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                        "--default-toolchain",
                        "none",
                    ],
                    cwd=os.path.join(self.plugin.partdir, "build"),
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [self.plugin._rustup_cmd, "install", "stable"],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [
                        self.plugin._cargo_cmd,
                        "+stable",
                        "fetch",
                        "--manifest-path",
                        os.path.join(self.plugin.sourcedir, "Cargo.toml"),
                    ],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
            ]
        )

    @mock.patch.object(rust.sources, "Script")
    def test_pull_with_cargo_lock(self, script_mock):
        self.plugin.options.rust_revision = []
        self.plugin.options.rust_channel = []
        with open(self.plugin.source_path / "Cargo.lock", "w") as f:
            f.write("")

        self.plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(3))

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(self.plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                        "--default-toolchain",
                        "none",
                    ],
                    cwd=os.path.join(self.plugin.partdir, "build"),
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [self.plugin._rustup_cmd, "install", "stable"],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [
                        self.plugin._cargo_cmd,
                        "+stable",
                        "fetch",
                        "--manifest-path",
                        os.path.join(self.plugin.sourcedir, "Cargo.toml"),
                        "--locked",
                    ],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
            ]
        )

    @mock.patch.object(rust.sources, "Script")
    def test_pull_with_rust_toolchain_file(self, script_mock):
        open(os.path.join(self.plugin.sourcedir, "rust-toolchain"), "w").close()

        self.plugin.options.rust_revision = []
        self.plugin.options.rust_channel = []

        self.plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(2))

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(self.plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                    ],
                    cwd=os.path.join(self.plugin.partdir, "build"),
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [
                        self.plugin._cargo_cmd,
                        "fetch",
                        "--manifest-path",
                        os.path.join(self.plugin.sourcedir, "Cargo.toml"),
                    ],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
            ]
        )

    @mock.patch.object(rust.sources, "Script")
    def test_pull_with_channel(self, script_mock):
        self.plugin.options.rust_revision = ""
        self.plugin.options.rust_channel = "nightly"

        self.plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(3))

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(self.plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                        "--default-toolchain",
                        "none",
                    ],
                    cwd=os.path.join(self.plugin.partdir, "build"),
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [self.plugin._rustup_cmd, "install", "nightly"],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [
                        self.plugin._cargo_cmd,
                        "+nightly",
                        "fetch",
                        "--manifest-path",
                        os.path.join(self.plugin.sourcedir, "Cargo.toml"),
                    ],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
            ]
        )

    @mock.patch.object(rust.sources, "Script")
    def test_pull_with_revision(self, script_mock):
        self.plugin.options.rust_revision = "1.13.0"
        self.plugin.options.rust_channel = ""

        self.plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(3))

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(self.plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                        "--default-toolchain",
                        "none",
                    ],
                    cwd=os.path.join(self.plugin.partdir, "build"),
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [self.plugin._rustup_cmd, "install", "1.13.0"],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [
                        self.plugin._cargo_cmd,
                        "+1.13.0",
                        "fetch",
                        "--manifest-path",
                        os.path.join(self.plugin.sourcedir, "Cargo.toml"),
                    ],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
            ]
        )

    @mock.patch.object(rust.sources, "Script")
    def test_pull_with_source_and_source_subdir(self, script_mock):
        self.plugin.options.source_subdir = "test-subdir"

        self.plugin.pull()

        self.assertThat(self.run_mock.call_count, Equals(3))

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        os.path.join(self.plugin._rustup_dir, "rustup.sh"),
                        "-y",
                        "--no-modify-path",
                        "--profile=minimal",
                        "--default-toolchain",
                        "none",
                    ],
                    cwd=os.path.join(self.plugin.partdir, "build"),
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [self.plugin._rustup_cmd, "install", "stable"],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
                mock.call(
                    [
                        self.plugin._cargo_cmd,
                        "+stable",
                        "fetch",
                        "--manifest-path",
                        os.path.join(
                            self.plugin.sourcedir, "test-subdir", "Cargo.toml"
                        ),
                    ],
                    cwd=self.plugin.builddir,
                    env=self.plugin._build_env(),
                ),
            ]
        )

    @mock.patch("snapcraft.ProjectOptions.deb_arch", "fantasy-arch")
    def test_unsupported_target_arch_raises_exception(self):
        self.assertRaises(errors.SnapcraftEnvironmentError, self.plugin._get_target)

    def test_build(self):
        self.plugin.build()

        self.run_mock.assert_called_once_with(
            [
                self.plugin._cargo_cmd,
                "+stable",
                "install",
                "--path",
                self.plugin.builddir,
                "--root",
                self.plugin.installdir,
                "--force",
            ],
            cwd=os.path.join(self.plugin.partdir, "build"),
            env=self.plugin._build_env(),
        )

    def test_install_workspace_artifacts(self):
        release_path = Path(self.plugin.builddir, "target", "release")
        os.makedirs(release_path, exist_ok=True)

        p_nonexec = Path(release_path / "nonexec")
        open(p_nonexec, "w").write("")
        p_nonexec.chmod(0o664)

        p_exec = Path(release_path / "exec")
        open(p_exec, "w").write("")
        p_exec.chmod(0o755)

        p_exec_so = Path(release_path / "exec.so")
        open(p_exec_so, "w").write("")
        p_exec_so.chmod(0o755)

        self.plugin._install_workspace_artifacts()

        bindir = Path(self.plugin.installdir, "bin")
        bins = list(bindir.iterdir())

        libdir = Path(self.plugin.installdir, "usr", "lib", self.project.arch_triplet)
        libs = list(libdir.iterdir())

        self.assertThat(bins, Equals([bindir / "exec"]))
        self.assertThat(libs, Equals([libdir / "exec.so"]))

    def test_build_workspace(self):
        cargo_path = Path(self.plugin.builddir, "Cargo.toml")
        with open(cargo_path, "w") as cargo_file:
            cargo_file.write("[workspace]" + os.linesep)
        release_path = Path(self.plugin.builddir, "target", "release")
        os.makedirs(release_path, exist_ok=True)

        self.plugin.build()

        self.run_mock.assert_called_once_with(
            [self.plugin._cargo_cmd, "+stable", "build", "--release"],
            cwd=os.path.join(self.plugin.partdir, "build"),
            env=self.plugin._build_env(),
        )

    def test_build_with_rust_toolchain_file(self):
        open(os.path.join(self.plugin.sourcedir, "rust-toolchain"), "w").close()

        self.plugin.build()

        self.run_mock.assert_called_once_with(
            [
                self.plugin._cargo_cmd,
                "install",
                "--path",
                self.plugin.builddir,
                "--root",
                self.plugin.installdir,
                "--force",
            ],
            cwd=os.path.join(self.plugin.partdir, "build"),
            env=self.plugin._build_env(),
        )

    def test_build_with_conditional_compilation(self):
        self.plugin.options.rust_features = ["conditional-compilation"]

        self.plugin.build()

        self.run_mock.assert_called_once_with(
            [
                self.plugin._cargo_cmd,
                "+stable",
                "install",
                "--path",
                self.plugin.builddir,
                "--root",
                self.plugin.installdir,
                "--force",
                "--features",
                "conditional-compilation",
            ],
            cwd=os.path.join(self.plugin.partdir, "build"),
            env=self.plugin._build_env(),
        )

    def test_build_with_cargo_lock(self):
        with open(self.plugin.source_path / "Cargo.lock", "w") as f:
            f.write("")

        self.plugin.build()

        self.run_mock.assert_called_once_with(
            [
                self.plugin._cargo_cmd,
                "+stable",
                "install",
                "--path",
                self.plugin.builddir,
                "--root",
                self.plugin.installdir,
                "--force",
                "--locked",
            ],
            cwd=os.path.join(self.plugin.partdir, "build"),
            env=self.plugin._build_env(),
        )

    def test_get_manifest_with_cargo_lock_file(self):
        with open(
            os.path.join(self.plugin.builddir, "Cargo.lock"), "w"
        ) as cargo_lock_file:
            cargo_lock_file.write("test cargo lock contents")

        self.plugin.build()

        self.assertThat(
            self.plugin.get_manifest()["cargo-lock-contents"],
            Equals("test cargo lock contents"),
        )

    def test_get_manifest_with_unexisting_cargo_lock(self):
        self.plugin.build()

        self.assertThat(
            self.plugin.get_manifest(), Not(Contains("cargo-lock-contents"))
        )

    def test_workspace(self):
        self.plugin.build()

        self.assertThat(
            self.plugin.get_manifest(), Not(Contains("cargo-lock-contents"))
        )

    def test_get_manifest_with_cargo_lock_dir(self, *_):
        os.mkdir(os.path.join(self.plugin.builddir, "Cargo.lock"))

        self.plugin.build()

        self.assertThat(
            self.plugin.get_manifest(), Not(Contains("cargo-lock-contents"))
        )

    def test_get_manifest_with_versions(self):
        original_check_output = subprocess.check_output

        def side_effect(cmd, *args, **kwargs):
            if cmd[-1] == "--version":
                binary = os.path.basename(cmd[0])
                return "test {} version".format(binary)
            return original_check_output(cmd, *args, **kwargs)

        with mock.patch.object(rust.RustPlugin, "run_output") as run_output_mock:
            run_output_mock.side_effect = side_effect
            self.plugin.build()

        expected_manifest = collections.OrderedDict()
        expected_manifest["rustup-version"] = "test rustup version"
        expected_manifest["rustc-version"] = "test rustc version"
        expected_manifest["cargo-version"] = "test cargo version"

        self.assertThat(self.plugin.get_manifest(), Equals(expected_manifest))

    def test_write_cargo_config(self):
        config_toml_path = "config.toml"

        self.plugin._write_cargo_config(cargo_config_path=config_toml_path)

        self.assertThat(config_toml_path, FileExists())

        config = toml.load(config_toml_path)

        self.assertThat(
            config,
            Equals(
                dict(
                    rustdoc_cmd=self.plugin._rustdoc_cmd,
                    rustc_cmd=self.plugin._rustc_cmd,
                    arch_triplet=self.plugin.project.arch_triplet,
                    jobs=self.plugin.parallel_build_count,
                    target={
                        self.plugin._get_target(): dict(
                            linker=self.plugin._get_linker()
                        )
                    },
                )
            ),
        )

    def test_unsupported_base(self):
        self.project._snap_meta.base = "unsupported-base"

        raised = self.assertRaises(
            errors.PluginBaseError,
            rust.RustPlugin,
            "test-part",
            self.options,
            self.project,
        )

        self.assertThat(raised.part_name, Equals("test-part"))
        self.assertThat(raised.base, Equals("unsupported-base"))
