# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Unit tests for the ubuntu_core_initrd plugin."""

import dataclasses
import functools
import os
import pathlib
import re
from collections.abc import Generator

import pytest
from craft_parts import Part, PartInfo, ProjectInfo
from craft_parts.dirs import ProjectDirs

from snapcraft import errors
from snapcraft.parts.plugins import UbuntuCoreInitrdPlugin

# Kernel version constants shared across tests
KERNEL_ABI = "5.15.0-143"
KERNEL_FLAVOUR = "generic"
KERNEL_MODULE_DIR_NAME = f"{KERNEL_ABI}-{KERNEL_FLAVOUR}"


@dataclasses.dataclass
class BuildParameters:
    """Build parameters normally provided by snapcraft."""

    base: str
    arch_build_on: str
    arch_build_for: str
    arch_triplet_build_for: str


@functools.lru_cache
def get_project_info_parameters() -> list[BuildParameters]:
    """Generate BuildParameters for all tested bases and architecture combinations."""
    return [
        BuildParameters(
            base=base,
            arch_build_on="amd64",
            arch_build_for=arch_build_for,
            arch_triplet_build_for=(
                "x86_64-linux-gnu" if arch_build_for == "amd64" else "aarch64-linux-gnu"
            ),
        )
        for base in ["core22", "core24"]
        for arch_build_for in ["amd64", "arm64"]
    ]


def get_test_fixture_ids() -> list[str]:
    """Return human-readable pytest IDs for parametrized build parameters."""
    return [f"{p.base}, {p.arch_build_for}" for p in get_project_info_parameters()]


@pytest.fixture
def build_cmds_environ(monkeypatch: pytest.MonkeyPatch) -> None:
    """Set environment variables required by get_build_commands."""
    monkeypatch.setenv("SNAP_CONTEXT", "snap-context")
    monkeypatch.setenv("SNAP", "snap")
    monkeypatch.setenv("SNAP_VERSION", "1.0.0")


@pytest.fixture
def kernel_modules_dir(tmp_path: pathlib.Path) -> pathlib.Path:
    """Create a fake kernel modules directory under stage_dir.

    get_build_commands() iterates stage_dir/modules/ to discover the kernel
    version, so this directory must exist before the call.
    """
    modules_dir = tmp_path / "stage" / "modules" / KERNEL_MODULE_DIR_NAME
    modules_dir.mkdir(parents=True)
    return modules_dir


@pytest.fixture
def setup_plugin_fixture() -> Generator:
    """Factory fixture for creating UbuntuCoreInitrdPlugin instances."""

    def _make(
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        properties: dict | None = None,
    ) -> UbuntuCoreInitrdPlugin:
        if properties is None:
            properties = {}
        orig_cwd = os.getcwd()
        os.chdir(tmp_path)
        try:
            project_dirs = ProjectDirs(work_dir=tmp_path)
            part = Part("ubuntu-core-initrd", {}, project_dirs=project_dirs)
            project_info = ProjectInfo(
                application_name="test",
                cache_dir=tmp_path,
                project_dirs=project_dirs,
                arch=build_params.arch_build_for,
                base=build_params.base,
            )
        finally:
            os.chdir(orig_cwd)
        part_info = PartInfo(project_info=project_info, part=part)
        props = UbuntuCoreInitrdPlugin.properties_class.unmarshal(properties)
        return UbuntuCoreInitrdPlugin(properties=props, part_info=part_info)

    yield _make


def normalise_script(script: str) -> list[str]:
    """Split a rendered script into non-blank, non-comment lines."""
    lines = [line.strip() for line in script.split("\n")]
    lines = [re.sub(r"^\s*#.*$", "", line) for line in lines]
    return [line for line in lines if line]


class TestUbuntuCoreInitrdPluginProperties:
    """Tests for plugin __init__ and property validation."""

    def test_unsupported_base_raises_error(self, tmp_path: pathlib.Path) -> None:
        """Test that instantiating with an unsupported base raises SnapcraftError."""
        part = Part("ubuntu-core-initrd", {})
        project_info = ProjectInfo(
            application_name="test",
            cache_dir=tmp_path,
            arch="amd64",
            base="core20",
        )
        part_info = PartInfo(project_info=project_info, part=part)
        props = UbuntuCoreInitrdPlugin.properties_class.unmarshal({})
        with pytest.raises(
            errors.SnapcraftError,
            match="only core22 and core24 bases are supported",
        ):
            UbuntuCoreInitrdPlugin(properties=props, part_info=part_info)

    @pytest.mark.parametrize("base", ["core22", "core24"])
    def test_supported_bases_do_not_raise(
        self, base: str, tmp_path: pathlib.Path
    ) -> None:
        """Test that core22 and core24 bases do not raise on construction."""
        part = Part("ubuntu-core-initrd", {})
        project_info = ProjectInfo(
            application_name="test",
            cache_dir=tmp_path,
            arch="amd64",
            base=base,
        )
        part_info = PartInfo(project_info=project_info, part=part)
        props = UbuntuCoreInitrdPlugin.properties_class.unmarshal({})
        _ = UbuntuCoreInitrdPlugin(properties=props, part_info=part_info)


class TestPluginUbuntuCoreInitrd:
    """Tests for plugin metadata methods."""

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_snaps(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
    ) -> None:
        """Test that get_build_snaps returns an empty set."""
        plugin = setup_plugin_fixture(build_params=build_params, tmp_path=tmp_path)
        assert plugin.get_build_snaps() == set()

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_packages(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
    ) -> None:
        """Test that get_build_packages returns the expected set of packages."""
        plugin = setup_plugin_fixture(build_params=build_params, tmp_path=tmp_path)
        expected_packages = {
            "bzip2",
            "gzip",
            "lz4",
            "lzma",
            "lzop",
            "ubuntu-core-initramfs",
            "xz-utils",
            "zstd",
        }
        assert expected_packages.issubset(plugin.get_build_packages())

    @pytest.mark.parametrize(
        "build_params",
        [p for p in get_project_info_parameters() if p.arch_build_for == "amd64"],
        ids=[
            f"{p.base}, {p.arch_build_for}"
            for p in get_project_info_parameters()
            if p.arch_build_for == "amd64"
        ],
    )
    def test_get_build_environment_native(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
    ) -> None:
        """Test that a native (non-cross) build returns an empty build environment."""
        plugin = setup_plugin_fixture(build_params=build_params, tmp_path=tmp_path)
        assert plugin.get_build_environment() == {}

    @pytest.mark.parametrize(
        "build_params",
        [p for p in get_project_info_parameters() if p.arch_build_for == "arm64"],
        ids=[
            f"{p.base}, {p.arch_build_for}"
            for p in get_project_info_parameters()
            if p.arch_build_for == "arm64"
        ],
    )
    def test_get_build_environment_cross(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
    ) -> None:
        """Test that cross-compile builds return the expected build environment."""
        plugin = setup_plugin_fixture(build_params=build_params, tmp_path=tmp_path)
        expected_env = {
            "ARCH": "arm64",
            "CROSS_COMPILE": "aarch64-linux-gnu",
            "DEB_HOST_ARCH": "arm64",
            "DEB_BUILD_ARCH": "amd64",
            "DEB_HOST_MULTIARCH": "aarch64-linux-gnu",
        }
        assert plugin.get_build_environment() == expected_env

    def test_get_out_of_source_build(self) -> None:
        """Test that the plugin declares out-of-source builds."""
        assert UbuntuCoreInitrdPlugin.get_out_of_source_build() is True


class TestGetBuildCommands:
    """Tests for get_build_commands Jinja2 template rendering."""

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_default(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test default build commands: zstd compression, no firmware, no EFI."""
        plugin = setup_plugin_fixture(build_params=build_params, tmp_path=tmp_path)
        result = plugin.get_build_commands()

        assert len(result) == 1
        script = result[0]

        # Compression workaround sed command (default is zstd, so cmd replaces with itself)
        assert "sed -i 's/zstd -1 -T0/zstd -1 -T0/g'" in script
        # Initrd creation without firmware
        assert "ubuntu-core-initramfs create-initrd" in script
        assert f"--kernelver {KERNEL_ABI}" in script
        assert "--firmwaredir" not in script
        # Initrd image symlink install line
        assert f"initrd.img-{KERNEL_ABI}" in script
        # No EFI image creation
        assert "create-efi" not in script

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_with_firmware(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test that an overlay property produces a --firmwaredir argument."""
        overlay_dir = tmp_path / "my-overlay"
        overlay_dir.mkdir()
        plugin = setup_plugin_fixture(
            build_params=build_params,
            tmp_path=tmp_path,
            properties={"ubuntu-core-initrd-overlay": "my-overlay"},
        )
        script = plugin.get_build_commands()[0]

        assert "ubuntu-core-initramfs create-initrd" in script
        assert "--firmwaredir" in script
        assert "create-efi" not in script

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_efi_none(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test that efi_image_type=none produces no EFI section in the script."""
        plugin = setup_plugin_fixture(
            build_params=build_params,
            tmp_path=tmp_path,
            properties={"ubuntu-core-initrd-efi-image-type": "none"},
        )
        script = plugin.get_build_commands()[0]

        assert "ubuntu-core-initramfs create-initrd" in script
        assert "create-efi" not in script

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_efi_unsigned(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test EFI unsigned image generation uses the --unsigned flag."""
        plugin = setup_plugin_fixture(
            build_params=build_params,
            tmp_path=tmp_path,
            properties={"ubuntu-core-initrd-efi-image-type": "unsigned"},
        )
        script = plugin.get_build_commands()[0]

        assert "create-efi" in script
        assert "--unsigned" in script
        assert "--key" not in script
        assert "--cert" not in script
        assert f"kernel.efi-{KERNEL_ABI}" in script
        assert "rm" in script

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_efi_signed(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test EFI signed image generation uses custom key/cert from part_build_dir."""
        plugin = setup_plugin_fixture(
            build_params=build_params,
            tmp_path=tmp_path,
            properties={"ubuntu-core-initrd-efi-image-type": "signed"},
        )
        script = plugin.get_build_commands()[0]
        part_build_dir = str(plugin.part_info.part_build_dir)

        assert "create-efi" in script
        assert "--key" in script
        assert "--cert" in script
        assert f"{part_build_dir}/efi-signing-key.pem" in script
        assert f"{part_build_dir}/efi-certificate.pem" in script
        assert "--unsigned" not in script
        assert f"kernel.efi-{KERNEL_ABI}" in script

    @pytest.mark.parametrize(
        "compression,expected_cmd",
        [
            ("lz4", "lz4 -l -9"),
            ("gzip", "gzip"),
            ("bzip2", "bzip2"),
            ("xz", "xz -z"),
            ("lzma", "lzma -z"),
        ],
    )
    def test_get_build_commands_compression_variants(
        self,
        compression: str,
        expected_cmd: str,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test that each compression type generates the correct sed replacement."""
        build_params = BuildParameters(
            base="core24",
            arch_build_on="amd64",
            arch_build_for="amd64",
            arch_triplet_build_for="x86_64-linux-gnu",
        )
        plugin = setup_plugin_fixture(
            build_params=build_params,
            tmp_path=tmp_path,
            properties={"ubuntu-core-initrd-compression": compression},
        )
        script = plugin.get_build_commands()[0]

        assert f"sed -i 's/zstd -1 -T0/{expected_cmd}/g'" in script

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_kernel_abi_in_script(
        self,
        build_params: BuildParameters,
        tmp_path: pathlib.Path,
        setup_plugin_fixture,
        build_cmds_environ,
        kernel_modules_dir: pathlib.Path,
    ) -> None:
        """Test that the discovered kernel ABI appears in the rendered script."""
        plugin = setup_plugin_fixture(build_params=build_params, tmp_path=tmp_path)
        script = plugin.get_build_commands()[0]

        assert KERNEL_ABI in script
        assert "--kerneldir" in script
        assert KERNEL_MODULE_DIR_NAME in script
