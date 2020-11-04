# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import platform
from unittest import mock

import testtools
from testtools.matchers import Equals

import snapcraft
from snapcraft.internal import common
from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.project._project_options import (
    _32BIT_USERSPACE_ARCHITECTURE,
    _get_platform_architecture,
)
from tests import unit


class TestNativeOptions:

    scenarios = [
        (
            "amd64",
            dict(
                machine="x86_64",
                architecture=("64bit", "ELF"),
                expected_arch_triplet="x86_64-linux-gnu",
                expected_deb_arch="amd64",
                expected_kernel_arch="x86",
                expected_core_dynamic_linker="lib64/ld-linux-x86-64.so.2",
            ),
        ),
        (
            "amd64-kernel-i686-userspace",
            dict(
                machine="x86_64",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="i386-linux-gnu",
                expected_deb_arch="i386",
                expected_kernel_arch="x86",
                expected_core_dynamic_linker="lib/ld-linux.so.2",
            ),
        ),
        (
            "i686",
            dict(
                machine="i686",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="i386-linux-gnu",
                expected_deb_arch="i386",
                expected_kernel_arch="x86",
                expected_core_dynamic_linker="lib/ld-linux.so.2",
            ),
        ),
        (
            "armv7l",
            dict(
                machine="armv7l",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="arm-linux-gnueabihf",
                expected_deb_arch="armhf",
                expected_kernel_arch="arm",
                expected_core_dynamic_linker="lib/ld-linux-armhf.so.3",
            ),
        ),
        (
            "aarch64",
            dict(
                machine="aarch64",
                architecture=("64bit", "ELF"),
                expected_arch_triplet="aarch64-linux-gnu",
                expected_deb_arch="arm64",
                expected_kernel_arch="arm64",
                expected_core_dynamic_linker="lib/ld-linux-aarch64.so.1",
            ),
        ),
        (
            "aarch64-kernel-armv7l-userspace",
            dict(
                machine="aarch64",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="arm-linux-gnueabihf",
                expected_deb_arch="armhf",
                expected_kernel_arch="arm",
                expected_core_dynamic_linker="lib/ld-linux-armhf.so.3",
            ),
        ),
        (
            "armv8l-kernel-armv7l-userspace",
            dict(
                machine="armv8l",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="arm-linux-gnueabihf",
                expected_deb_arch="armhf",
                expected_kernel_arch="arm",
                expected_core_dynamic_linker="lib/ld-linux-armhf.so.3",
            ),
        ),
        (
            "ppc",
            dict(
                machine="ppc",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="powerpc-linux-gnu",
                expected_deb_arch="powerpc",
                expected_kernel_arch="powerpc",
                expected_core_dynamic_linker="lib/ld-linux.so.2",
            ),
        ),
        (
            "ppc64le",
            dict(
                machine="ppc64le",
                architecture=("64bit", "ELF"),
                expected_arch_triplet="powerpc64le-linux-gnu",
                expected_deb_arch="ppc64el",
                expected_kernel_arch="powerpc",
                expected_core_dynamic_linker="lib64/ld64.so.2",
            ),
        ),
        (
            "ppc64le-kernel-ppc-userspace",
            dict(
                machine="ppc64le",
                architecture=("32bit", "ELF"),
                expected_arch_triplet="powerpc-linux-gnu",
                expected_deb_arch="powerpc",
                expected_kernel_arch="powerpc",
                expected_core_dynamic_linker="lib/ld-linux.so.2",
            ),
        ),
        (
            "riscv64",
            dict(
                machine="riscv64",
                architecture=("64bit", "ELF"),
                expected_arch_triplet="riscv64-linux-gnu",
                expected_deb_arch="riscv64",
                expected_kernel_arch="riscv64",
                expected_core_dynamic_linker="lib/ld-linux-riscv64-lp64d.so.1",
            ),
        ),
        (
            "s390x",
            dict(
                machine="s390x",
                architecture=("64bit", "ELF"),
                expected_arch_triplet="s390x-linux-gnu",
                expected_deb_arch="s390x",
                expected_kernel_arch="s390",
                expected_core_dynamic_linker="lib/ld64.so.1",
            ),
        ),
    ]

    def test_architecture_options(
        self,
        monkeypatch,
        machine,
        architecture,
        expected_arch_triplet,
        expected_deb_arch,
        expected_kernel_arch,
        expected_core_dynamic_linker,
    ):
        monkeypatch.setattr(platform, "architecture", lambda: architecture)
        monkeypatch.setattr(platform, "machine", lambda: machine)

        options = snapcraft.ProjectOptions()

        assert options.arch_triplet == expected_arch_triplet
        assert options.deb_arch == expected_deb_arch
        assert options.kernel_arch == expected_kernel_arch

        # The core dynamic linker is correct.  Guard against stray absolute
        # paths, as they cause os.path.join to discard the previous
        # argument.
        monkeypatch.setattr(os.path, "lexists", lambda x: True)
        monkeypatch.setattr(os.path, "islink", lambda x: False)
        expected_linker_path = os.path.join(
            common.get_installed_snap_path("core"), expected_core_dynamic_linker
        )
        assert options.get_core_dynamic_linker("core") == expected_linker_path

    def test_get_platform_architecture(
        self,
        monkeypatch,
        machine,
        architecture,
        expected_arch_triplet,
        expected_deb_arch,
        expected_kernel_arch,
        expected_core_dynamic_linker,
    ):
        monkeypatch.setattr(platform, "architecture", lambda: architecture)
        monkeypatch.setattr(platform, "machine", lambda: machine)

        platform_arch = _get_platform_architecture()
        userspace_conversions = _32BIT_USERSPACE_ARCHITECTURE

        if architecture[0] == "32bit" and machine in userspace_conversions:
            assert platform_arch == userspace_conversions[machine]
        else:
            assert platform_arch == machine


class OptionsTestCase(unit.TestCase):
    def test_cross_compiler_prefix_missing(self):
        options = snapcraft.ProjectOptions(target_deb_arch="x86_64")

        with testtools.ExpectedException(
            SnapcraftEnvironmentError,
            "Cross compilation not supported for target arch 'x86_64'",
        ):
            options.cross_compiler_prefix

    @mock.patch("platform.architecture")
    @mock.patch("platform.machine")
    def test_cross_compiler_prefix_empty(
        self, mock_platform_machine, mock_platform_architecture
    ):
        mock_platform_machine.return_value = "x86_64"
        mock_platform_architecture.return_value = ("64bit", "ELF")
        options = snapcraft.ProjectOptions(target_deb_arch="i386")
        self.assertThat(options.cross_compiler_prefix, Equals(""))


class TestHostIsCompatibleWithTargetBase:

    scenarios = (
        ("trusty core", dict(codename="trusty", base="core", is_compatible=True)),
        ("xenial core", dict(codename="xenial", base="core", is_compatible=True)),
        ("bionic core", dict(codename="bionic", base="core", is_compatible=False)),
        ("trusty core18", dict(codename="trusty", base="core18", is_compatible=True)),
        ("xenial core18", dict(codename="xenial", base="core18", is_compatible=True)),
        ("bionic core18", dict(codename="bionic", base="core18", is_compatible=True)),
        (
            "Random codename core18",
            dict(codename="random", base="core18", is_compatible=False),
        ),
        (
            "trusty unknown-base",
            dict(codename="trusty", base="unknown", is_compatible=False),
        ),
    )

    def test_compatibility(self, monkeypatch, codename, base, is_compatible):
        monkeypatch.setattr(
            snapcraft.internal.os_release.OsRelease,
            "version_codename",
            lambda x: codename,
        )

        assert (
            snapcraft.ProjectOptions().is_host_compatible_with_base(base)
            is is_compatible
        )


class TestLinkerVersionForBase(unit.TestCase):
    def setUp(self):
        super().setUp()
        patcher = mock.patch("snapcraft.file_utils.get_linker_version_from_file")
        self.get_linker_version_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_get_linker_version_for_core(self):
        self.assertThat(
            snapcraft.ProjectOptions()._get_linker_version_for_base("core"),
            Equals("2.23"),
        )
        self.get_linker_version_mock.assert_not_called()

    def test_get_linker_version_for_core18(self):
        self.assertThat(
            snapcraft.ProjectOptions()._get_linker_version_for_base("core18"),
            Equals("2.27"),
        )
        self.get_linker_version_mock.assert_not_called()

    def test_get_linker_version_for_random_core(self):
        self.get_linker_version_mock.return_value = "4.10"
        self.assertThat(
            snapcraft.ProjectOptions()._get_linker_version_for_base("random"),
            Equals("4.10"),
        )
        self.get_linker_version_mock.assert_called_once_with("ld-2.23.so")
