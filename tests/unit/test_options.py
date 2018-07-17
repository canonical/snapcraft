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

import os.path
from unittest import mock

import testtools
from testtools.matchers import Equals

import snapcraft
from snapcraft.project._project_options import (
    _get_platform_architecture,
    _32BIT_USERSPACE_ARCHITECTURE,
)
from snapcraft.internal import common
from snapcraft.internal.errors import SnapcraftEnvironmentError
from tests import unit


class NativeOptionsTestCase(unit.TestCase):

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

    @mock.patch("platform.architecture")
    @mock.patch("platform.machine")
    def test_architecture_options(
        self, mock_platform_machine, mock_platform_architecture
    ):
        mock_platform_machine.return_value = self.machine
        mock_platform_architecture.return_value = self.architecture
        options = snapcraft.ProjectOptions()
        self.assertThat(options.arch_triplet, Equals(self.expected_arch_triplet))
        self.assertThat(options.deb_arch, Equals(self.expected_deb_arch))
        self.assertThat(options.kernel_arch, Equals(self.expected_kernel_arch))

        # The core dynamic linker is correct.  Guard against stray absolute
        # paths, as they cause os.path.join to discard the previous
        # argument.
        self.assertFalse(os.path.isabs(self.expected_core_dynamic_linker))
        with mock.patch("os.path.lexists") as mock_lexists:
            mock_lexists.return_value = True
            with mock.patch("os.path.islink") as mock_islink:
                mock_islink.return_value = False
                self.assertThat(
                    options.get_core_dynamic_linker("core"),
                    Equals(
                        os.path.join(
                            common.get_core_path("core"),
                            self.expected_core_dynamic_linker,
                        )
                    ),
                )

    @mock.patch("platform.architecture")
    @mock.patch("platform.machine")
    def test_get_platform_architecture(
        self, mock_platform_machine, mock_platform_architecture
    ):
        mock_platform_machine.return_value = self.machine
        mock_platform_architecture.return_value = self.architecture
        platform_arch = _get_platform_architecture()
        userspace_conversions = _32BIT_USERSPACE_ARCHITECTURE

        if self.architecture[0] == "32bit" and self.machine in userspace_conversions:
            self.assertThat(platform_arch, Equals(userspace_conversions[self.machine]))
        else:
            self.assertThat(platform_arch, Equals(self.machine))


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


class TestHostIsCompatibleWithTargetBase(unit.TestCase):

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

    def setUp(self):
        super().setUp()
        patcher = mock.patch("snapcraft.internal.os_release.OsRelease.version_codename")
        self.codename_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_compatibility(self):
        self.codename_mock.return_value = self.codename

        self.assertThat(
            snapcraft.ProjectOptions().is_host_compatible_with_base(self.base),
            Equals(self.is_compatible),
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
