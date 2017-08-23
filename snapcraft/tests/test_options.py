# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from unittest import mock

import testtools
from testtools.matchers import Equals

import snapcraft
from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft import tests


class NativeOptionsTestCase(tests.TestCase):

    scenarios = [
        ('amd64', dict(
            machine='x86_64',
            architecture=('64bit', 'ELF'),
            expected_arch_triplet='x86_64-linux-gnu',
            expected_deb_arch='amd64',
            expected_kernel_arch='x86')),
        ('amd64-kernel-i686-userspace', dict(
            machine='x86_64',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='i386-linux-gnu',
            expected_deb_arch='i386',
            expected_kernel_arch='x86')),
        ('i686', dict(
            machine='i686',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='i386-linux-gnu',
            expected_deb_arch='i386',
            expected_kernel_arch='x86')),
        ('armv7l', dict(
            machine='armv7l',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='arm-linux-gnueabihf',
            expected_deb_arch='armhf',
            expected_kernel_arch='arm')),
        ('aarch64', dict(
            machine='aarch64',
            architecture=('64bit', 'ELF'),
            expected_arch_triplet='aarch64-linux-gnu',
            expected_deb_arch='arm64',
            expected_kernel_arch='arm64')),
        ('aarch64-kernel-armv7l-userspace', dict(
            machine='aarch64',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='arm-linux-gnueabihf',
            expected_deb_arch='armhf',
            expected_kernel_arch='arm')),
        ('armv8l-kernel-armv7l-userspace', dict(
            machine='armv8l',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='arm-linux-gnueabihf',
            expected_deb_arch='armhf',
            expected_kernel_arch='arm')),
        ('ppc', dict(
            machine='ppc',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='powerpc-linux-gnu',
            expected_deb_arch='powerpc',
            expected_kernel_arch='powerpc')),
        ('ppc64le', dict(
            machine='ppc64le',
            architecture=('64bit', 'ELF'),
            expected_arch_triplet='powerpc64le-linux-gnu',
            expected_deb_arch='ppc64el',
            expected_kernel_arch='powerpc')),
        ('ppc64le-kernel-ppc-userspace', dict(
            machine='ppc64le',
            architecture=('32bit', 'ELF'),
            expected_arch_triplet='powerpc-linux-gnu',
            expected_deb_arch='powerpc',
            expected_kernel_arch='powerpc')),
        ('s390x', dict(
            machine='s390x',
            architecture=('64bit', 'ELF'),
            expected_arch_triplet='s390x-linux-gnu',
            expected_deb_arch='s390x',
            expected_kernel_arch='s390'))
    ]

    @mock.patch('platform.architecture')
    @mock.patch('platform.machine')
    def test_architecture_options(
            self, mock_platform_machine, mock_platform_architecture):
        mock_platform_machine.return_value = self.machine
        mock_platform_architecture.return_value = self.architecture
        options = snapcraft.ProjectOptions()
        self.assertThat(
            options.arch_triplet, Equals(self.expected_arch_triplet))
        self.assertThat(
            options.deb_arch, Equals(self.expected_deb_arch))
        self.assertThat(
            options.kernel_arch, Equals(self.expected_kernel_arch))

    @mock.patch('platform.architecture')
    @mock.patch('platform.machine')
    def test_get_platform_architecture(
            self, mock_platform_machine, mock_platform_architecture):
        mock_platform_machine.return_value = self.machine
        mock_platform_architecture.return_value = self.architecture
        platform_arch = snapcraft._options._get_platform_architecture()
        userspace_conversions = \
            snapcraft._options._32BIT_USERSPACE_ARCHITECTURE

        if self.architecture[0] == '32bit' and \
           self.machine in userspace_conversions:
            self.assertThat(
                platform_arch, Equals(userspace_conversions[self.machine]))
        else:
            self.assertThat(platform_arch, Equals(self.machine))


class OptionsTestCase(tests.TestCase):

    def test_cross_compiler_prefix_missing(self):
        options = snapcraft.ProjectOptions(target_deb_arch='x86_64')

        with testtools.ExpectedException(
                SnapcraftEnvironmentError,
                "Cross compilation not supported for target arch 'x86_64'"):
            options.cross_compiler_prefix
