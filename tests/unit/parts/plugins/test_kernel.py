# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2022 Canonical Ltd
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

import pytest
from craft_parts import Part, PartInfo, ProjectInfo

from snapcraft.parts.plugins.kernel import KernelPlugin

# pylint: disable=attribute-defined-outside-init

class TestPluginKernel:
    """
    Kernel plugin tests.
    """

    @pytest.fixture(autouse=True)
    def setup_method_fixture(self, new_dir):
        properties = KernelPlugin.properties_class.unmarshal({
            "source": ".",
            "kernel-initrd-compression": "lz4",
            "kernel-enable-zfs-support": "False",
            "kernel-image-target": "bzImage",
        })
        part = Part("foo", {})

        project_info = ProjectInfo(application_name="test", cache_dir=new_dir)

        part_info = PartInfo(project_info=project_info, part=part)

        self._plugin = KernelPlugin(properties=properties, part_info=part_info)

    def test_get_base_build_packages(self):
        assert self._plugin.get_build_packages() == {
            "bc",
            "binutils",
            "gcc",
            "cmake",
            "cryptsetup",
            "dracut-core",
            "kmod",
            "kpartx",
            "lz4",
            "systemd",
        }

    def test_get_build_snaps(self):
        assert self._plugin.get_build_snaps() == set()

    def test_check_configuration(self):
        opt = self._plugin.options

        assert opt.kernel_kdefconfig == ["defconfig"]
        assert opt.kernel_kconfigfile is None
        assert opt.kernel_kconfigflavour is None
        assert opt.kernel_kconfigs is None
        assert opt.kernel_image_target == "bzImage"
        assert opt.kernel_with_firmware
        assert opt.kernel_device_trees is None
        assert not opt.kernel_build_efi_image
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert opt.kernel_initrd_modules is None
        assert opt.kernel_initrd_configured_modules is None
        assert opt.kernel_initrd_firmware is None
        assert opt.kernel_initrd_compression == "lz4"
        assert opt.kernel_initrd_compression_options is None
        assert opt.kernel_initrd_channel == "stable"
        assert opt.kernel_initrd_overlay is None
        assert opt.kernel_initrd_addons is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf
