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

from testtools import TestCase

from snapcraft_legacy.plugins.v2.kernel import KernelPlugin


class TestPluginKernel(TestCase):
    """
    Legacy kernel plugin tests.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        class Options:
            kernel_kdefconfig = ["defconfig"]
            kernel_kconfigfile = None
            kernel_kconfigflavour = ""
            kernel_kconfigs = []
            kernel_image_target = "bzImage"
            kernel_with_firmware = True
            kernel_device_trees = []
            kernel_initrd_modules = []
            kernel_initrd_configured_modules = []
            kernel_initrd_firmware = []
            kernel_initrd_compression = "lz4"
            kernel_initrd_compression_options = []
            kernel_initrd_overlay = ""
            kernel_initrd_addons = []
            kernel_compiler = "gcc"
            kernel_compiler_paths = []
            kernel_compiler_parameters = []
            kernel_enable_zfs_support = False
            kernel_enable_perf = False
            # Ensure that the PPA is not added so we don't cause side-effects
            kernel_add_ppa = False

        self._plugin = KernelPlugin(part_name="kernel", options=Options())

    def test_schema(self):
        self.assertEqual(
            KernelPlugin.get_schema(),
            {
                "$schema": "http://json-schema.org/draft-04/schema#",
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "kernel-kdefconfig": {
                        "type": "array",
                        "default": ["defconfig"],
                    },
                    "kernel-kconfigfile": {
                        "type": "string",
                        "default": None,
                    },
                    "kernel-kconfigflavour": {"type": "string", "default": ""},
                    "kernel-kconfigs": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-image-target": {
                        "oneOf": [{"type": "string"}, {"type": "object"}],
                        "default": "",
                    },
                    "kernel-with-firmware": {
                        "type": "boolean",
                        "default": True,
                    },
                    "kernel-device-trees": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-modules": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-configured-modules": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-firmware": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-compression": {
                        "type": "string",
                        "enum": ["lz4", "xz", "gz", "zstd"],
                    },
                    "kernel-initrd-compression-options": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-overlay": {
                        "type": "string",
                        "default": "",
                    },
                    "kernel-initrd-addons": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-compiler": {
                        "type": "string",
                        "default": "",
                    },
                    "kernel-compiler-paths": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-compiler-parameters": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-enable-zfs-support": {
                        "type": "boolean",
                        "default": False,
                    },
                    "kernel-enable-perf": {
                        "type": "boolean",
                        "default": False,
                    },
                    "kernel-add-ppa": {
                        "type": "boolean",
                        "default": True,
                    },
                },
            },
        )

    def test_get_base_build_packages(self):
        self.assertEqual(
            self._plugin.get_build_packages(),
            {
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
            },
        )

    def test_get_build_snaps(self):
        self.assertEqual(self._plugin.get_build_snaps(), set())

    def test_check_configuration(self):
        opt = self._plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["defconfig"])
        self.assertIs(opt.kernel_kconfigfile, None)
        self.assertEqual(opt.kernel_kconfigflavour, "")
        self.assertEqual(opt.kernel_kconfigs, [])
        self.assertEqual(opt.kernel_image_target, "bzImage")
        self.assertTrue(opt.kernel_with_firmware)
        self.assertEqual(opt.kernel_device_trees, [])
        self.assertEqual(opt.kernel_compiler, "gcc")
        self.assertEqual(opt.kernel_compiler_paths, [])
        self.assertEqual(opt.kernel_compiler_parameters, [])
        self.assertEqual(opt.kernel_initrd_modules, [])
        self.assertEqual(opt.kernel_initrd_configured_modules, [])
        self.assertEqual(opt.kernel_initrd_firmware, [])
        self.assertEqual(opt.kernel_initrd_compression, "lz4")
        self.assertEqual(opt.kernel_initrd_compression_options, [])
        self.assertEqual(opt.kernel_initrd_overlay, "")
        self.assertEqual(opt.kernel_initrd_addons, [])
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)
        self.assertFalse(opt.kernel_add_ppa)
