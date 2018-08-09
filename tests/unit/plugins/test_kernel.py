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

import contextlib
import logging
import os
import subprocess
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, FileContains, HasLength

from textwrap import dedent

import snapcraft
from snapcraft import storeapi
from snapcraft.plugins import kernel
from tests import unit


class KernelPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            build_parameters = []
            kconfigfile = None
            kconfigflavour = None
            kdefconfig = []
            kconfigs = []
            kernel_image_target = "bzImage"
            kernel_with_firmware = True
            kernel_initrd_modules = []
            kernel_initrd_firmware = []
            kernel_device_trees = []
            kernel_initrd_compression = "gz"
            build_attributes = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch("subprocess.check_call")
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(kernel.KernelPlugin, "run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(kernel.KernelPlugin, "run_output")
        self.run_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.BasePlugin.build")
        self.base_build_mock = patcher.start()
        self.addCleanup(patcher.stop)

        @contextlib.contextmanager
        def tempdir():
            self.tempdir = "temporary-directory"
            os.mkdir(self.tempdir)
            yield self.tempdir

        patcher = mock.patch("tempfile.TemporaryDirectory")
        self.tempdir_mock = patcher.start()
        self.tempdir_mock.side_effect = tempdir
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = kernel.KernelPlugin.schema()

        properties = schema["properties"]
        self.assertThat(properties["kdefconfig"]["type"], Equals("array"))
        self.assertThat(properties["kdefconfig"]["default"], Equals(["defconfig"]))

        self.assertThat(properties["kconfigfile"]["type"], Equals("string"))
        self.assertThat(properties["kconfigfile"]["default"], Equals(None))

        self.assertThat(properties["kconfigflavour"]["type"], Equals("string"))
        self.assertThat(properties["kconfigflavour"]["default"], Equals(None))

        for prop in [
            "kconfigs",
            "kernel-initrd-modules",
            "kernel-initrd-firmware",
            "kernel-device-trees",
        ]:
            self.assertThat(properties[prop]["type"], Equals("array"))
            self.assertThat(properties[prop]["default"], Equals([]))
            self.assertThat(properties[prop]["minitems"], Equals(1))
            self.assertThat(properties[prop]["items"]["type"], Equals("string"))
            self.assertTrue(properties[prop]["uniqueItems"])

        self.assertThat(
            properties["kernel-image-target"]["oneOf"],
            Equals([{"type": "string"}, {"type": "object"}]),
        )
        self.assertThat(properties["kernel-image-target"]["default"], Equals(""))

        self.assertThat(properties["kernel-with-firmware"]["type"], Equals("boolean"))
        self.assertThat(properties["kernel-with-firmware"]["default"], Equals(True))

        self.assertThat(
            properties["kernel-initrd-compression"]["type"], Equals("string")
        )
        self.assertThat(
            properties["kernel-initrd-compression"]["default"], Equals("gz")
        )
        self.assertThat(properties["kernel-initrd-compression"]["enum"], Equals(["gz"]))

    def test_get_build_properties(self):
        expected_build_properties = [
            "kernel-image-target",
            "kernel-with-firmware",
            "kernel-initrd-modules",
            "kernel-initrd-firmware",
            "kernel-device-trees",
            "kernel-initrd-compression",
        ]
        resulting_build_properties = kernel.KernelPlugin.get_build_properties()
        expected_build_properties.extend(
            snapcraft.plugins.kbuild.KBuildPlugin.get_build_properties()
        )

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def _assert_generic_check_call(self, builddir, installdir, os_snap_path):
        self.assertThat(self.check_call_mock.call_count, Equals(4))
        self.check_call_mock.assert_has_calls(
            [
                mock.call('yes "" | make -j2 oldconfig', shell=True, cwd=builddir),
                mock.call(
                    ["unsquashfs", os_snap_path, "boot"], cwd="temporary-directory"
                ),
                mock.call(
                    "cat temporary-directory/squashfs-root/boot"
                    "/initrd.img-core | xz -dc | cpio -i",
                    cwd=os.path.join(builddir, "initrd-staging"),
                    shell=True,
                ),
                mock.call(
                    "find . | cpio --create --format=newc | "
                    "gzip > {}".format(os.path.join(installdir, "initrd-4.4.2.img")),
                    cwd=os.path.join(builddir, "initrd-staging"),
                    shell=True,
                ),
            ]
        )

    def _assert_common_assets(self, installdir):
        for asset in [
            "initrd-4.4.2.img",
            "initrd.img",
            "kernel.img",
            "bzImage-4.4.2",
            "System.map-4.4.2",
        ]:
            self.assertTrue(
                os.path.exists(os.path.join(installdir, asset)),
                "Missing {}".format(asset),
            )

    def _simulate_build(
        self,
        sourcedir,
        builddir,
        installdir,
        do_dtbs=False,
        do_release=True,
        do_kernel=True,
        do_system_map=True,
        do_firmware=True,
    ):
        os.makedirs(sourcedir)
        kernel_version = "4.4.2"

        def create_assets():
            build_arch_path = os.path.join(
                builddir, "arch", self.project_options.kernel_arch, "boot"
            )

            initrd_path = os.path.join(
                installdir, "initrd-{}.img".format(kernel_version)
            )
            release_path = os.path.join(builddir, "include", "config", "kernel.release")
            kernel_path = os.path.join(
                build_arch_path, self.options.kernel_image_target
            )
            system_map_path = os.path.join(builddir, "System.map")
            modules_dep_path = os.path.join(
                installdir, "lib", "modules", kernel_version, "modules.dep"
            )
            modules_dep_bin_path = "{}.bin".format(modules_dep_path)
            if do_firmware:
                fw_bin_path = os.path.join(installdir, "lib", "firmware", "fake-fw.bin")
                os.makedirs(os.path.join(installdir, "lib", "firmware", "fake-fw-dir"))
            dtb_path = os.path.join(build_arch_path, "dts", "fake-dtb.dtb")

            os.makedirs(os.path.dirname(release_path))
            with open(release_path, "w") as f:
                if do_release:
                    f.write("{}\n".format(kernel_version))
                else:
                    f.write("\n")

            files = [initrd_path, modules_dep_path, modules_dep_bin_path]
            if do_kernel:
                files.append(kernel_path)
            if do_system_map:
                files.append(system_map_path)
            if do_dtbs:
                files.append(dtb_path)
            if do_firmware:
                files.append(fw_bin_path)

            for f in files:
                os.makedirs(os.path.dirname(f), exist_ok=True)
                open(f, "w").close()

        self.base_build_mock.side_effect = create_assets

    def test_unpack_gzip_initrd(self):
        def check_call_effect(*args, **kwargs):
            if "xz" in args[0]:
                raise subprocess.CalledProcessError(1, args)

        self.check_call_mock.side_effect = check_call_effect

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)
        plugin._unpack_generic_initrd()

        self.check_call_mock.assert_has_calls(
            [
                mock.call(
                    "cat temporary-directory/squashfs-root/boot/"
                    "initrd.img-core | gzip -dc | cpio -i",
                    cwd=os.path.join(plugin.builddir, "initrd-staging"),
                    shell=True,
                )
            ]
        )

    def test_unpack_lzma_initrd(self):
        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        plugin._unpack_generic_initrd()

        self.check_call_mock.assert_has_calls(
            [
                mock.call(
                    "cat temporary-directory/squashfs-root/boot/"
                    "initrd.img-core | xz -dc | cpio -i",
                    cwd=os.path.join(plugin.builddir, "initrd-staging"),
                    shell=True,
                )
            ]
        )

    def test_unpack_unsupported_initrd_type(self):
        def check_call_effect(*args, **kwargs):
            if "unsquashfs" not in args[0]:
                raise subprocess.CalledProcessError(1, args)

        self.check_call_mock.side_effect = check_call_effect
        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self.assertRaises(RuntimeError, plugin._unpack_generic_initrd)

    def test_pack_initrd_modules(self):
        self.options.kernel_initrd_modules = ["squashfs", "vfat"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        # Fake some assets
        plugin.kernel_release = "4.4"
        modules_path = os.path.join(plugin.installdir, "lib", "modules", "4.4")
        initrd_modules_staging_path = os.path.join("staging", "lib", "modules", "4.4")
        os.makedirs(modules_path)
        open(os.path.join(modules_path, "modules.dep"), "w").close()
        open(os.path.join(modules_path, "modules.dep.bin"), "w").close()
        os.makedirs(initrd_modules_staging_path)
        open(os.path.join(plugin.installdir, "initrd-4.4.img"), "w").close()

        with mock.patch.object(plugin, "_unpack_generic_initrd") as m_unpack:
            m_unpack.return_value = "staging"
            plugin._make_initrd()

        modprobe_cmd = [
            "modprobe",
            "-n",
            "--show-depends",
            "-d",
            plugin.installdir,
            "-S",
            "4.4",
        ]
        self.run_output_mock.assert_has_calls(
            [mock.call(modprobe_cmd + ["squashfs"], env=mock.ANY)]
        )
        self.run_output_mock.assert_has_calls(
            [mock.call(modprobe_cmd + ["vfat"], env=mock.ANY)]
        )

    def test_pack_initrd_modules_return_same_deps(self):
        self.options.kernel_initrd_modules = ["squashfs", "vfat"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        # Fake some assets
        plugin.kernel_release = "4.4"
        modules_path = os.path.join(plugin.installdir, "lib", "modules", "4.4")
        initrd_modules_staging_path = os.path.join("staging", "lib", "modules", "4.4")
        os.makedirs(modules_path)
        open(os.path.join(modules_path, "modules.dep"), "w").close()
        open(os.path.join(modules_path, "modules.dep.bin"), "w").close()
        os.makedirs(initrd_modules_staging_path)
        open(os.path.join(plugin.installdir, "initrd-4.4.img"), "w").close()
        open(os.path.join(plugin.installdir, "serport.ko"), "w").close()

        self.run_output_mock.return_value = "insmod {}/serport.ko".format(
            plugin.installdir
        )

        with mock.patch.object(plugin, "_unpack_generic_initrd") as m_unpack:
            m_unpack.return_value = "staging"
            plugin._make_initrd()

        modprobe_cmd = [
            "modprobe",
            "-n",
            "--show-depends",
            "-d",
            plugin.installdir,
            "-S",
            "4.4",
        ]
        self.run_output_mock.assert_has_calls(
            [mock.call(modprobe_cmd + ["squashfs"], env=mock.ANY)]
        )
        self.run_output_mock.assert_has_calls(
            [mock.call(modprobe_cmd + ["vfat"], env=mock.ANY)]
        )

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigfile(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))
        self._assert_common_assets(plugin.installdir)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_verbose_with_kconfigfile(self):
        fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(fake_logger)

        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        plugin.build()

        self.assertThat(self.check_call_mock.call_count, Equals(4))
        self.check_call_mock.assert_has_calls(
            [
                mock.call(
                    'yes "" | make -j2 V=1 oldconfig', shell=True, cwd=plugin.builddir
                ),
                mock.call(
                    ["unsquashfs", plugin.os_snap, "boot"], cwd="temporary-directory"
                ),
                mock.call(
                    "cat temporary-directory/squashfs-root/boot/"
                    "initrd.img-core | xz -dc | cpio -i",
                    cwd=os.path.join(plugin.builddir, "initrd-staging"),
                    shell=True,
                ),
                mock.call(
                    "find . | cpio --create --format=newc | "
                    "gzip > {}".format(
                        os.path.join(plugin.installdir, "initrd-4.4.2.img")
                    ),
                    cwd=os.path.join(plugin.builddir, "initrd-staging"),
                    shell=True,
                ),
            ]
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "V=1", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "V=1",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))
        self._assert_common_assets(plugin.installdir)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_check_config(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        builtin, modules = plugin._do_parse_config(self.options.kconfigfile)
        plugin._do_check_config(builtin, modules)

        required_opts = (
            kernel.required_generic
            + kernel.required_security
            + kernel.required_snappy
            + kernel.required_systemd
        )
        for warn in required_opts:
            self.assertIn("CONFIG_{}".format(warn), fake_logger.output)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_check_initrd(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.options.kconfigfile = "config"
        self.options.kernel_initrd_modules = ["my-fake-module"]
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        builtin, modules = plugin._do_parse_config(self.options.kconfigfile)
        plugin._do_check_initrd(builtin, modules)

        for module in kernel.required_boot:
            self.assertIn("CONFIG_{}".format(module.upper()), fake_logger.output)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigfile_and_kconfigs(self):
        self.options.kconfigfile = "config"
        self.options.kconfigs = ["SOMETHING=y", "ACCEPT=n"]

        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        expected_config = """SOMETHING=y
ACCEPT=n

ACCEPT=y

SOMETHING=y
ACCEPT=n
"""
        self.assertThat(config_contents, Equals(expected_config))
        self._assert_common_assets(plugin.installdir)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_defconfig_and_kconfigs(self):
        self.options.kdefconfig = ["defconfig"]
        self.options.kconfigs = ["SOMETHING=y", "ACCEPT=n"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        config_file = os.path.join(plugin.builddir, ".config")

        def fake_defconfig(*args, **kwargs):
            if os.path.exists(config_file):
                return
            with open(config_file, "w") as f:
                f.write("ACCEPT=y\n")

        self.run_mock.side_effect = fake_defconfig

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(3))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j1", "defconfig"]),
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        expected_config = """SOMETHING=y
ACCEPT=n

ACCEPT=y

SOMETHING=y
ACCEPT=n
"""
        self.assertThat(config_contents, Equals(expected_config))
        self._assert_common_assets(plugin.installdir)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_two_defconfigs(self):
        self.options.kdefconfig = ["defconfig", "defconfig2"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        config_file = os.path.join(plugin.builddir, ".config")

        def fake_defconfig(*args, **kwargs):
            if os.path.exists(config_file):
                return
            with open(config_file, "w") as f:
                f.write("ACCEPT=y\n")

        self.run_mock.side_effect = fake_defconfig

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(3))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j1", "defconfig", "defconfig2"]),
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        self.assertTrue(os.path.exists(config_file))
        self._assert_common_assets(plugin.installdir)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigfile_and_dtbs(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")
        self.options.kernel_device_trees = ["fake-dtb"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(
            plugin.sourcedir, plugin.builddir, plugin.installdir, do_dtbs=True
        )

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules", "fake-dtb.dtb"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))
        self._assert_common_assets(plugin.installdir)
        self.assertTrue(
            os.path.exists(os.path.join(plugin.installdir, "dtbs", "fake-dtb.dtb"))
        )

    def test_build_with_kconfigfile_and_dtbs_not_found(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")
        self.options.kernel_device_trees = ["fake-dtb"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        raised = self.assertRaises(RuntimeError, plugin.build)

        self.assertThat(
            str(raised), Equals("No match for dtb 'fake-dtb.dtb' was found")
        )

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigfile_and_modules(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")
        self.options.kernel_initrd_modules = ["my-fake-module"]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        def fake_unpack(*args, **kwargs):
            modules_dir = os.path.join(
                plugin.builddir, "initrd-staging", "lib", "modules", "4.4.2"
            )
            if os.path.exists(modules_dir):
                return
            os.makedirs(modules_dir)

        self.check_call_mock.side_effect = fake_unpack

        def fake_output(*args, **kwargs):
            if args[0][:3] == ["modprobe", "-n", "--show-depends"]:
                module_path = os.path.join(
                    plugin.installdir, "lib", "modules", "4.4.2", "some-module.ko"
                )
                open(module_path, "w").close()
                return "insmod {} enable_fbdev=1\n".format(module_path)
            else:
                raise Exception(args[0])

        self.run_output_mock.side_effect = fake_output

        # Set a path that doesn't contain '/sbin' so we can verify that it's
        # added to the modprobe call.
        self.useFixture(fixtures.EnvironmentVariable("PATH", "/usr/bin"))

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        class _check_env:
            def __init__(self, test):
                self.test = test

            def __eq__(self, other):
                self.test.assertThat(other, Contains("PATH"))
                paths = other["PATH"].split(":")
                self.test.assertThat(paths, Contains("/sbin"))
                return True

        self.assertThat(self.run_output_mock.call_count, Equals(1))
        self.run_output_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "modprobe",
                        "-n",
                        "--show-depends",
                        "-d",
                        plugin.installdir,
                        "-S",
                        "4.4.2",
                        "my-fake-module",
                    ],
                    env=_check_env(self),
                )
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))
        self._assert_common_assets(plugin.installdir)

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigfile_and_firmware(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")
        self.options.kernel_initrd_firmware = [
            "lib/firmware/fake-fw-dir",
            "lib/firmware/fake-fw.bin",
        ]

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        def fake_unpack(*args, **kwargs):
            modules_dir = os.path.join(
                plugin.builddir, "initrd-staging", "lib", "modules", "4.4.2"
            )
            if os.path.exists(modules_dir):
                return
            os.makedirs(modules_dir)

        self.check_call_mock.side_effect = fake_unpack

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertThat(config_contents, Equals("ACCEPT=y\n"))
        self._assert_common_assets(plugin.installdir)
        self.assertTrue(
            os.path.exists(os.path.join(plugin.installdir, "firmware", "fake-fw-dir"))
        )

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigfile_and_no_firmware(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")
        self.options.kernel_with_firmware = False

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(
            plugin.sourcedir,
            plugin.builddir,
            plugin.installdir,
            do_dtbs=True,
            do_firmware=False,
        )

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

    @mock.patch.object(snapcraft.ProjectOptions, "kernel_arch", new="not_arm")
    def test_build_with_kconfigflavour(self):
        arch = self.project_options.deb_arch
        branch = "master"
        flavour = "vanilla"
        self.options.kconfigflavour = flavour

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)
        self._simulate_build(plugin.sourcedir, plugin.builddir, plugin.installdir)

        debiandir = os.path.join(plugin.sourcedir, "debian")
        os.mkdir(debiandir)
        with open(os.path.join(debiandir, "debian.env"), "w") as f:
            f.write("DEBIAN=debian.{}".format(branch))
        os.mkdir(os.path.join(plugin.sourcedir, "debian.{}".format(branch)))
        basedir = os.path.join(plugin.sourcedir, "debian.{}".format(branch), "config")
        archdir = os.path.join(
            plugin.sourcedir, "debian.{}".format(branch), "config", arch
        )
        os.mkdir(basedir)
        os.mkdir(archdir)
        commoncfg = os.path.join(basedir, "config.common.ports")
        ubuntucfg = os.path.join(basedir, "config.common.ubuntu")
        archcfg = os.path.join(archdir, "config.common.{}".format(arch))
        flavourcfg = os.path.join(archdir, "config.flavour.{}".format(flavour))

        with open(commoncfg, "w") as f:
            f.write("ACCEPT=y\n")
        with open(ubuntucfg, "w") as f:
            f.write("ACCEPT=m\n")
        with open(archcfg, "w") as f:
            f.write("ACCEPT=y\n")
        with open(flavourcfg, "w") as f:
            f.write("# ACCEPT is not set\n")

        plugin.build()

        self._assert_generic_check_call(
            plugin.builddir, plugin.installdir, plugin.os_snap
        )

        self.assertThat(self.run_mock.call_count, Equals(2))
        self.run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2", "bzImage", "modules"]),
                mock.call(
                    [
                        "make",
                        "-j2",
                        "CONFIG_PREFIX={}".format(plugin.installdir),
                        "modules_install",
                        "INSTALL_MOD_PATH={}".format(plugin.installdir),
                        "firmware_install",
                        "INSTALL_FW_PATH={}".format(
                            os.path.join(plugin.installdir, "lib", "firmware")
                        ),
                    ]
                ),
            ]
        )

        config_file = os.path.join(plugin.builddir, ".config")
        self.assertTrue(os.path.exists(config_file))

        self.assertThat(
            config_file,
            FileContains(
                dedent(
                    """\
        ACCEPT=y
        ACCEPT=m
        ACCEPT=y
        # ACCEPT is not set
        """
                )
            ),
        )
        self._assert_common_assets(plugin.installdir)

    def test_build_with_missing_kernel_fails(self):
        self.options.kconfigfile = "config"

        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(
            plugin.sourcedir, plugin.builddir, plugin.installdir, do_kernel=False
        )

        raised = self.assertRaises(ValueError, plugin.build)

        self.assertThat(
            str(raised),
            Equals(
                "kernel build did not output a vmlinux binary in top level "
                "dir, expected {!r}".format(
                    os.path.join(
                        plugin.builddir,
                        "arch",
                        self.project_options.kernel_arch,
                        "boot",
                        "bzImage",
                    )
                )
            ),
        )

    def test_build_with_missing_kernel_release_fails(self):
        self.options.kconfigfile = "config"

        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(
            plugin.sourcedir, plugin.builddir, plugin.installdir, do_release=False
        )

        raised = self.assertRaises(ValueError, plugin.build)

        self.assertThat(
            str(raised),
            Equals(
                "No kernel release version info found at {!r}".format(
                    os.path.join(plugin.builddir, "include", "config", "kernel.release")
                )
            ),
        )

    def test_build_with_missing_system_map_fails(self):
        self.options.kconfigfile = "config"
        with open(self.options.kconfigfile, "w") as f:
            f.write("ACCEPT=y\n")

        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)

        self._simulate_build(
            plugin.sourcedir, plugin.builddir, plugin.installdir, do_system_map=False
        )

        raised = self.assertRaises(ValueError, plugin.build)

        self.assertThat(
            str(raised),
            Equals("kernel build did not output a System.map in top level dir"),
        )

    def test_enable_cross_compilation(self):
        project_options = snapcraft.ProjectOptions(target_deb_arch="arm64")
        plugin = kernel.KernelPlugin("test-part", self.options, project_options)
        plugin.enable_cross_compilation()

        self.assertThat(
            plugin.make_cmd,
            Equals(
                [
                    "make",
                    "-j2",
                    "ARCH=arm64",
                    "CROSS_COMPILE=aarch64-linux-gnu-",
                    "PATH={}:/usr/{}/bin".format(
                        os.environ.copy().get("PATH", ""), "aarch64-linux-gnu"
                    ),
                ]
            ),
        )

    def test_override_cross_compile(self):
        project_options = snapcraft.ProjectOptions(target_deb_arch="arm64")
        plugin = kernel.KernelPlugin("test-part", self.options, project_options)
        self.useFixture(
            fixtures.EnvironmentVariable("CROSS_COMPILE", "foo-bar-toolchain-")
        )
        plugin.enable_cross_compilation()

        self.assertThat(
            plugin.make_cmd,
            Equals(
                [
                    "make",
                    "-j2",
                    "ARCH=arm64",
                    "CROSS_COMPILE=foo-bar-toolchain-",
                    "PATH={}:/usr/{}/bin".format(
                        os.environ.copy().get("PATH", ""), "aarch64-linux-gnu"
                    ),
                ]
            ),
        )

    def test_override_cross_compile_empty(self):
        project_options = snapcraft.ProjectOptions(target_deb_arch="arm64")
        plugin = kernel.KernelPlugin("test-part", self.options, project_options)
        self.useFixture(fixtures.EnvironmentVariable("CROSS_COMPILE", ""))
        plugin.enable_cross_compilation()

        self.assertThat(
            plugin.make_cmd,
            Equals(
                [
                    "make",
                    "-j2",
                    "ARCH=arm64",
                    "CROSS_COMPILE=aarch64-linux-gnu-",
                    "PATH={}:/usr/{}/bin".format(
                        os.environ.copy().get("PATH", ""), "aarch64-linux-gnu"
                    ),
                ]
            ),
        )

    def test_kernel_image_target_as_map(self):
        self.options.kernel_image_target = {"arm64": "Image"}
        project_options = snapcraft.ProjectOptions(target_deb_arch="arm64")
        plugin = kernel.KernelPlugin("test-part", self.options, project_options)

        self.assertThat(plugin.make_targets, Equals(["Image", "modules", "dtbs"]))

    def test_kernel_image_target_as_string(self):
        self.options.kernel_image_target = "Image"
        project_options = snapcraft.ProjectOptions(target_deb_arch="arm64")
        plugin = kernel.KernelPlugin("test-part", self.options, project_options)

        self.assertThat(plugin.make_targets, Equals(["Image", "modules", "dtbs"]))

    def test_kernel_image_target_non_existent(self):
        class Options:
            build_parameters = []
            kconfigfile = None
            kdefconfig = []
            kconfigs = []
            kernel_with_firmware = True
            kernel_initrd_modules = []
            kernel_initrd_firmware = []
            kernel_device_trees = []
            kernel_initrd_compression = "gz"

        project_options = snapcraft.ProjectOptions(target_deb_arch="arm64")
        plugin = kernel.KernelPlugin("test-part", self.options, project_options)

        self.assertThat(plugin.make_targets, Equals(["bzImage", "modules", "dtbs"]))

    @mock.patch.object(storeapi.StoreClient, "download")
    def test_pull(self, download_mock):
        plugin = kernel.KernelPlugin("test-part", self.options, self.project_options)
        plugin.pull()

        download_mock.assert_called_once_with(
            "core", "stable", plugin.os_snap, self.project_options.deb_arch, ""
        )


class KernelPluginDefaulTargetsTestCase(unit.TestCase):

    scenarios = [
        ("amd64", {"deb_arch": "amd64", "expected": "bzImage"}),
        ("i386", {"deb_arch": "i386", "expected": "bzImage"}),
        ("arm64", {"deb_arch": "arm64", "expected": "Image.gz"}),
        ("armhf", {"deb_arch": "armhf", "expected": "zImage"}),
    ]

    def setUp(self):
        super().setUp()

        class Options:
            build_parameters = []
            kconfigfile = None
            kdefconfig = []
            kconfigs = []
            kernel_image_target = ""
            kernel_with_firmware = True
            kernel_initrd_modules = []
            kernel_initrd_firmware = []
            kernel_device_trees = []
            kernel_initrd_compression = "gz"

        self.options = Options()

    def test_default(self):
        project = snapcraft.ProjectOptions(target_deb_arch=self.deb_arch)
        plugin = kernel.KernelPlugin("test-part", self.options, project)

        self.assertThat(plugin.kernel_image_target, Equals(self.expected))
