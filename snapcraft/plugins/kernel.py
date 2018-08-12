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

"""The kernel plugin refines the generic kbuild plugin to allow building
kernel snaps with all the bells and whistles in one shot...

WARNING: this plugin's API is unstable. The cross compiling support is
         experimental.

The following kernel specific options are provided by this plugin:

    - kernel-image-target:
      (yaml object, string or null for default target)
      the default target is bzImage and can be set to any specific
      target.
      For more complex cases where one would want to use
      the same snapcraft.yaml to target multiple architectures a
      yaml object can be used. This yaml object would be a map of
      debian architecture and kernel image build targets.

    - kernel-initrd-modules:
      (array of string)
      list of modules to include in initrd; note that kernel snaps do not
      provide the core boot logic which comes from snappy Ubuntu Core
      OS snap. Include all modules you need for mounting rootfs here.

    - kernel-with-firmware:
      (boolean; default: True)
      use this flag to disable shipping binary firmwares

    - kernel-initrd-firmware:
      (array of string)
      list of firmware files to include in the initrd; these need to be
      relative paths to .installdir and this option does not work if you
      disable building firmware

    - kernel-initrd-compression:
      (string; default: gz)
      initrd compression to use; the only supported value now is 'gz'.

    - kernel-device-trees:
      (array of string)
      list of device trees to build, the format is <device-tree-name>.dts.
"""

import glob
import logging
import os
import shutil
import subprocess
import tempfile

import snapcraft
from snapcraft.plugins import kbuild

logger = logging.getLogger(__name__)


_compression_command = {"gz": "gzip"}

default_kernel_image_target = {
    "amd64": "bzImage",
    "i386": "bzImage",
    "armhf": "zImage",
    "arm64": "Image.gz",
    "powerpc": "uImage",
    "ppc64el": "vmlinux.strip",
    "s390x": "bzImage",
}

required_generic = [
    "DEVTMPFS",
    "DEVTMPFS_MOUNT",
    "TMPFS_POSIX_ACL",
    "IPV6",
    "SYSVIPC",
    "SYSVIPC_SYSCTL",
    "VFAT_FS",
    "NLS_CODEPAGE_437",
    "NLS_ISO8859_1",
]

required_security = [
    "SECURITY",
    "SECURITY_APPARMOR",
    "SYN_COOKIES",
    "STRICT_DEVMEM",
    "DEFAULT_SECURITY_APPARMOR",
    "SECCOMP",
    "SECCOMP_FILTER",
    "CC_STACKPROTECTOR",
    "CC_STACKPROTECTOR_STRONG",
    "DEBUG_RODATA",
    "DEBUG_SET_MODULE_RONX",
]

required_snappy = [
    "RD_LZMA",
    "KEYS",
    "ENCRYPTED_KEYS",
    "SQUASHFS",
    "SQUASHFS_XATTR",
    "SQUASHFS_XZ",
    "DEVPTS_MULTIPLE_INSTANCES",
]

required_systemd = [
    "DEVTMPFS",
    "CGROUPS",
    "INOTIFY_USER",
    "SIGNALFD",
    "TIMERFD",
    "EPOLL",
    "NET",
    "SYSFS",
    "PROC_FS",
    "FHANDLE",
    "BLK_DEV_BSG",
    "NET_NS",
    "IPV6",
    "AUTOFS4_FS",
    "TMPFS_POSIX_ACL",
    "TMPFS_XATTR",
    "SECCOMP",
]

required_boot = ["squashfs"]


class KernelPlugin(kbuild.KBuildPlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["kernel-image-target"] = {
            "oneOf": [{"type": "string"}, {"type": "object"}],
            "default": "",
        }

        schema["properties"]["kernel-with-firmware"] = {
            "type": "boolean",
            "default": True,
        }

        schema["properties"]["kernel-initrd-modules"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["kernel-initrd-firmware"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["kernel-device-trees"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["kernel-initrd-compression"] = {
            "type": "string",
            "default": "gz",
            "enum": ["gz"],
        }

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return super().get_build_properties() + [
            "kernel-image-target",
            "kernel-with-firmware",
            "kernel-initrd-modules",
            "kernel-initrd-firmware",
            "kernel-device-trees",
            "kernel-initrd-compression",
        ]

    @property
    def compression_cmd(self):
        return _compression_command[self.options.kernel_initrd_compression]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        # We need to be able to shell out to modprobe
        self.build_packages.append("kmod")

        self._set_kernel_targets()

        self.os_snap = os.path.join(self.sourcedir, "os.snap")
        self.kernel_release = ""

    def enable_cross_compilation(self):
        logger.info(
            "Cross compiling kernel target {!r}".format(self.project.kernel_arch)
        )
        super().enable_cross_compilation()
        # by enabling cross compilation, the kernel_arch and deb_arch
        # from the project options have effectively changed so we reset
        # kernel targets.
        self._set_kernel_targets()

    def _set_kernel_targets(self):
        if not self.options.kernel_image_target:
            self.kernel_image_target = default_kernel_image_target[
                self.project.deb_arch
            ]
        elif isinstance(self.options.kernel_image_target, str):
            self.kernel_image_target = self.options.kernel_image_target
        elif self.project.deb_arch in self.options.kernel_image_target:
            self.kernel_image_target = self.options.kernel_image_target[
                self.project.deb_arch
            ]

        self.make_targets = [self.kernel_image_target, "modules"]
        self.make_install_targets = [
            "modules_install",
            "INSTALL_MOD_PATH={}".format(self.installdir),
        ]
        self.dtbs = ["{}.dtb".format(i) for i in self.options.kernel_device_trees]
        if self.dtbs:
            self.make_targets.extend(self.dtbs)
        elif self.project.kernel_arch == "arm" or self.project.kernel_arch == "arm64":
            self.make_targets.append("dtbs")
            self.make_install_targets.extend(
                ["dtbs_install", "INSTALL_DTBS_PATH={}/dtbs".format(self.installdir)]
            )
        self.make_install_targets.extend(self._get_fw_install_targets())

    def _get_fw_install_targets(self):
        if not self.options.kernel_with_firmware:
            return []

        return [
            "firmware_install",
            "INSTALL_FW_PATH={}".format(
                os.path.join(self.installdir, "lib", "firmware")
            ),
        ]

    def _unpack_generic_initrd(self):
        initrd_path = os.path.join("boot", "initrd.img-core")
        initrd_unpacked_path = os.path.join(self.builddir, "initrd-staging")
        if os.path.exists(initrd_unpacked_path):
            shutil.rmtree(initrd_unpacked_path)
        os.makedirs(initrd_unpacked_path)

        with tempfile.TemporaryDirectory() as temp_dir:
            unsquashfs_path = snapcraft.file_utils.get_tool_path("unsquashfs")
            subprocess.check_call(
                [unsquashfs_path, self.os_snap, os.path.dirname(initrd_path)],
                cwd=temp_dir,
            )

            tmp_initrd_path = os.path.join(temp_dir, "squashfs-root", initrd_path)

            decompressed = False
            # Roll over valid decompression mechanisms until one works
            for decompressor in ("xz", "gzip"):
                try:
                    subprocess.check_call(
                        "cat {0} | {1} -dc | cpio -i".format(
                            tmp_initrd_path, decompressor
                        ),
                        shell=True,
                        cwd=initrd_unpacked_path,
                    )
                except subprocess.CalledProcessError:
                    pass
                else:
                    decompressed = True
                    break

            if not decompressed:
                raise RuntimeError("The initrd file type is unsupported")

        return initrd_unpacked_path

    def _make_initrd(self):

        logger.info(
            "Generating driver initrd for kernel release: {}".format(
                self.kernel_release
            )
        )

        initrd_unpacked_path = self._unpack_generic_initrd()

        modprobe_outs = []

        # modprobe is typically in /sbin, which may not always be available on
        # the PATH. Add it for this call.
        env = os.environ.copy()
        env["PATH"] += ":/sbin"
        for module in self.options.kernel_initrd_modules:
            modprobe_out = self.run_output(
                [
                    "modprobe",
                    "-n",
                    "--show-depends",
                    "-d",
                    self.installdir,
                    "-S",
                    self.kernel_release,
                    module,
                ],
                env=env,
            )
            modprobe_outs.extend(modprobe_out.split(os.linesep))

        modprobe_outs = [_ for _ in modprobe_outs if _]
        modules_path = os.path.join("lib", "modules", self.kernel_release)
        for src in set(_.split()[1] for _ in modprobe_outs):
            dst = os.path.join(
                initrd_unpacked_path, os.path.relpath(src, self.installdir)
            )
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            os.link(src, dst)

        if modprobe_outs:
            for module_info in ["modules.dep", "modules.dep.bin"]:
                module_info_path = os.path.join(modules_path, module_info)
                src = os.path.join(self.installdir, module_info_path)
                dst = os.path.join(initrd_unpacked_path, module_info_path)
                os.link(src, dst)

        # TODO pickup required firmware from modules.
        for firmware in self.options.kernel_initrd_firmware:
            src = os.path.join(self.installdir, firmware)
            dst = os.path.join(initrd_unpacked_path, firmware)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if os.path.isdir(src):
                shutil.copytree(src, dst)
            else:
                os.link(src, dst)

        initrd = "initrd-{}.img".format(self.kernel_release)
        initrd_path = os.path.join(self.installdir, initrd)
        subprocess.check_call(
            "find . | cpio --create --format=newc | "
            "{} > {}".format(self.compression_cmd, initrd_path),
            shell=True,
            cwd=initrd_unpacked_path,
        )
        unversioned_initrd_path = os.path.join(self.installdir, "initrd.img")
        os.link(initrd_path, unversioned_initrd_path)

    def _parse_kernel_release(self):
        kernel_release_path = os.path.join(
            self.builddir, "include", "config", "kernel.release"
        )

        with open(kernel_release_path, "r") as f:
            self.kernel_release = f.read().strip()

        if not self.kernel_release:
            raise ValueError(
                "No kernel release version info found at {!r}".format(
                    kernel_release_path
                )
            )

    def _get_build_arch_dir(self):
        return os.path.join(self.builddir, "arch", self.project.kernel_arch, "boot")

    def _copy_vmlinuz(self):
        kernel = "{}-{}".format(self.kernel_image_target, self.kernel_release)
        src = os.path.join(self._get_build_arch_dir(), self.kernel_image_target)
        dst = os.path.join(self.installdir, kernel)
        if not os.path.exists(src):
            raise ValueError(
                "kernel build did not output a vmlinux binary in top level "
                "dir, expected {!r}".format(src)
            )
        os.link(src, dst)
        os.link(src, os.path.join(self.installdir, "kernel.img"))

    def _copy_system_map(self):
        src = os.path.join(self.builddir, "System.map")
        dst = os.path.join(self.installdir, "System.map-{}".format(self.kernel_release))
        if not os.path.exists(src):
            raise ValueError(
                "kernel build did not output a System.map in top level dir"
            )
        os.link(src, dst)

    def _copy_dtbs(self):
        if not self.options.kernel_device_trees:
            return

        dtb_dir = os.path.join(self.installdir, "dtbs")
        os.makedirs(dtb_dir)

        base_path = os.path.join(self._get_build_arch_dir(), "dts")
        for dtb in self.dtbs:
            found_dtbs = glob.glob(os.path.join(base_path, dtb))
            if not found_dtbs:
                raise RuntimeError("No match for dtb {!r} was found".format(dtb))
            for f in found_dtbs:
                os.link(f, os.path.join(dtb_dir, os.path.basename(f)))

    def _do_parse_config(self, config_path):
        builtin = []
        modules = []
        # tokenize .config and store options in builtin[] or modules[]
        with open(config_path) as f:
            for line in f:
                tok = line.strip().split("=")
                items = len(tok)
                if items == 2:
                    opt = tok[0].upper()
                    val = tok[1].upper()
                    if val == "Y":
                        builtin.append(opt)
                    elif val == "M":
                        modules.append(opt)
        return builtin, modules

    def _do_check_config(self, builtin, modules):
        # check the resulting .config has all the necessary options
        msg = (
            "**** WARNING **** WARNING **** WARNING **** WARNING ****\n"
            "Your kernel config is missing some features that Ubuntu Core "
            "recommends or requires.\n"
            "While we will not prevent you from building this kernel snap, "
            "we suggest you take a look at these:\n"
        )
        required_opts = (
            required_generic + required_security + required_snappy + required_systemd
        )
        missing = []

        for code in required_opts:
            opt = "CONFIG_{}".format(code)
            if opt in builtin:
                continue
            elif opt in modules:
                continue
            else:
                missing.append(opt)

        if missing:
            warn = "\n{}\n".format(msg)
            for opt in missing:
                note = ""
                if opt == "CONFIG_CC_STACKPROTECTOR_STRONG":
                    note = "(4.1.x and later versions only)"
                elif opt == "CONFIG_DEVPTS_MULTIPLE_INSTANCES":
                    note = "(4.8.x and earlier versions only)"
                warn += "{} {}\n".format(opt, note)
            logger.warn(warn)

    def _do_check_initrd(self, builtin, modules):
        # check all required_boot[] items are either builtin or part of initrd
        msg = (
            "**** WARNING **** WARNING **** WARNING **** WARNING ****\n"
            "The following features are deemed boot essential for\n"
            "ubuntu core, consider making them static[=Y] or adding\n"
            "the corresponding module to initrd:\n"
        )
        missing = []

        for code in required_boot:
            opt = "CONFIG_{}".format(code.upper())
            if opt in builtin:
                continue
            elif opt in modules:
                if code in self.options.kernel_initrd_modules:
                    continue
                else:
                    missing.append(opt)
            else:
                missing.append(opt)

        if missing:
            warn = "\n{}\n".format(msg)
            for opt in missing:
                warn += "{}\n".format(opt)
            logger.warn(warn)

    def pull(self):
        super().pull()
        snapcraft.download("core", "stable", self.os_snap, self.project.deb_arch)

    def do_configure(self):
        super().do_configure()

        builtin, modules = self._do_parse_config(self.get_config_path())
        self._do_check_config(builtin, modules)
        self._do_check_initrd(builtin, modules)

    def do_install(self):
        super().do_install()

        self._parse_kernel_release()
        self._make_initrd()
        self._copy_vmlinuz()
        self._copy_system_map()
        self._copy_dtbs()

        # upstream kernel installs under $INSTALL_MOD_PATH/lib/modules/
        # but snapd expects modules/ and firmware/
        shutil.move(os.path.join(self.installdir, "lib", "modules"), self.installdir)
        if self.options.kernel_with_firmware:
            shutil.move(
                os.path.join(self.installdir, "lib", "firmware"), self.installdir
            )
        os.rmdir(os.path.join(self.installdir, "lib"))
        # install .config as config-$version
        config = "config-{}".format(self.kernel_release)
        config_path = os.path.join(self.installdir, config)
        dot_config_path = self.get_config_path()
        os.link(dot_config_path, config_path)
