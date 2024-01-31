# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-

#
# Copyright 2020-2022 Canonical Ltd.
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

"""Common utilities for kernel plugins."""

import logging
import os
import sys
import textwrap
from typing import List, Optional

logger = logging.getLogger(__name__)


_ZFS_URL = "https://github.com/openzfs/zfs"


_required_generic = [
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

_required_security = [
    "SECURITY",
    "SECURITY_APPARMOR",
    "SYN_COOKIES",
    "STRICT_DEVMEM",
    "DEFAULT_SECURITY_APPARMOR",
    "SECCOMP",
    "SECCOMP_FILTER",
]

_required_snappy = [
    "RD_LZMA",
    "KEYS",
    "ENCRYPTED_KEYS",
    "SQUASHFS",
    "SQUASHFS_XATTR",
    "SQUASHFS_XZ",
    "SQUASHFS_LZO",
]

_required_systemd = [
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

_required_boot = ["squashfs"]


def _clone_zfs_cmd(enable_zfs: bool, dest_dir: str) -> List[str]:
    """Clone zfs git repository if needed."""
    if enable_zfs:
        return [
            textwrap.dedent(
                """
                if [ ! -d {dest_dir}/zfs ]; then
                	echo "cloning zfs..."
                	git clone --depth=1 {zfs_url} {dest_dir}/zfs -b master
                fi
                """.format(
                    dest_dir=dest_dir, zfs_url=_ZFS_URL
                )
            )
        ]
    return [
        'echo "zfs is not enabled"',
    ]


def _clean_old_build_cmd(dest_dir: str) -> List[str]:
    """Clean previous build."""
    return [
        textwrap.dedent(
            """
            echo "Cleaning previous build first..."
            [ -e {dest_dir}/modules ] && rm -rf {dest_dir}/modules
            [ -L {dest_dir}/lib/modules ] && rm -rf {dest_dir}/lib/modules
            """.format(
                dest_dir=dest_dir
            )
        )
    ]


def _do_base_config_cmd(
    make_cmd: List[str],
    config_flavour: str,
    defconfig: Optional[List[str]],
    dest_dir: str,
) -> List[str]:
    # if the parts build dir already contains a .config file, use it
    cmd = [
        'echo "Preparing config..."',
        f"if [ ! -e {dest_dir}/.config ]; then",
    ]

    # if kconfigflavour is provided, assemble the ubuntu.flavour config
    # otherwise use defconfig to seed the base config
    if defconfig:
        logger.info("Using defconfig: %s", defconfig)
        make_cmd = make_cmd.copy()
        make_cmd[1] = "-j1"  # FIXME: make this more robust
        cmd.append(f'\t{" ".join(make_cmd + defconfig)}')
    else:
        logger.info("Using ubuntu config flavour %s", config_flavour)
        conf_cmd = textwrap.dedent(
            f"""	echo "Assembling Ubuntu config..."
	if [ -f ${{KERNEL_SRC}}/debian/rules ] && [ -x ${{KERNEL_SRC}}/debian/rules ]; then
		# Generate Ubuntu kernel configs
		pushd ${{KERNEL_SRC}}
		fakeroot debian/rules clean genconfigs || true
		popd

		# Pick the right kernel .config for the target arch and flavour
		ubuntuconfig=${{KERNEL_SRC}}/CONFIGS/${{DEB_ARCH}}-config.flavour.{config_flavour}
		cat ${{ubuntuconfig}} > {dest_dir}/.config

		# Clean up kernel source directory
		pushd ${{KERNEL_SRC}}
		fakeroot debian/rules clean
		rm -rf CONFIGS/
		popd
	fi"""
        )
        cmd.extend([conf_cmd])

    cmd.append("fi")

    return cmd


def _do_patch_config_cmd(configs: Optional[List[str]], dest_dir: str) -> List[str]:
    # prepend the generated file with provided kconfigs
    #  - concat kconfigs to buffer
    #  - read current .config and append
    #  - write out to disk
    if not configs:
        return []

    config = "\n".join(configs)

    # note that prepending and appending the overrides seems
    # only way to convince all kbuild versions to pick up the
    # configs during oldconfig in .config
    return [
        'echo "Applying extra config...."',
        f"echo '{config}' > {dest_dir}/.config_snap",
        f"cat {dest_dir}/.config >> {dest_dir}/.config_snap",
        f"echo '{config}' >> {dest_dir}/.config_snap",
        f"mv {dest_dir}/.config_snap {dest_dir}/.config",
    ]


def _do_remake_config_cmd(make_cmd: List[str]) -> List[str]:
    # update config to include kconfig amendments using oldconfig
    make_cmd = make_cmd.copy()
    make_cmd[1] = "-j1"  # FIXME: make this more robust
    return [
        'echo "Remaking oldconfig...."',
        f"bash -c 'yes \"\" || true' | {' '.join(make_cmd)} oldconfig",
    ]


def _get_configure_command(
    make_cmd: List[str],
    config_flavour: str,
    defconfig: Optional[List[str]],
    configs: Optional[List[str]],
    dest_dir: str,
) -> List[str]:
    """Configure the kernel."""
    return [
        *_do_base_config_cmd(
            make_cmd=make_cmd,
            config_flavour=config_flavour,
            defconfig=defconfig,
            dest_dir=dest_dir,
        ),
        *_do_patch_config_cmd(configs=configs, dest_dir=dest_dir),
        *_do_remake_config_cmd(make_cmd=make_cmd),
    ]


def _do_parse_config(config_path: str):
    builtin = []
    modules = []
    # tokenize .config and store options in builtin[] or modules[]
    with open(config_path, encoding="utf8") as file:
        for line in file:
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


def _do_check_config(builtin: List[str], modules: List[str]):
    # check the resulting .config has all the necessary options
    msg = (
        "**** WARNING **** WARNING **** WARNING **** WARNING ****\n"
        "Your kernel config is missing some features that Ubuntu Core "
        "recommends or requires.\n"
        "While we will not prevent you from building this kernel snap, "
        "we suggest you take a look at these:\n"
    )
    required_opts = (
        _required_generic + _required_security + _required_snappy + _required_systemd
    )
    missing = []

    for code in required_opts:
        opt = f"CONFIG_{code}"
        if opt in builtin or opt in modules:
            continue
        missing.append(opt)

    if missing:
        warn = f"\n{msg}\n"
        for opt in missing:
            note = ""
            if opt == "CONFIG_SQUASHFS_LZO":
                note = "(used by desktop snaps for accelerated loading)"
            warn += f"{opt} {note}\n"
        logger.warning(warn)


def _do_check_initrd(builtin: List[str], modules: List[str]):
    # check all config items are either builtin or should be added to initrd as modules
    msg = (
        "**** WARNING **** WARNING **** WARNING **** WARNING ****\n"
        "The following features are deemed boot essential for\n"
        "ubuntu core, consider making them static[=Y] or adding\n"
        "the corresponding module to initrd:\n"
        "missing options:"
    )
    missing = []
    include_modules = []

    for code in _required_boot:
        opt = f"CONFIG_{code.upper()}"
        if opt in builtin:
            continue
        if opt in modules:
            include_modules.append(opt)
            continue
        missing.append(opt)

    if missing or include_modules:
        warn = f"\n{msg}\n"
        for opt in missing:
            warn += f"{opt}\n"
        warn += "modules to be included in the initrd:\n"
        for opt in include_modules:
            warn += f"{opt}\n"
        logger.warning(warn)


def check_new_config(config_path: str):
    """Check passed kernel config and initrd modules for required dependencies."""
    print("Checking created config...")
    builtin, modules = _do_parse_config(config_path)
    _do_check_config(builtin, modules)
    _do_check_initrd(builtin, modules)


def _call_check_config_cmd(dest_dir: str) -> List[str]:
    """Invoke the python interpreter and execute check_new_config()."""
    return [
        'echo "Checking config for expected options..."',
        " ".join(
            [
                sys.executable,
                "-I",
                os.path.abspath(__file__),
                "check_new_config",
                f"{dest_dir}/.config",
            ]
        ),
    ]


def _get_build_command(make_cmd: List[str], targets: List[str]) -> List[str]:
    """Build the kernel."""
    return [
        'echo "Building kernel..."',
        " ".join(make_cmd + targets),
    ]


def _get_zfs_build_commands(
    enable_zfs: bool, arch_triplet: str, build_dir: str, install_dir: str
) -> List[str]:
    """Include zfs build steps if required."""
    if not enable_zfs:
        return ['echo "Not building zfs modules"']

    return [
        textwrap.dedent(
            """
            echo "Building zfs modules..."
            cd {build_dir}/zfs
            ./autogen.sh
            ./configure --with-linux=${{KERNEL_SRC}} --with-linux-obj={build_dir} \
--with-config=kernel --host={arch_triplet}
            make -j$(nproc)
            make install DESTDIR={install_dir}/zfs
            release_version="$(ls {install_dir}/modules)"
            mv {install_dir}/zfs/lib/modules/${{release_version}}/extra \
{install_dir}/modules/${{release_version}}
            rm -rf {install_dir}/zfs
            echo "Rebuilding module dependencies"
            depmod -b {install_dir} ${{release_version}}
            """.format(
                build_dir=build_dir, install_dir=install_dir, arch_triplet=arch_triplet
            )
        )
    ]


def _get_perf_build_commands(
    make_cmd: List[str],
    enable_perf: bool,
    src_dir: str,
    build_dir: str,
    install_dir: str,
) -> List[str]:
    """Build perf binary if enabled."""
    if not enable_perf:
        return ['echo "Not building perf binary"']

    cmd = make_cmd + [
        # Override source and build directories
        "-C",
        f'"{src_dir}/tools/perf"',
        f'O="{build_dir}/tools/perf"',
    ]
    return [
        'echo "Building perf binary..."',
        f'mkdir -p "{build_dir}/tools/perf"',
        " ".join(cmd),
        f'install -Dm0755 "{build_dir}/tools/perf/perf" "{install_dir}/bin/perf"',
    ]


def get_build_commands(
    make_cmd: List[str],
    make_targets: List[str],
    make_install_targets: List[str],
    target_arch_triplet: str,
    config_flavour: str,
    defconfig: Optional[List[str]],
    configs: Optional[List[str]],
    enable_zfs_support: bool,
    enable_perf: bool,
    project_dir: str,
    source_dir: str,
    build_dir: str,
    install_dir: str,
) -> List[str]:
    """Get build command"""
    # kernel source can be either CRAFT_PART_SRC or CRAFT_PROJECT_DIR
    return [
        f"[ -d {source_dir}/kernel ] && KERNEL_SRC={source_dir} || KERNEL_SRC={project_dir}",
        'echo "PATH=$PATH"',
        'echo "KERNEL_SRC=${KERNEL_SRC}"',
        "",
        *_clone_zfs_cmd(
            enable_zfs=enable_zfs_support,
            dest_dir=build_dir,
        ),
        "",
        *_clean_old_build_cmd(dest_dir=install_dir),
        "",
        *_get_configure_command(
            make_cmd=make_cmd,
            config_flavour=config_flavour,
            defconfig=defconfig,
            configs=configs,
            dest_dir=build_dir,
        ),
        "",
        *_call_check_config_cmd(dest_dir=build_dir),
        "",
        *_get_build_command(make_cmd=make_cmd, targets=make_targets),
        *_get_install_command(
            make_cmd=make_cmd.copy(),
            make_install_targets=make_install_targets,
            build_dir=build_dir,
            install_dir=install_dir,
        ),
        "",
        *_get_zfs_build_commands(
            enable_zfs=enable_zfs_support,
            arch_triplet=target_arch_triplet,
            build_dir=build_dir,
            install_dir=install_dir,
        ),
        "",
        *_get_perf_build_commands(
            make_cmd=make_cmd,
            enable_perf=enable_perf,
            src_dir="${KERNEL_SRC}",
            build_dir=build_dir,
            install_dir=install_dir,
        ),
        'echo "Kernel build finished!"',
    ]


### Install


def _parse_kernel_release_cmd(build_dir: str) -> List[str]:
    """Set release from kernel.release file."""
    return [
        'echo "Parsing created kernel release..."',
        f"KERNEL_RELEASE=$(cat {build_dir}/include/config/kernel.release)",
    ]


def _copy_vmlinuz_cmd(install_dir: str) -> List[str]:
    """Install kernel image."""
    cmd = [
        'echo "Copying kernel image..."',
        # if kernel.img already exists, replace it, we are probably re-running
        # build
        f"[ -e {install_dir}/kernel.img ] && rm -rf {install_dir}/kernel.img",
        f"mv ${{KERNEL_BUILD_ARCH_DIR}}/${{KERNEL_IMAGE_TARGET}} {install_dir}/kernel.img",
    ]
    return cmd


def _copy_system_map_cmd(build_dir: str, install_dir: str) -> List[str]:
    """Install the system map."""
    cmd = [
        'echo "Copying System map..."',
        f"[ -e {install_dir}/System.map ] && rm -rf {install_dir}/System.map*",
        f"ln -f {build_dir}/System.map {install_dir}/System.map-${{KERNEL_RELEASE}}",
    ]
    return cmd


def _install_config_cmd(build_dir: str, install_dir: str) -> List[str]:
    """Install the kernel configuration file."""
    # install .config as config-$version
    return [
        "",
        'echo "Installing kernel config..."',
        f"ln -f {build_dir}/.config {install_dir}/config-${{KERNEL_RELEASE}}",
    ]


def _arrange_install_dir_cmd(install_dir: str) -> List[str]:
    """Final adjustments to installation directory."""
    return [
        textwrap.dedent(
            """

            echo "Finalizing install directory..."
            # upstream kernel installs under $INSTALL_MOD_PATH/lib/modules/
            # but snapd expects modules/ and firmware/
            mv {install_dir}/lib/modules {install_dir}/
            # remove symlinks modules/*/build and modules/*/source
            rm -rf {install_dir}/modules/*/build {install_dir}/modules/*/source
            # if there is firmware dir, move it to snap root
            # this could have been from stage packages or from kernel build
            [ -d {install_dir}/lib/firmware ] && mv {install_dir}/lib/firmware {install_dir}
            # create symlinks for modules and firmware for convenience
            ln -sf ../modules {install_dir}/lib/modules
            ln -sf ../firmware {install_dir}/lib/firmware
            """.format(
                install_dir=install_dir
            )
        )
    ]


def _get_post_install_cmd(
    build_dir: str,
    install_dir: str,
) -> List[str]:
    return [
        "",
        *_parse_kernel_release_cmd(build_dir=build_dir),
        "",
        *_copy_vmlinuz_cmd(install_dir=install_dir),
        "",
        *_copy_system_map_cmd(build_dir=build_dir, install_dir=install_dir),
        "",
    ]


def _get_install_command(
    make_cmd: List[str],
    make_install_targets: List[str],
    build_dir: str,
    install_dir: str,
) -> List[str]:
    # install to installdir
    # make_cmd = self._make_cmd.copy()
    make_cmd += [
        f"CONFIG_PREFIX={install_dir}",
    ]
    make_cmd += make_install_targets
    cmd = [
        'echo "Installing kernel build..."',
        " ".join(make_cmd),
    ]

    # add post-install steps
    cmd.extend(
        _get_post_install_cmd(
            build_dir=build_dir,
            install_dir=install_dir,
        ),
    )

    # install .config as config-$version
    cmd.extend(_install_config_cmd(build_dir=build_dir, install_dir=install_dir))

    cmd.extend(_arrange_install_dir_cmd(install_dir=install_dir))

    return cmd


### Utilities


def get_kernel_architecture(target_arch: str) -> str:
    """Obtain the kernel architecture from the target architecture."""
    if target_arch == "armhf":
        return "arm"

    if target_arch == "arm64":
        return "arm64"

    if target_arch == "riscv64":
        return "riscv"

    if target_arch == "amd64":
        return "x86"

    raise ValueError("unknown kernel architecture")


def get_deb_architecture(target_arch: str) -> str:
    """Obtain the deb architecture from the target architecture."""
    if target_arch == "armhf":
        return "armhf"

    if target_arch == "arm64":
        return "arm64"

    if target_arch == "riscv64":
        return "riscv64"

    if target_arch == "amd64":
        return "amd64"

    raise ValueError("unknown deb architecture")


if __name__ == "__main__":
    globals()[sys.argv[1]](sys.argv[2])
