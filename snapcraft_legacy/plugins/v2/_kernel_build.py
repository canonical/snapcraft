# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
# pylint: disable=line-too-long,too-many-lines,attribute-defined-outside-init
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
from typing import List, Optional

logger = logging.getLogger(__name__)


_compression_command = {"gz": "gzip", "lz4": "lz4", "xz": "xz", "zstd": "zstd"}
_compressor_options = {"gz": "-7", "lz4": "-l -9", "xz": "-7", "zstd": "-1 -T0"}

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


def get_initrd_kernel_modules(
    initrd_modules: Optional[List[str]], configured_modules: Optional[List[str]]
) -> List[str]:
    """Collect a list of kernel modules to add to the initrd."""
    initrd_installed_kernel_modules = ""
    initrd_configured_kernel_modules = ""

    if initrd_modules:
        initrd_installed_kernel_modules = f"{' '.join(initrd_modules)}"
    if configured_modules:
        initrd_configured_kernel_modules = f"{' '.join(configured_modules)}"
    return [
        "# list of kernel modules to be installed in initrd",
        f'initrd_installed_kernel_modules="{initrd_installed_kernel_modules}"',
        "# list of kernel modules in initrd to be auto loaded by",
        "# any module in this list implies to be added to initrd",
        f'initrd_configured_kernel_modules="{initrd_configured_kernel_modules}"',
    ]


def link_files_fnc_cmd() -> List[str]:
    """Add function to link files."""
    return [
        "# link files, accept wild cards",
        "# 1: reference dir, 2: file(s) including wild cards, 3: dst dir",
        "link_files() {",
        '\tif [ "${2}" = "*" ]; then',
        "\t\tfor f in $(ls ${1})",
        "\t\tdo",
        '\t\t\tlink_files "${1}" "${f}" "${3}"',
        "\t\tdone",
        "\t\treturn 0",
        "\tfi",
        '\tif [ -d "${1}/${2}" ]; then',
        "\t\tfor f in $(ls ${1}/${2})",
        "\t\tdo",
        '\t\t\tlink_files "${1}" "${2}/${f}" "${3}"',
        "\t\tdone",
        "\t\treturn 0",
        "\tfi",
        "",
        '\tlocal found=""',
        "\tfor f in $(ls ${1}/${2})",
        "\tdo",
        '\t\tif [[ -L "${f}" ]]; then',
        " ".join(
            [
                "\t\t\tlocal rel_path=$(",
                "realpath",
                "--no-symlinks",
                "--relative-to=${1}",
                "${f}",
                ")",
            ]
        ),
        "\t\telse",
        " ".join(
            [
                "\t\t\tlocal rel_path=$(",
                "realpath",
                "-se",
                "--relative-to=${1}",
                "${f}",
                ")",
            ]
        ),
        "\t\tfi",
        "\t\tlocal dir_path=$(dirname ${rel_path})",
        "\t\tmkdir -p ${3}/${dir_path}",
        '\t\techo "installing ${f} to ${3}/${dir_path}"',
        "\t\tln -f ${f} ${3}/${dir_path}",
        '\t\tfound="yes"',
        "\tdone",
        '\tif [ "yes" = "${found}" ]; then',
        "\t\treturn 0",
        "\telse",
        "\t\treturn 1",
        "\tfi",
        "}",
    ]


def download_core_initrd_fnc_cmd() -> List[str]:
    """Define helper to download code initrd deb package."""
    return [
        "# Helper to download code initrd deb package",
        "# 1: arch, 2: output dir",
        "download_core_initrd() {",
        "\tapt-get download ubuntu-core-initramfs:${1}",
        "\t# unpack dep to the target dir",
        "\tdpkg -x ubuntu-core-initramfs_*.deb ${2}",
        "}",
    ]


def download_generic_initrd_cmd(target_arch: str) -> List[str]:
    """Download Ubuntu Core initrd deb."""
    return [
        'echo "Getting ubuntu-core-initrd...."',
        # only download u-c-initrd deb if needed
        "if [ ! -e ${UC_INITRD_DEB} ]; then",
        f"\tdownload_core_initrd {target_arch} ${{UC_INITRD_DEB}}",
        "fi",
    ]


def download_snap_bootstrap_fnc_cmd() -> List[str]:
    """Define helper to download snap-bootstrap from snapd deb package."""
    return [
        "# Helper to download snap-bootstrap from snapd deb package",
        "# 1: arch, 2: output dir",
        "download_snap_bootstrap() {",
        "\tapt-get download snapd:${1}",
        "\t# unpack dep to the target dir",
        "\tdpkg -x snapd_*.deb ${2}",
        "}",
    ]


def download_snap_bootstrap_cmd(target_arch: str) -> List[str]:
    """Download snap-bootstrap deb."""
    return [
        'echo "Getting snapd deb for snap bootstrap..."',
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        "if [ ! -e ${SNAPD_UNPACKED_SNAP} ]; then",
        f"\tdownload_snap_bootstrap {target_arch} ${{SNAPD_UNPACKED_SNAP}}",
        "fi",
    ]


def clone_zfs_cmd(enable_zfs: bool, dest_dir: str) -> List[str]:
    """Clone zfs git repository if needed."""
    if enable_zfs:
        return [
            f"if [ ! -d {dest_dir}/zfs ]; then",
            '\techo "cloning zfs..."',
            " ".join(
                [
                    "\tgit",
                    "clone",
                    "--depth=1",
                    _ZFS_URL,
                    f"{dest_dir}/zfs",
                    "-b",
                    "master",
                ]
            ),
            "fi",
        ]
    return [
        'echo "zfs is not enabled"',
    ]


def clean_old_build_cmd(dest_dir: str) -> List[str]:
    """Clean previous build."""
    return [
        'echo "Cleaning previous build first..."',
        f"[ -e {dest_dir}/modules ] && rm -rf {dest_dir}/modules",
        f"[ -L {dest_dir}/lib/modules ] && rm -rf {dest_dir}/lib/modules",
    ]


def _do_base_config_cmd(
    make_cmd: List[str],
    config_file: Optional[str],
    config_flavour: Optional[str],
    defconfig: List[str],
    dest_dir: str,
) -> List[str]:
    # if the parts build dir already contains a .config file, use it
    cmd = [
        'echo "Preparing config..."',
        f"if [ ! -e {dest_dir}/.config ]; then",
    ]

    # if kconfigfile is provided use that
    # elif kconfigflavour is provided, assemble the ubuntu.flavour config
    # otherwise use defconfig to seed the base config
    if config_file:
        cmd.append(f"\tcp {config_file} {dest_dir}/.config")
    elif config_flavour:
        logger.info("Using ubuntu config flavour %s", config_flavour)
        cmd.extend(
            [
                '\techo "Assembling Ubuntu config..."',
                "\tbranch=$(cut -d'.' -f 2- < ${KERNEL_SRC}/debian/debian.env)",
                "\tbaseconfigdir=${KERNEL_SRC}/debian.${branch}/config",
                "\tarchconfigdir=${KERNEL_SRC}/debian.${branch}/config/${DEB_ARCH}",
                "\tcommonconfig=${baseconfigdir}/config.common.ports",
                "\tubuntuconfig=${baseconfigdir}/config.common.ubuntu",
                "\tarchconfig=${archconfigdir}/config.common.${DEB_ARCH}",
                f"\tflavourconfig=${{archconfigdir}}/config.flavour.{config_flavour}",
                f"\tcat ${{commonconfig}} ${{ubuntuconfig}} ${{archconfig}} ${{flavourconfig}} > {dest_dir}/.config 2>/dev/null",
            ]
        )
    else:
        make_cmd = make_cmd.copy()
        make_cmd[1] = "-j1"  # FIXME: make this more robust
        cmd.append(f'\t{" ".join(make_cmd + defconfig)}')

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


def get_configure_command(
    make_cmd: List[str],
    config_file: Optional[str],
    config_flavour: Optional[str],
    defconfig: List[str],
    configs: Optional[List[str]],
    dest_dir: str,
) -> List[str]:
    """Configure the kernel."""
    return [
        *_do_base_config_cmd(
            make_cmd=make_cmd,
            config_file=config_file,
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


def _do_check_initrd(builtin: List[str], modules: List[str], initrd_modules: List[str]):
    # check all config items are either builtin or part of initrd as modules
    msg = (
        "**** WARNING **** WARNING **** WARNING **** WARNING ****\n"
        "The following features are deemed boot essential for\n"
        "ubuntu core, consider making them static[=Y] or adding\n"
        "the corresponding module to initrd:\n"
    )
    missing = []

    for code in _required_boot:
        opt = f"CONFIG_{code.upper()}"
        if opt in builtin:
            continue
        if opt in modules and code in initrd_modules:
            continue
        missing.append(opt)

    if missing:
        warn = f"\n{msg}\n"
        for opt in missing:
            warn += f"{opt}\n"
        logger.warning(warn)


def check_new_config(config_path: str, initrd_modules: List[str]):
    """Check passed kernel config and initrd modules for required dependencies."""
    print("Checking created config...")
    builtin, modules = _do_parse_config(config_path)
    _do_check_config(builtin, modules)
    _do_check_initrd(builtin, modules, initrd_modules)


def call_check_config_cmd(dest_dir: str) -> List[str]:
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
                "${initrd_installed_kernel_modules}",
                "${initrd_configured_kernel_modules}",
            ]
        ),
    ]


def get_build_command(make_cmd: List[str], targets: List[str]) -> List[str]:
    """Build the kernel."""
    return [
        'echo "Building kernel..."',
        " ".join(make_cmd + targets),
    ]


def get_zfs_build_commands(
    enable_zfs: bool, arch_triplet: str, build_dir: str, install_dir: str
) -> List[str]:
    """Include zfs build steps if required."""
    if not enable_zfs:
        return ['echo "Not building zfs modules"']

    return [
        'echo "Building zfs modules..."',
        f"cd {build_dir}/zfs",
        "./autogen.sh",
        " ".join(
            [
                "./configure",
                "--with-linux=${KERNEL_SRC}",
                f"--with-linux-obj={build_dir}",
                "--with-config=kernel",
                f"--host={arch_triplet}",
            ]
        ),
        "make -j$(nproc)",
        f"make install DESTDIR={install_dir}/zfs",
        f'release_version="$(ls {install_dir}/modules)"',
        (
            f"mv {install_dir}/zfs/lib/modules/${{release_version}}/extra "
            f"{install_dir}/modules/${{release_version}}"
        ),
        f"rm -rf {install_dir}/zfs",
        'echo "Rebuilding module dependencies"',
        f"depmod -b {install_dir} ${{release_version}}",
    ]


def get_perf_build_commands(
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


### Install


def parse_kernel_release_cmd(build_dir: str) -> List[str]:
    """Set release from kernel.release file."""
    return [
        'echo "Parsing created kernel release..."',
        f"KERNEL_RELEASE=$(cat {build_dir}/include/config/kernel.release)",
    ]


def copy_vmlinuz_cmd(install_dir: str) -> List[str]:
    """Install kernel image."""
    cmd = [
        'echo "Copying kernel image..."',
        # if kernel already exists, replace it, we are probably re-running
        # build
        " ".join(
            [
                f"[ -e {install_dir}/kernel.img ]",
                "&&",
                f"rm -rf {install_dir}/kernel.img",
            ]
        ),
        " ".join(
            [
                "ln",
                "-f",
                "${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET}",
                f"{install_dir}/${{KERNEL_IMAGE_TARGET}}-${{KERNEL_RELEASE}}",
            ]
        ),
        " ".join(
            [
                "ln",
                "-f",
                "${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET}",
                f"{install_dir}/kernel.img",
            ]
        ),
    ]
    return cmd


def copy_system_map_cmd(build_dir: str, install_dir: str) -> List[str]:
    """Install the system map."""
    cmd = [
        'echo "Copying System map..."',
        " ".join(
            [
                f"[ -e {install_dir}/System.map ]",
                "&&",
                f"rm -rf {install_dir}/System.map*",
            ]
        ),
        " ".join(
            [
                "ln",
                "-f",
                f"{build_dir}/System.map",
                f"{install_dir}/System.map-${{KERNEL_RELEASE}}",
            ]
        ),
    ]
    return cmd


def copy_dtbs_cmd(
    device_trees: Optional[List[str]], install_dir: str
) -> List[str]:
    """Install custom device trees."""
    if not device_trees:
        return [""]

    cmd = [
        'echo "Copying custom dtbs..."',
        f"mkdir -p {install_dir}/dtbs",
    ]

    for dtb in [f"{i}.dtb" for i in device_trees]:
        # Strip any subdirectories
        subdir_index = dtb.rfind("/")
        if subdir_index > 0:
            install_dtb = dtb[subdir_index + 1 :]
        else:
            install_dtb = dtb

        cmd.extend(
            [
                " ".join(
                    [
                        "ln -f",
                        f"${{KERNEL_BUILD_ARCH_DIR}}/dts/{dtb}",
                        f"{install_dir}/dtbs/{install_dtb}",
                    ]
                ),
            ]
        )
    return cmd


def install_config_cmd(build_dir: str, install_dir: str) -> List[str]:
    """Install the kernel configuration file."""
    # install .config as config-$version
    return [
        "",
        'echo "Installing kernel config..."',
        f"ln -f {build_dir}/.config {install_dir}/config-${{KERNEL_RELEASE}}",
    ]


def arrange_install_dir_cmd(install_dir: str) -> List[str]:
    """Final adjustments to installation directory."""
    return [
        "",
        'echo "Finalizing install directory..."',
        # upstream kernel installs under $INSTALL_MOD_PATH/lib/modules/
        # but snapd expects modules/ and firmware/
        f"mv {install_dir}/lib/modules {install_dir}/",
        # remove symlinks modules/*/build and modules/*/source
        f"rm {install_dir}/modules/*/build {install_dir}/modules/*/source",
        # if there is firmware dir, move it to snap root
        # this could have been from stage packages or from kernel build
        " ".join(
            [
                f"[ -d {install_dir}/lib/firmware ]",
                "&&",
                "mv",
                f"{install_dir}/lib/firmware",
                f"{install_dir}",
            ]
        ),
        # create symlinks for modules and firmware for convenience
        f"ln -sf ../modules {install_dir}/lib/modules",
        f"ln -sf ../firmware {install_dir}/lib/firmware",
    ]


def compression_cmd(
    initrd_compression: Optional[str], initrd_compression_options: Optional[List[str]]
) -> str:
    if not initrd_compression:
        return ""
    compressor = _compression_command[initrd_compression]
    options = ""
    if initrd_compression_options:
        options = f"{' '.join(initrd_compression_options)}"
    else:
        options = _compressor_options[initrd_compression]

    cmd = f"{compressor} {options}"
    logger.warning("WARNING: Using custom initrd compressions command: %s", cmd)
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
    globals()[sys.argv[1]](sys.argv[2], sys.argv[3:])
