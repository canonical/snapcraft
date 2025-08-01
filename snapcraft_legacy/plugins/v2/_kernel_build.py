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


default_kernel_image_target = {
    "amd64": "bzImage",
    "armhf": "zImage",
    "arm64": "Image",
    "powerpc": "uImage",
    "ppc64el": "vmlinux.strip",
    "s390x": "bzImage",
    "riscv64": "Image",
}

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


def _clone_zfs_cmd(enable_zfs: bool) -> List[str]:
    """Clone zfs git repository if needed."""
    if enable_zfs:
        return [
            textwrap.dedent(
                f"""
                if [ ! -d "${{CRAFT_PART_BUILD}}/zfs" ]; then
                    echo "cloning zfs..."
                    git clone --depth=1 {_ZFS_URL} "${{CRAFT_PART_BUILD}}/zfs" -b master
                fi
                """
            )
        ]
    return [
        'echo "zfs is not enabled"',
    ]


def _clean_old_build_cmd() -> List[str]:
    """Clean previous build."""
    return [
        textwrap.dedent(
            """
            echo "Cleaning previous build first..."
            [ -e "${CRAFT_PART_INSTALL}/modules" ] && rm -rf "${CRAFT_PART_INSTALL}/modules"
            [ -L "${CRAFT_PART_INSTALL}/lib/modules" ] && rm -rf "${CRAFT_PART_INSTALL}/lib/modules"
            """
        )
    ]


def _do_base_config_cmd(
    make_arg: List[str],
    config_flavour: str,
    defconfig: Optional[List[str]],
) -> List[str]:
    # if the parts build dir already contains a .config file, use it
    # if kconfigflavour is provided, assemble the ubuntu.flavour config
    # otherwise use defconfig to seed the base config
    if defconfig:
        logger.info("Using defconfig: %s", defconfig)
        conf_cmd = textwrap.dedent(
            f"""
            echo "Preparing config..."
            if [ ! -e "${{CRAFT_PART_BUILD}}/.config" ]; then
                make \\
                    -j1 \\
                    {" ".join(make_arg)} \\
                    {" ".join(defconfig)}
            fi
            """
        )
    else:
        logger.info("Using ubuntu config flavour %s", config_flavour)
        conf_cmd = textwrap.dedent(
            f"""
            echo "Preparing config..."
            if [ ! -e "${{CRAFT_PART_BUILD}}/.config" ]; then
                echo "Assembling Ubuntu config..."
                if [ -f "${{KERNEL_SRC}}/debian/rules" ] && [ -x "${{KERNEL_SRC}}/debian/rules" ]; then
                    # Generate Ubuntu kernel configs
                    pushd "${{KERNEL_SRC}}"
                    fakeroot debian/rules clean genconfigs || true
                    popd

                    # Pick the right kernel .config for the target arch and flavour
                    ubuntuconfig="${{KERNEL_SRC}}/CONFIGS/${{DEB_ARCH}}-config.flavour.{config_flavour}"
                    cat "${{ubuntuconfig}}" > "${{CRAFT_PART_BUILD}}/.config"

                    # Clean up kernel source directory
                    pushd "${{KERNEL_SRC}}"
                    fakeroot debian/rules clean
                    rm -rf CONFIGS/
                    popd
                fi
            fi
            """
        )
    return [conf_cmd]


def _do_patch_config_cmd(configs: Optional[List[str]]) -> List[str]:
    # prepend the generated file with provided kconfigs
    #  - concat kconfigs to buffer
    #  - read current .config and append
    #  - write out to disk
    if not configs:
        return []

    config = "\\n".join(configs)

    # note that prepending and appending the overrides seems
    # only way to convince all kbuild versions to pick up the
    # configs during oldconfig in .config
    return [
        textwrap.dedent(
            f"""
            echo "Applying extra config...."
            echo -e '{config}' > "${{CRAFT_PART_BUILD}}/.config_snap"
            cat "${{CRAFT_PART_BUILD}}/.config" >> "${{CRAFT_PART_BUILD}}/.config_snap"
            echo -e '{config}' >> "${{CRAFT_PART_BUILD}}/.config_snap"
            mv "${{CRAFT_PART_BUILD}}/.config_snap" "${{CRAFT_PART_BUILD}}/.config"
            """
        )
    ]


def _do_remake_config_cmd(make_arg: List[str]) -> List[str]:
    # update config to include kconfig amendments using oldconfig
    return [
        textwrap.dedent(
            f"""
            echo "Remaking oldconfig...."
            bash -c 'yes "" || true' | make -j1 {' '.join(make_arg)} oldconfig
            """
        )
    ]


def _get_configure_command(
    make_arg: List[str],
    config_flavour: str,
    defconfig: Optional[List[str]],
    configs: Optional[List[str]],
) -> List[str]:
    """Configure the kernel."""
    return [
        *_do_base_config_cmd(
            make_arg=make_arg,
            config_flavour=config_flavour,
            defconfig=defconfig,
        ),
        *_do_patch_config_cmd(configs=configs),
        *_do_remake_config_cmd(make_arg=make_arg),
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


def _call_check_config_cmd() -> List[str]:
    """Invoke the python interpreter and execute check_new_config()."""
    return [
        textwrap.dedent(
            f"""
            echo "Checking config for expected options..."
            {sys.executable} \\
                -I {os.path.abspath(__file__)} \\
                    check_new_config "${{CRAFT_PART_BUILD}}/.config"
            """
        ),
    ]


def _get_build_command(
    make_arg: List[str], make_targets: List[str], config_flavour: str
) -> List[str]:
    """Build the kernel."""
    # if config_flavour is used, determine ABI version and set KERNELRELEASE name
    if config_flavour:
        return [
            textwrap.dedent(
                """
                echo "Gathering release information"
                DEBIAN="${KERNEL_SRC}/debian"
                src_pkg_name=$(sed -n '1s/^\\(.*\\) (.*).*$/\\1/p' "${DEBIAN}/changelog")
                release=$(sed -n '1s/^'"${src_pkg_name}"'.*(\\(.*\\)-.*).*$/\\1/p' "${DEBIAN}/changelog")
                revisions=$(sed -n 's/^'"${src_pkg_name}"'\\ .*('"${release}"'-\\(.*\\)).*$/\\1/p' "${DEBIAN}/changelog" | tac)
                revision=$(echo ${revisions} | awk '{print $NF}')
                abinum=$(echo ${revision} | sed -r -e 's/([^\\+~]*)\\.[^\\.]+(~.*)?(\\+.*)?$/\\1/')
                abi_release="${release}-${abinum}"
                uploadnum=$(echo ${revision} | sed -r -e 's/[^\\+~]*\\.([^\\.~]+(~.*)?(\\+.*)?$)/\\1/')
                """
            ),
            textwrap.dedent(
                f"""
                echo "Building kernel..."
                # shellcheck disable=SC2086 #SC2086 does not apply as ${{KERNEL_IMAGE_TARGET}} is single word
                make \\
                    -j "$(nproc)" \\
                    {" ".join(make_arg)} \\
                    KERNELVERSION="${{abi_release}}-{config_flavour}" \\
                    CONFIG_DEBUG_SECTION_MISMATCH=y \\
                    KBUILD_BUILD_VERSION="${{uploadnum}}" \\
                    LOCALVERSION= localver-extra= \\
                    CFLAGS_MODULE="-DPKG_ABI=${{abinum}}" \\
                    {" ".join(make_targets)}
                """
            ),
        ]

    return [
        textwrap.dedent(
            f"""
            echo "Building kernel..."
            # shellcheck disable=SC2086 #SC2086 does not apply as ${{KERNEL_IMAGE_TARGET}} is single word
            make \\
                -j "$(nproc)" \\
                {" ".join(make_arg)} \\
                {" ".join(make_targets)}
            """
        ),
    ]


def _get_zfs_build_commands(enable_zfs: bool) -> List[str]:
    """Include zfs build steps if required."""
    if not enable_zfs:
        return ['echo "Not building zfs modules"']

    return [
        textwrap.dedent(
            """
            echo "Building zfs modules..."
            cd "${CRAFT_PART_BUILD}/zfs"
            ./autogen.sh
            ./configure --with-linux="${KERNEL_SRC}" \\
                        --with-linux-obj="${CRAFT_PART_BUILD}" \\
                        --with-config=kernel \\
                        --host="${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
            make -j "$(nproc)"
            make install DESTDIR="${CRAFT_PART_INSTALL}/zfs"
            release_version="$(ls "${CRAFT_PART_INSTALL}/modules")"
            mv "${CRAFT_PART_INSTALL}/zfs/lib/modules/${release_version}/extra" \\
               "${CRAFT_PART_INSTALL}/modules/${release_version}"
            rm -rf "${CRAFT_PART_INSTALL}/zfs"
            echo "Rebuilding module dependencies"
            depmod -b "${CRAFT_PART_INSTALL}" "${release_version}"
            """
        ),
    ]


def _get_perf_build_commands(enable_perf: bool) -> List[str]:
    """Build perf binary if enabled."""
    if not enable_perf:
        return ['echo "Not building perf binary"']

    return [
        textwrap.dedent(
            """
            echo "Building perf binary..."
            mkdir -p "${CRAFT_PART_BUILD}/tools/perf
            # Override source and build directories
            make -j "$(nproc)" \\
                 -C "${KERNEL_SRC}/tools/perf" \\
                 O="${CRAFT_PART_BUILD}/tools/perf"
            install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" \\
                            "${CRAFT_PART_INSTALL}"/bin/perf
            """
        ),
    ]


def get_build_commands(
    kernel_arch: str,
    config_flavour: str,
    defconfig: Optional[List[str]],
    configs: Optional[List[str]],
    enable_zfs_support: bool,
    enable_perf: bool,
) -> List[str]:
    """Get build command"""

    logger.info("Initializing build env...")
    # we might be building out of tree, configure paths
    make_arg = [
        "-C",
        '"${KERNEL_SRC}"',
        'O="${CRAFT_PART_BUILD}"',
    ]

    make_targets = ["${KERNEL_IMAGE_TARGET}", "modules"]
    make_install_targets = [
        "modules_install",
        "INSTALL_MOD_STRIP=1",
        'INSTALL_MOD_PATH="${CRAFT_PART_INSTALL}"',
    ]

    if not kernel_arch == "x86":
        make_targets.append("dtbs")
        make_install_targets.extend(
            ["dtbs_install", 'INSTALL_DTBS_PATH="${CRAFT_PART_INSTALL}/dtbs"']
        )

    # kernel source can be either CRAFT_PART_SRC or CRAFT_PROJECT_DIR
    kernel_src_cmd = textwrap.dedent(
        """
        [ -d "${CRAFT_PART_SRC}/kernel" ] && KERNEL_SRC="${CRAFT_PART_SRC}" || KERNEL_SRC="${CRAFT_PROJECT_DIR}"
        echo "PATH=${PATH}"
        echo "KERNEL_SRC=${KERNEL_SRC}"
        """
    )

    return [
        kernel_src_cmd,
        *_clone_zfs_cmd(
            enable_zfs=enable_zfs_support,
        ),
        "",
        *_clean_old_build_cmd(),
        "",
        *_get_configure_command(
            make_arg=make_arg,
            config_flavour=config_flavour,
            defconfig=defconfig,
            configs=configs,
        ),
        "",
        *_call_check_config_cmd(),
        *_get_build_command(
            make_arg=make_arg,
            make_targets=make_targets,
            config_flavour=config_flavour,
        ),
        *_get_install_command(
            make_arg=make_arg,
            make_install_targets=make_install_targets,
        ),
        "",
        *_get_zfs_build_commands(enable_zfs=enable_zfs_support),
        "",
        *_get_perf_build_commands(enable_perf=enable_perf),
        "",
        'echo "Kernel build finished!"',
    ]


### Install


def _arrange_install_dir_cmd() -> List[str]:
    """Final adjustments to installation directory."""
    return [
        textwrap.dedent(
            """
            echo "Finalizing install directory..."
            # upstream kernel installs under $INSTALL_MOD_PATH/lib/modules/
            # but snapd expects modules/ and firmware/
            mv "${CRAFT_PART_INSTALL}/lib/modules" "${CRAFT_PART_INSTALL}"
            # remove symlinks modules/*/build and modules/*/source
            rm -rf "${CRAFT_PART_INSTALL}"/modules/*/build "${CRAFT_PART_INSTALL}"/modules/*/source
            # if there is firmware dir, move it to snap root
            # this could have been from stage packages or from kernel build
            [ -d "${CRAFT_PART_INSTALL}/lib/firmware" ] && mv "${CRAFT_PART_INSTALL}/lib/firmware" "${CRAFT_PART_INSTALL}"
            # create symlinks for modules and firmware for convenience
            ln -sf ../modules "${CRAFT_PART_INSTALL}/lib/modules"
            ln -sf ../firmware "${CRAFT_PART_INSTALL}/lib/firmware"
            """
        ),
    ]


def _get_post_install_cmd() -> List[str]:
    return [
        textwrap.dedent(
            """
            echo "Parsing created kernel release..."
            KERNEL_RELEASE=$(cat "${CRAFT_PART_BUILD}/include/config/kernel.release")

            echo "Copying kernel image..."
            # if kernel.img already exists, replace it, we are probably re-running
            # build
            [ -e "${CRAFT_PART_INSTALL}/kernel.img" ] && rm -rf "${CRAFT_PART_INSTALL}/kernel.img"
            mv "${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET}" "${CRAFT_PART_INSTALL}/kernel.img"

            echo "Copying System map..."
            [ -e "${CRAFT_PART_INSTALL}/System.map" ] && rm -rf "${CRAFT_PART_INSTALL}"/System.map*
            ln -f "${CRAFT_PART_BUILD}/System.map" "${CRAFT_PART_INSTALL}/System.map-${KERNEL_RELEASE}"

            echo "Installing kernel config..."
            ln -f "${CRAFT_PART_BUILD}/.config" "${CRAFT_PART_INSTALL}/config-${KERNEL_RELEASE}"
            """
        ),
    ]


def _get_install_command(
    make_arg: List[str],
    make_install_targets: List[str],
) -> List[str]:
    # install to installdir
    install_cmd = textwrap.dedent(
        f"""
        echo "Installing kernel build..."
        make \\
            -j "$(nproc)" \\
            {" ".join(make_arg)} \\
            CONFIG_PREFIX="${{CRAFT_PART_INSTALL}}" \\
            {" ".join(make_install_targets)}
        """
    )

    return [
        install_cmd,
        # add post-install steps
        *_get_post_install_cmd(),
        *_arrange_install_dir_cmd(),
    ]


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
