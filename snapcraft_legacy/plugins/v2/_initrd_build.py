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
import textwrap
from typing import List, Optional

logger = logging.getLogger(__name__)


_compression_command = {"gz": "gzip", "lz4": "lz4", "xz": "xz", "zstd": "zstd"}
_compressor_options = {"gz": "-7", "lz4": "-l -9", "xz": "-7", "zstd": "-1 -T0"}

_SNAPPY_DEV_KEY_FINGERPRINT = "F1831DDAFC42E99D"


def _get_initrd_kernel_modules(
    initrd_modules: Optional[List[str]], initrd_configured_modules: Optional[List[str]]
) -> List[str]:
    """Collect a list of kernel modules to add to the initrd."""
    initrd_installed_kernel_modules = ""
    initrd_configured_kernel_modules = ""

    if initrd_modules:
        initrd_installed_kernel_modules = f"{' '.join(initrd_modules)}"
    if initrd_configured_modules:
        initrd_configured_kernel_modules = f"{' '.join(initrd_configured_modules)}"
    installed_modules = textwrap.dedent(
        f"""
        # list of kernel modules to be installed in the initrd
        initrd_installed_kernel_modules="{initrd_installed_kernel_modules}"
        """
    )

    configured_modules = textwrap.dedent(
        f"""
        # list of kernel modules in the initrd to be auto loaded
        # any module in this list implies it will be added to initrd
        initrd_configured_kernel_modules="{initrd_configured_kernel_modules}"
        """
    )
    return [installed_modules, configured_modules]


def _link_files_fnc_cmd() -> List[str]:
    """Add function to link files."""
    cmd = textwrap.dedent(
        """
        # link files helper, accept wild cards
        # 1: reference dir, 2: file(s) including wild cards, 3: dst dir
        # 4: quiet mode [ "-quiet" ] (optional)
        link_files() {
            set +x
            link_files_impl "${@}"
            local retVal=$?
            set -x
            return ${retVal}
        }

        # link files helper implementation, accept wild cards
        # 1: reference dir, 2: file(s) including wild cards, 3: dst dir
        # 4: quiet mode [ "-quiet" ] (optional)
        link_files_impl() {
            if [ -z "${2}" ]; then
                return 0
            fi
            local quiet="${4:--noisy}"
            local f
            if [ "${2}" = "*" ]; then
                while IFS= read -r -d $'\\0' f
                do
                    link_files_impl "${1}" "${f}" "${3}" "${quiet}"
                done < <(find "${1}" -maxdepth 1 -mindepth 1 -printf '%P\\0')
                return 0
            fi
            if [ -d "${1}/${2}" ]; then
                while IFS= read -r -d $'\\0' f
                do
                    link_files_impl "${1}" "${2}/${f}" "${3}" "${quiet}"
                done < <(find "${1}/${2}" -maxdepth 1 -mindepth 1 -printf '%P\\0')
                return 0
            fi

            local found searchdir basename rel_path dir_path
            searchdir=$(dirname "${2}")
            basename=$(basename "${2}")
            if ! compgen -G "${1}/${searchdir}" > /dev/null; then
                echo "search pattern <${1}/${searchdir}> <${basename}> does not exist"
                return 1
            fi
            # shellcheck disable=SC2086 # SC2086 does not apply, searchdir can contain wild cards, it cannot be quoted
            while IFS= read -r -d $'\\0' f
            do
                if [[ -d "${f}" ]]; then
                    link_files_impl "${1}" "${rel_path}" "${3}" "${quiet}"
                else
                    if [[ -L "${f}" ]]; then
                        rel_path=$( realpath --no-symlinks --relative-to="${1}" "${f}" )
                    else
                        rel_path=$( realpath -se --relative-to="${1}" "${f}" )
                    fi
                    dir_path=$(dirname "${rel_path}")
                    mkdir -p "${3}/${dir_path}"
                    [ "${quiet}" != "-quiet" ] && echo "installing ${f} to ${3}/${dir_path}"
                    ln -f "${f}" "${3}/${dir_path}"
                fi
                found="yes"
            done < <(find "${1}"/${searchdir} -maxdepth 1 -mindepth 1 -name "${basename}" -printf '%p\\0')
            if [ "yes" = "${found:-}" ]; then
                return 0
            else
                return 1
            fi
        }
        """
    )
    return [cmd]


def _setup_ubuntu_base_chroot_fnc_cmd() -> List[str]:
    """Add function to set tup chroot from Ubuntu Base."""
    cmd = textwrap.dedent(
        """
        # setup chroot from Ubuntu Base
        # 1: work dir, 2: ubuntu series
        setup_chroot_base() {
            local work_dir="${1}"
            local series="${2}"
            local ubuntu_base="${work_dir}/ubuntu-base-${series}-${CRAFT_ARCH_BUILD_FOR}.tar.gz"
            rm -rf "${ubuntu_base}"
            curl \\
                "https://cdimage.ubuntu.com/ubuntu-base/${series}/daily/current/${series}-base-${CRAFT_ARCH_BUILD_FOR}.tar.gz" \\
                --output "${ubuntu_base}"
            rm -rf "${UC_INITRD_ROOT}"
            mkdir -p "${UC_INITRD_ROOT}"
            tar --extract --file "${ubuntu_base}" --directory "${UC_INITRD_ROOT}"
            cp --no-dereference /etc/resolv.conf "${UC_INITRD_ROOT}/etc/resolv.conf"
            # if not running as root, setup build root env with fakechroot, fakeroot
            if [ "$(whoami)" != "root" ]; then
                cp --no-dereference --recursive \\
                    /usr/lib/"${CRAFT_ARCH_TRIPLET_BUILD_FOR}"/fakechroot \\
                    "${UC_INITRD_ROOT}/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                cp --no-dereference --recursive \\
                    /usr/lib/"${CRAFT_ARCH_TRIPLET_BUILD_FOR}"/libfakeroot \\
                    "${UC_INITRD_ROOT}/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
            fi
            # setup /dev/null as it's used to mask systemd service files
            touch "${UC_INITRD_ROOT}/dev/null"
        }
        """
    )
    return [cmd]


def _chroot_add_snappy_dev_ppa_fnc_cmd() -> List[str]:
    """Add function to add snappy-dev ppa to chroot."""
    cmd = textwrap.dedent(
        """
        # add snappy-dev/image ppa to the chroot
        # import ppa keys for the ppa
        # 1: work dir, 2: ubuntu series, 3: ppa fingerprint
        chroot_add_snappy_dev_ppa() {
            local work_dir="${1}"
            local series="${2}"
            local key_fingerprint="${3}"
            local source_file="snappy-dev-image.sources"
            local key_file="/etc/apt/keyrings/snappy-dev-image.gpg"
            local chroot_home="${UC_INITRD_ROOT}"
            local gnupg_home="${work_dir}/.gnupg"
            local snappy_key="${gnupg_home}/snappy-dev.kbx"
            rm -rf "${gnupg_home}"
            mkdir --mode 700 "${gnupg_home}"
            gpg \\
                --homedir "${gnupg_home}" \\
                --no-default-keyring \\
                --keyring "${snappy_key}" \\
                --keyserver keyserver.ubuntu.com \\
                --recv-keys "${key_fingerprint}"

            mkdir -p "$(dirname "${chroot_home}/${key_file}")"
            rm -rf "${chroot_home:?}/${key_file}"
            gpg \\
                --homedir "${gnupg_home}" \\
                --no-default-keyring \\
                --keyring "${snappy_key}" \\
                --export \\
                --out "${chroot_home}/${key_file}"

            tee "${chroot_home}/etc/apt/sources.list.d/${source_file}" <<EOF
        Types: deb
        URIs: https://ppa.launchpadcontent.net/snappy-dev/image/ubuntu/
        Suites: ${series}
        Components: main
        Signed-By: ${key_file}
        EOF
        }
        """
    )
    return [cmd]


def _chroot_run_cmd_fnc_cmd() -> List[str]:
    """Add helper function to run chroot commands."""
    cmd = textwrap.dedent(
        """
        # clean any existing mounts for the chroot function
        _clean_chroot() {
            local chroot_home="${1}"
            if [ -z "${chroot_home}" ]; then
                echo "Missing chroot home to clean"
                return
            fi
            for m in dev/pts dev/null dev/zero dev/full dev/random dev/urandom dev/tty dev proc run sys
            do
                if grep "${chroot_home}/${m}" /proc/self/mounts > /dev/null; then
                    umount "${chroot_home}/${m}"
                fi
            done
        }

        _chroot_configured="no"
        # setup necessary mounts for the chroot function
        _setup_chroot() {
            if [ "${_chroot_configured}" = "yes" ]; then
                return
            fi
            local chroot_home="${1}"
            for m in proc run sys dev dev/pts dev/null dev/zero dev/full dev/random dev/urandom dev/tty
            do
                mount --bind "/${m}" "${chroot_home}/${m}"
            done
            _chroot_configured="yes"
        }

        # run command in true chroot
        _run_truechroot() {
            local chroot_home="${1}"
            local cmd="${2}"
            trap "_clean_chroot ${chroot_home}" EXIT
            _setup_chroot "${chroot_home}"
            chroot "${chroot_home}" /bin/bash -c "${cmd}"
        }

        # run command in fake chroot
        _run_fakechroot() {
            local chroot_home="${1}"
            local cmd="${2}"
            if [ "${UBUNTU_SERIES}" = "focal" ] || [ "${UBUNTU_SERIES}" = "jammy" ]; then
                ld_path="${LD_LIBRARY_PATH}:/usr/lib/systemd:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/fakechroot:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libfakeroot"
            else
                ld_path="${LD_LIBRARY_PATH}:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/systemd:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/fakechroot:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libfakeroot"
            fi
            LD_LIBRARY_PATH="${ld_path}" fakechroot fakeroot chroot "${chroot_home}" /bin/bash -c "${cmd}"
        }

        # run command with chroot
        # 1: chroot home, 2: command to run
        run_chroot() {
            local chroot_home="${1}"
            local cmd="${2}"
            # use true chroot if we have root permissions
            if [ "$(whoami)" = "root" ]; then
                _run_truechroot "${chroot_home}" "${cmd}"
            else
                _run_fakechroot "${chroot_home}" "${cmd}"
            fi
        }
        """
    )
    return [cmd]


def _setup_initrd_chroot_fnc_cmd() -> List[str]:
    """Define helper fnc to prepare build chroot ."""
    cmd = textwrap.dedent(
        """
        # setup chroot to build Ubuntu Core initrd
        # chroot is based on the Ubuntu Base
        # 1: work dir, 2: ppa fingerprint
        setup_initrd_chroot() {
            set +x
            local work_dir="${1}"
            local ppa_fingerprint="${2}"

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.base" ]; then
                setup_chroot_base "${work_dir}" "${UBUNTU_SERIES}"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.base"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.ppa" ]; then
                run_chroot "${UC_INITRD_ROOT}" "apt-get update"
                run_chroot "${UC_INITRD_ROOT}" "apt-get dist-upgrade -y"
                run_chroot "${UC_INITRD_ROOT}" "apt-get install --no-install-recommends -y ca-certificates debconf-utils libfakeroot lz4 xz-utils zstd"
                if [ "${UBUNTU_SERIES}" = "focal" ] || [ "${UBUNTU_SERIES}" = "jammy" ]; then
                    run_chroot "${UC_INITRD_ROOT}" "apt-get install --no-install-recommends -y systemd"
                else
                    run_chroot "${UC_INITRD_ROOT}" "apt-get install --no-install-recommends -y libsystemd-shared"
                fi
                chroot_add_snappy_dev_ppa "${work_dir}" "${UBUNTU_SERIES}" "${ppa_fingerprint}"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.ppa"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.u-c-i" ]; then
                run_chroot "${UC_INITRD_ROOT}" "apt-get update"
                run_chroot "${UC_INITRD_ROOT}" "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"
                run_chroot "${UC_INITRD_ROOT}" "DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y snapd ubuntu-core-initramfs"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.u-c-i"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.firmware" ]; then
                rm -rf "${UC_INITRD_ROOT}"/usr/lib/firmware/*
                link_files "${KERNEL_FIRMWARE}" "*" "${UC_INITRD_ROOT}/usr/lib/firmware"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.firmware"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.modules" ]; then
                rm -rf "${UC_INITRD_ROOT}"/usr/lib/modules/*
                link_files "${KERNEL_MODULES}" "*" "${UC_INITRD_ROOT}/usr/lib/modules"
                # remove potentially dangling source link
                rm -rf ${UC_INITRD_ROOT}/usr/lib/modules/*/build
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.modules"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.os-release" ]; then
                cp "/snap/${UBUNTU_CORE_BASE}/current/etc/os-release" "${UC_INITRD_ROOT}/etc/os-release"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.os-release"
            fi
            set -x
        }
        """
    )
    return [cmd]


def _check_for_stage_firmware_cmd() -> List[str]:
    """Check stage firmware exists."""
    cmd = textwrap.dedent(
        """
        [ ! -d "${CRAFT_STAGE}/firmware" ] && \\
            echo -e "firmware directory ${CRAFT_STAGE}/firmware does not exist, ensure part building firmware is run before this part." \\
                && exit 1
        """
    )
    return [cmd]


def _setup_initrd_build_env_cmd() -> List[str]:
    """Prepare Ubuntu Core Initrd build chroot."""
    cmd = textwrap.dedent(
        f"""
        echo "Preparing Ubuntu Core Initrd chroot build environment..."
        setup_initrd_chroot "${{CRAFT_PART_SRC}}" "{_SNAPPY_DEV_KEY_FINGERPRINT}"
        """
    )
    return [cmd]


# pylint: disable-next=too-many-arguments, too-many-locals
def _make_initrd_cmd(
    initrd_compression: Optional[str],
    initrd_compression_options: Optional[List[str]],
    initrd_firmware: Optional[List[str]],
    initrd_addons: Optional[List[str]],
    initrd_overlay: Optional[str],
    initrd_ko_use_workaround: bool,
    initrd_default_compression: str,
    build_efi_image: Optional[bool],
    efi_image_key: Optional[str],
    efi_image_cert: Optional[str],
) -> List[str]:
    cmd_echo = [
        'echo "Generating initrd with ko modules for kernel release: ${KERNEL_RELEASE}"',
    ]
    if initrd_ko_use_workaround:
        initrd_configured_modules = "$(cat ${initramfs_ko_modules_conf})"
    else:
        initrd_configured_modules = "${initrd_configured_kernel_modules}"

    cmd_prepare_modules_feature = textwrap.dedent(
        f"""
        echo "Installing ko modules to initrd..."
        # shellcheck disable=SC2034 #SC2034 does not apply as install_modules not be always used
        install_modules=""
        echo "Gathering module dependencies..."
        uc_initrd_feature_kernel_modules=${{UC_INITRD_ROOT}}/usr/lib/ubuntu-core-initramfs/kernel-modules
        mkdir -p "${{uc_initrd_feature_kernel_modules}}"
        initramfs_ko_modules_conf=${{UC_INITRD_ROOT}}/usr/lib/ubuntu-core-initramfs/extra-modules.conf
        rm -rf "${{initramfs_ko_modules_conf}}"
        for m in ${{initrd_installed_kernel_modules}} ${{initrd_configured_kernel_modules}}
        do
            echo "${{m}}" >> "${{initramfs_ko_modules_conf}}"
        done
        [ -e "${{initramfs_ko_modules_conf}}" ] && sort -fu "${{initramfs_ko_modules_conf}}" -o "${{initramfs_ko_modules_conf}}"

        echo "Configuring ubuntu-core-initramfs.conf with supported modules"
        echo "If module does not exist, do not include it"
        initramfs_conf_dir=${{uc_initrd_feature_kernel_modules}}/usr/lib/modules-load.d
        mkdir -p "${{initramfs_conf_dir}}"
        initramfs_conf=${{initramfs_conf_dir}}/ubuntu-core-initramfs.conf
        echo "# configured modules" > "${{initramfs_conf}}"
        # shellcheck disable=SC2013 #SC2013 does not apply as array could be env, or file content
        for m in {initrd_configured_modules}
        do
            if [ -n "$(modprobe -n -q --show-depends -d "${{CRAFT_STAGE}}" -S "${{KERNEL_RELEASE}}" "${{m}}")" ] ; then
                echo "${{m}}" >> "${{initramfs_conf}}"
            fi
        done
        """
    )

    cmd_prepare_initrd_overlay_feature = textwrap.dedent(
        """
        uc_initrd_feature_firmware=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-firmware
        mkdir -p "${uc_initrd_feature_firmware}"
        uc_initrd_feature_overlay=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-overlay
        mkdir -p "${uc_initrd_feature_overlay}"
        """
    )

    # gather firmware files
    cmd_prepare_initrd_overlay_firmware = ""
    if initrd_firmware:
        cmd_prepare_initrd_overlay_firmware = textwrap.dedent(
            f"""
            echo "Installing initrd overlay firmware..."
            for fw in {' '.join(initrd_firmware)}
            do
                # firmware can be from kernel build or from stage
                # firmware from kernel build takes preference
                if ! link_files "${{CRAFT_PART_INSTALL}}" "${{fw}}" "${{uc_initrd_feature_firmware}}/usr/lib" ; then
                    if ! link_files "${{CRAFT_STAGE}}" "${{fw}}" "${{uc_initrd_feature_firmware}}/usr/lib"; then
                        echo "Missing firmware [${{fw}}], ignoring it"
                    fi
                fi
            done
            """
        )

    # apply overlay if defined
    cmd_prepare_initrd_overlay = ""
    if initrd_overlay:
        cmd_prepare_initrd_overlay = textwrap.dedent(
            f"""
            link_files "${{CRAFT_STAGE}}/{initrd_overlay}" "*" "${{uc_initrd_feature_overlay}}"
            """
        )

    # apply overlay addons if defined
    cmd_prepare_initrd_addons = ""
    if initrd_addons:
        cmd_prepare_initrd_addons = textwrap.dedent(
            f"""
            echo "Installing initrd addons..."
            for a in {' '.join(initrd_addons)}
            do
                echo "Copy overlay: ${{a}}"
                link_files "${{CRAFT_STAGE}}" "${{a}}" "${{uc_initrd_feature_overlay}}"
            done
            """
        )

    cmd_prepare_snap_bootstrap_feature = textwrap.dedent(
        """
        # install selected snap bootstrap
        echo "Preparing snap-boostrap initrd feature..."
        uc_initrd_main_lib_snapd=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main/usr/lib/snapd
        [ -e "${uc_initrd_main_lib_snapd}/snap-bootstrap" ] && ln -f "${UC_INITRD_ROOT}/usr/lib/snapd/snap-bootstrap" \\
                                                                     "${uc_initrd_main_lib_snapd}/snap-bootstrap"
        [ -e "${uc_initrd_main_lib_snapd}/info" ] && ln -f "${UC_INITRD_ROOT}/usr/lib/snapd/info" \\
                                                           "${uc_initrd_main_lib_snapd}/info"

        cp "${UC_INITRD_ROOT}/usr/lib/snapd/info" "${CRAFT_PART_INSTALL}/snapd-info"
        """
    )

    cmd_create_initrd = textwrap.dedent(
        """
        if compgen -G "${CRAFT_PART_INSTALL}/initrd.img*" > /dev/null; then
            rm -rf "${CRAFT_PART_INSTALL}"/initrd.img*
        fi
        """
    )

    # ubuntu-core-initramfs does not support configurable compression command
    # we still want to support this as configurable option though.
    comp_command = _compression_cmd(
        initrd_compression=initrd_compression,
        initrd_compression_options=initrd_compression_options,
    )

    cmd_create_initrd_update_compression = ""
    if comp_command:
        cmd_create_initrd_update_compression = textwrap.dedent(
            f"""
            echo "Updating compression command to be used for initrd"
            sed -i 's/{initrd_default_compression}/{comp_command}/g' "${{UC_INITRD_ROOT}}/usr/bin/ubuntu-core-initramfs"',
            """
        )

    cmd_create_initrd_workaround = textwrap.dedent(
        """
        echo "Workaround for bug in ubuntu-core-initramfs"
        for feature in kernel-modules uc-firmware uc-overlay
        do
            link_files "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/${feature}" \\
                "*" \\
                "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main"
        done

        # actual ubuntu-core initramfs build is performed in chroot
        # where tmp is not really tmpfs, avoid excessive use of cp
        # cp "-ar"/"-aR" -> cp "-lR"
        sed -i \\
            -e 's/"cp", "-ar", args./"cp", "-lR", args./g' \\
            -e 's/"cp", "-aR", args./"cp", "-lR", args./g' \\
            ${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs
        if [ "$(whoami)" != "root" ]; then
            # ubuntu-core-initramfs unsets LD_PRELOAD before invoking dracut
            # this leads to escaping of fakechroot, disable this
            sed -i \\
                -e 's/\\(.*\\)proc_env\\["LD_PRELOAD"\\]\\(.*\\)/\\1# proc_env\\["LD_PRELOAD"\\]\\2/g' \\
                ${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs
        fi
        """
    )

    cmd_create_initrd_install_extra_modules = textwrap.dedent(
        """
        if [ -e "${initramfs_ko_modules_conf}" ]; then
            if [ "${UBUNTU_SERIES}" = "focal" ]; then
                cp "${initramfs_ko_modules_conf}" \\
                   "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main/extra-modules.conf"
            else
                cp "${initramfs_ko_modules_conf}" \\
                   "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/modules/main/extra-modules.conf"
            fi
        fi
        """
    )

    cmd_create_initrd_build_initrd = textwrap.dedent(
        """
        rm -rf ${UC_INITRD_ROOT}/boot/initrd*
        run_chroot "${UC_INITRD_ROOT}" \\
                "ubuntu-core-initramfs create-initrd --kernelver "${KERNEL_RELEASE}" --output /boot/initrd.img"
        """
    )

    cmd_create_initrd_install_initrd = textwrap.dedent(
        """
        link_files "${UC_INITRD_ROOT}"/boot "initrd.img*" "${CRAFT_PART_INSTALL}"
        ln -f "$(ls "${CRAFT_PART_INSTALL}"/initrd.img*)" "${CRAFT_PART_INSTALL}"/initrd.img
        """
    )

    cmd_create_initrd_build_efi_setup_keys = ""
    cmd_create_initrd_build_efi = ""
    if build_efi_image:
        signing_key = "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key"
        certificate = "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem"
        if efi_image_key and efi_image_cert:
            signing_key = "/root/efi-signing-key.key"
            certificate = "/root/efi-certificate.pem"
            cmd_create_initrd_build_efi_setup_keys = textwrap.dedent(
                f"""
                cp --link "{efi_image_key}" "${{UC_INITRD_ROOT}}{signing_key}"
                cp --link "{efi_image_cert}" "${{UC_INITRD_ROOT}}{certificate}"
                """
            )

        cmd_create_initrd_build_efi = textwrap.dedent(
            f"""
            echo "Building kernel.efi"
            rm -rf ${{UC_INITRD_ROOT}}/boot/kernel.efi*
            ln -f "${{CRAFT_STAGE}}"/kernel.img "${{UC_INITRD_ROOT}}/boot/kernel.img-${{KERNEL_RELEASE}}"
            run_chroot "${{UC_INITRD_ROOT}}" \\
                    "ubuntu-core-initramfs create-efi \\
                        --kernelver=${{KERNEL_RELEASE}} \\
                        --key {signing_key} \\
                        --cert {certificate} \\
                        --initrd /boot/initrd.img \\
                        --kernel /boot/kernel.img \\
                        --output /boot/kernel.efi"

            echo "Installing created kernel.efi image"
            link_files "${{UC_INITRD_ROOT}}/boot" "kernel.efi*" "${{CRAFT_PART_INSTALL}}"
            ln -f "$(ls "${{CRAFT_PART_INSTALL}}"/kernel.efi*)" "${{CRAFT_PART_INSTALL}}/kernel.efi"
            rm "${{CRAFT_PART_INSTALL}}"/initrd.img*
            """
        )

    return [
        *cmd_echo,
        cmd_prepare_modules_feature,
        cmd_prepare_initrd_overlay_feature,
        cmd_prepare_initrd_overlay_firmware,
        cmd_prepare_initrd_overlay,
        cmd_prepare_initrd_addons,
        cmd_prepare_snap_bootstrap_feature,
        'echo "Create new initrd..."',
        cmd_create_initrd,
        cmd_create_initrd_update_compression,
        cmd_create_initrd_workaround,
        cmd_create_initrd_install_extra_modules,
        cmd_create_initrd_build_initrd,
        cmd_create_initrd_install_initrd,
        cmd_create_initrd_build_efi_setup_keys,
        cmd_create_initrd_build_efi,
    ]


# pylint: disable-next=too-many-arguments
def get_build_commands(
    initrd_modules: Optional[List[str]],
    initrd_configured_modules: Optional[List[str]],
    initrd_compression: Optional[str],
    initrd_compression_options: Optional[List[str]],
    initrd_firmware: Optional[List[str]],
    initrd_addons: Optional[List[str]],
    initrd_overlay: Optional[str],
    initrd_ko_use_workaround: bool,
    initrd_default_compression: str,
    build_efi_image: Optional[bool] = False,
    efi_image_key: Optional[str] = None,
    efi_image_cert: Optional[str] = None,
) -> List[str]:
    """Get build command"""
    return [
        *_get_initrd_kernel_modules(
            initrd_modules=initrd_modules,
            initrd_configured_modules=initrd_configured_modules,
        ),
        "",
        *_link_files_fnc_cmd(),
        "",
        *_setup_ubuntu_base_chroot_fnc_cmd(),
        "",
        *_chroot_add_snappy_dev_ppa_fnc_cmd(),
        "",
        *_chroot_run_cmd_fnc_cmd(),
        "",
        *_setup_initrd_chroot_fnc_cmd(),
        "",
        *_check_for_stage_firmware_cmd(),
        "",
        *_setup_initrd_build_env_cmd(),
        "",
        'echo "building initramfs..."',
        "",
        *_parse_kernel_release_cmd(),
        "",
        *_make_initrd_cmd(
            initrd_compression=initrd_compression,
            initrd_compression_options=initrd_compression_options,
            initrd_firmware=initrd_firmware,
            initrd_addons=initrd_addons,
            initrd_overlay=initrd_overlay,
            initrd_ko_use_workaround=initrd_ko_use_workaround,
            initrd_default_compression=initrd_default_compression,
            build_efi_image=build_efi_image,
            efi_image_key=efi_image_key,
            efi_image_cert=efi_image_cert,
        ),
        'echo "Initramfs build finished!"',
    ]


### Install


def _parse_kernel_release_cmd() -> List[str]:
    """Set kernel release from module/<release> directory name."""
    return [
        'echo "Parsing kernel release..."',
        'KERNEL_RELEASE=$(ls "${CRAFT_STAGE}/modules")',
    ]


def _compression_cmd(
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
