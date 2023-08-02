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
import subprocess
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
    return [
        "# list of kernel modules to be installed in the initrd",
        f'initrd_installed_kernel_modules="{initrd_installed_kernel_modules}"',
        "# list of kernel modules in the initrd to be auto loaded",
        "# any module in this list implies it will be added to initrd",
        f'initrd_configured_kernel_modules="{initrd_configured_kernel_modules}"',
    ]


def _link_files_fnc_cmd() -> List[str]:
    """Add function to link files."""
    cmd = textwrap.dedent(
        """
        # link files, accept wild cards
        # 1: reference dir, 2: file(s) including wild cards, 3: dst dir
        link_files() {
            if [ "${2}" = "*" ]; then
                for f in $(ls ${1})
                do
                    link_files "${1}" "${f}" "${3}"
                done
                return 0
            fi
            if [ -d "${1}/${2}" ]; then
                for f in $(ls ${1}/${2})
                do
                    link_files "${1}" "${2}/${f}" "${3}"
                done
                return 0
            fi

            local found=""
            for f in $(ls ${1}/${2})
            do
                if [[ -L "${f}" ]]; then
                    local rel_path=$( realpath --no-symlinks --relative-to=${1} ${f} )
                else
                    local rel_path=$( realpath -se --relative-to=${1} ${f} )
                fi
                local dir_path=$(dirname ${rel_path})
                mkdir -p ${3}/${dir_path}
                echo "installing ${f} to ${3}/${dir_path}"
                ln -f ${f} ${3}/${dir_path}
                found="yes"
            done
            if [ "yes" = "${found}" ]; then
                return 0
            else
                return 1
            fi
        }
        """
    )
    return [cmd]


def _download_core_initrd_fnc_cmd() -> List[str]:
    """Define helper to download code initrd deb package."""
    cmd = textwrap.dedent(
        """
        # Helper to download code initrd deb package
        # 1: arch, 2: output dir
        download_core_initrd() {
        	apt-get download ubuntu-core-initramfs:${1}
        	# unpack dep to the target dir
        	dpkg -x ubuntu-core-initramfs_*.deb ${2}
        }
        """
    )
    return [cmd]


def _download_generic_initrd_cmd(target_arch: str) -> List[str]:
    """Download Ubuntu Core initrd deb."""
    cmd = textwrap.dedent(
        f"""
        echo "Getting ubuntu-core-initrd...."
        # only download u-c-initrd deb if needed
        if [ ! -e ${{UC_INITRD_DEB}} ]; then
            download_core_initrd {target_arch} ${{UC_INITRD_DEB}}
        fi
        """
    )
    return [cmd]


def _download_snap_bootstrap_fnc_cmd() -> List[str]:
    """Define helper to download snap-bootstrap from snapd deb package."""
    cmd = textwrap.dedent(
        """
        # Helper to download snap-bootstrap from snapd deb package
        # 1: arch, 2: output dir
        download_snap_bootstrap() {
        	apt-get download snapd:${1}
        	# unpack dep to the target dir
        	dpkg -x snapd_*.deb ${2}
        }
        """
    )
    return [cmd]


def _download_snap_bootstrap_cmd(target_arch: str) -> List[str]:
    """Download snap-bootstrap deb."""
    cmd = textwrap.dedent(
        f"""
        echo "Getting snapd deb for snap bootstrap..."
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        if [ ! -e ${{UC_INITRD_DEB}}/usr/lib/snapd ]; then
            download_snap_bootstrap {target_arch} ${{UC_INITRD_DEB}}
        fi
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
    initrd_stage_firmware: bool,
    build_efi_image: bool,
    initrd_ko_use_workaround: bool,
    initrd_default_compression: str,
    initrd_include_extra_modules_conf: bool,
    initrd_tool_pass_root: bool,
    install_dir: str,
    stage_dir: str,
) -> List[str]:
    cmd_echo = [
        'echo "Generating initrd with ko modules for kernel release: ${KERNEL_RELEASE}"',
    ]

    cmd_prepare_modules_feature = [
        # install required modules to initrd
        'echo "Installing ko modules to initrd..."',
        'install_modules=""',
        'echo "Gathering module dependencies..."',
        'install_modules=""',
        "uc_initrd_feature_kernel_modules=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/kernel-modules",
        "mkdir -p ${uc_initrd_feature_kernel_modules}",
        "initramfs_ko_modules_conf=${uc_initrd_feature_kernel_modules}/extra-modules.conf",
        " ".join(
            [
                "touch",
                "${initramfs_ko_modules_conf}",
            ]
        ),
        " ".join(
            [
                "for",
                "m",
                "in",
                "${initrd_installed_kernel_modules}",
                "${initrd_configured_kernel_modules}",
            ]
        ),
        "do",
        " ".join(["\techo", "${m}", ">>", "${initramfs_ko_modules_conf}"]),
        "done",
        " ".join(
            [
                "[",
                "-e",
                "${initramfs_ko_modules_conf}",
                "]",
                "&&",
                "sort",
                "-fu",
                "${initramfs_ko_modules_conf} -o ${initramfs_ko_modules_conf}",
            ],
        ),
    ]

    if initrd_ko_use_workaround:
        initrd_configured_modules = "$(cat ${initramfs_ko_modules_conf})"
    else:
        initrd_configured_modules = "${initrd_configured_kernel_modules}"

    cmd_prepare_modules_feature.extend(
        [
            'echo "Configuring ubuntu-core-initramfs.conf with supported modules"',
            'echo "If module does not exist, do not include it"',
            "initramfs_conf_dir=${uc_initrd_feature_kernel_modules}/usr/lib/modules-load.d",
            "mkdir -p ${initramfs_conf_dir}",
            "initramfs_conf=${initramfs_conf_dir}/ubuntu-core-initramfs.conf",
            'echo "# configured modules" > ${initramfs_conf}',
            " ".join(
                [
                    "for",
                    "m",
                    "in",
                    initrd_configured_modules,
                ]
            ),
            "do",
            " ".join(
                [
                    "\tif [",
                    "-n",
                    f'"$(modprobe -n -q --show-depends -d {stage_dir} -S "${{KERNEL_RELEASE}}" ${{m}})"',
                    "]; then",
                ]
            ),
            "\t\techo ${m} >> ${initramfs_conf}",
            "\tfi",
            "done",
        ]
    )

    cmd_prepare_initrd_overlay_feature = [
        "uc_initrd_feature_firmware=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/uc-firmware",
        "mkdir -p ${uc_initrd_feature_firmware}",
        "uc_initrd_feature_overlay=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/uc-overlay",
        "mkdir -p ${uc_initrd_feature_overlay}",
        "",
    ]

    # gather firmware files
    if initrd_firmware:
        cmd_prepare_initrd_overlay_feature.extend(
            [
                'echo "Installing initrd overlay firmware..."',
                f"for f in {' '.join(initrd_firmware)}",
                "do",
                # firmware can be from kernel build or from stage
                # firmware from kernel build takes preference
                " ".join(
                    [
                        "\tif !",
                        "link_files",
                        f'"{install_dir}"',
                        '"${f}"',
                        '"${uc_initrd_feature_firmware}/lib"',
                        ";",
                        "then",
                    ]
                ),
                " ".join(
                    [
                        "\t\tif !",
                        "link_files",
                        f'"{stage_dir}"',
                        '"${f}"',
                        '"${uc_initrd_feature_firmware}/lib"',
                        ";",
                        "then",
                    ]
                ),
                '\t\t\techo "Missing firmware [${f}], ignoring it"',
                "\t\tfi",
                "\tfi",
                "done",
                "",
            ]
        )

    # apply overlay if defined
    if initrd_overlay:
        cmd_prepare_initrd_overlay_feature.extend(
            [
                " ".join(
                    [
                        "link_files",
                        f'"{stage_dir}/{initrd_overlay}"',
                        '""',
                        '"${uc_initrd_feature_overlay}"',
                    ]
                ),
                "",
            ]
        )

    # apply overlay addons if defined
    if initrd_addons:
        cmd_prepare_initrd_overlay_feature.extend(
            [
                'echo "Installing initrd addons..."',
                f"for a in {' '.join(initrd_addons)}",
                "do",
                '\techo "Copy overlay: ${a}"',
                " ".join(
                    [
                        "\tlink_files",
                        f'"{stage_dir}"',
                        '"${a}"',
                        '"${uc_initrd_feature_overlay}"',
                    ]
                ),
                "done",
            ],
        )

    cmd_prepare_snap_bootstrap_feature = [
        # install selected snap bootstrap
        'echo "Preparing snap-boostrap initrd feature..."',
        "uc_initrd_feature_snap_bootstratp=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snap-bootstrap",
        "mkdir -p ${uc_initrd_feature_snap_bootstratp}",
        " ".join(
            [
                "link_files",
                '"${UC_INITRD_DEB}"',
                '"usr/lib/snapd/snap-bootstrap"',
                '"${uc_initrd_feature_snap_bootstratp}"',
            ]
        ),
        " ".join(
            [
                "link_files",
                '"${UC_INITRD_DEB}"',
                '"usr/lib/snapd/info"',
                '"${uc_initrd_feature_snap_bootstratp}"',
            ]
        ),
        " ".join(
            [
                "cp",
                "${UC_INITRD_DEB}/usr/lib/snapd/info",
                f"{install_dir}/snapd-info",
            ]
        ),
    ]

    cmd_create_initrd = [
        f"if compgen -G {install_dir}/initrd.img* > /dev/null; then",
        f"\trm -rf {install_dir}/initrd.img*",
        "fi",
    ]

    cmd_create_initrd.extend(
        [
            "",
            "",
            " ".join(
                ["ubuntu_core_initramfs=${UC_INITRD_DEB}/usr/bin/ubuntu-core-initramfs"]
            ),
        ],
    )

    # ubuntu-core-initramfs does not support configurable compression command
    # we still want to support this as configurable option though.
    comp_command = _compression_cmd(
        initrd_compression=initrd_compression,
        initrd_compression_options=initrd_compression_options,
    )
    if comp_command:
        cmd_create_initrd.extend(
            [
                "",
                'echo "Updating compression command to be used for initrd"',
                " ".join(
                    [
                        "sed",
                        "-i",
                        f"'s/{initrd_default_compression}/{comp_command}/g'",
                        "${ubuntu_core_initramfs}",
                    ],
                ),
            ]
        )
    cmd_create_initrd.extend(
        [
            'echo "Workaround for bug in ubuntu-core-initramfs"',
            " ".join(
                [
                    "for",
                    "feature",
                    "in",
                    "kernel-modules",
                    "snap-bootstrap",
                    "uc-firmware",
                    "uc-overlay",
                ],
            ),
            "do",
            " ".join(
                [
                    "\tlink_files",
                    '"${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/${feature}"',
                    '"*"',
                    '"${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/main"',
                ],
            ),
            "done",
        ]
    )

    if initrd_include_extra_modules_conf:
        cmd_create_initrd.extend(
            [
                " ".join(
                    [
                        "cp",
                        "${initramfs_ko_modules_conf}",
                        "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/modules/main/extra-modules.conf",
                    ],
                ),
                "",
            ],
        )

    firmware_dir = f"{install_dir}/lib/firmware"
    if initrd_stage_firmware:
        firmware_dir = f"{stage_dir}/firmware"

    cmd_create_initrd.extend(
        [
            "",
            " ".join(
                [
                    f'[ ! -d "{firmware_dir}" ]',
                    "&&",
                    f'echo -e "firmware directory {firmware_dir} does not exist, consider using'
                    ' initrd-stage-firmware: true/false option"',
                    "&&",
                    "exit 1",
                ]
            ),
            "",
        ],
    )
    if initrd_tool_pass_root:
        build_initrd = [
            "${ubuntu_core_initramfs}",
            "create-initrd",
            "--root",
            "${UC_INITRD_DEB}",
        ]
    else:
        build_initrd = [
            "${ubuntu_core_initramfs}",
            "create-initrd",
        ]

    build_initrd.extend(
        [
            "--kernelver=${KERNEL_RELEASE}",
            "--kerneldir",
            f"{stage_dir}/modules/${{KERNEL_RELEASE}}",
            "--firmwaredir",
            firmware_dir,
            "--skeleton",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs",
            # "--feature",
            # "kernel-modules",
            # "snap-bootstrap",
            # "uc-firmware",
            # "uc-overlay",
            "--output",
            f"{install_dir}/initrd.img",
        ],
    )
    cmd_create_initrd.extend(
        [
            " ".join(build_initrd),
        ],
    )
    cmd_create_initrd.extend(
        [
            f"ln $(ls {install_dir}/initrd.img*) {install_dir}/initrd.img",
        ],
    )
    if build_efi_image:
        cmd_create_initrd.extend(
            [
                "",
                'echo "Building kernel.efi"',
                "stub_p=$(find ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/efi/ -maxdepth 1 -name 'linux*.efi.stub' -printf '%f\\n')",
                " ".join(
                    [
                        "${ubuntu_core_initramfs}",
                        "create-efi",
                        "--kernelver=${KERNEL_RELEASE}",
                        "--stub",
                        "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/efi/${stub_p}",
                        "--sbat",
                        "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/efi/sbat.txt",
                        "--key",
                        "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key",
                        "--cert",
                        "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem",
                        "--initrd",
                        f"{install_dir}/initrd.img",
                        "--kernel",
                        f"{install_dir}/${{KERNEL_IMAGE_TARGET}}",
                        "--output",
                        f"{install_dir}/kernel.efi",
                    ],
                ),
                f"ln $(ls {install_dir}/kernel.efi*) {install_dir}/kernel.efi",
            ],
        )

    return [
        *cmd_echo,
        *cmd_prepare_modules_feature,
        "",
        *cmd_prepare_initrd_overlay_feature,
        "",
        *cmd_prepare_snap_bootstrap_feature,
        "",
        'echo "Create new initrd..."',
        *cmd_create_initrd,
    ]


# pylint: disable-next=too-many-arguments
def get_build_commands(
    target_arch: str,
    initrd_modules: Optional[List[str]],
    initrd_configured_modules: Optional[List[str]],
    initrd_compression: Optional[str],
    initrd_compression_options: Optional[List[str]],
    initrd_firmware: Optional[List[str]],
    initrd_addons: Optional[List[str]],
    initrd_overlay: Optional[str],
    initrd_stage_firmware: bool,
    build_efi_image: bool,
    initrd_ko_use_workaround: bool,
    initrd_default_compression: str,
    initrd_include_extra_modules_conf: bool,
    initrd_tool_pass_root: bool,
    install_dir: str,
    stage_dir: str,
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
        *_download_core_initrd_fnc_cmd(),
        "",
        *_download_generic_initrd_cmd(target_arch=target_arch),
        "",
        *_download_snap_bootstrap_fnc_cmd(),
        "",
        *_download_snap_bootstrap_cmd(target_arch=target_arch),
        "",
        *_get_initrd_install_command(
            initrd_compression=initrd_compression,
            initrd_compression_options=initrd_compression_options,
            initrd_firmware=initrd_firmware,
            initrd_addons=initrd_addons,
            initrd_overlay=initrd_overlay,
            initrd_stage_firmware=initrd_stage_firmware,
            build_efi_image=build_efi_image,
            initrd_ko_use_workaround=initrd_ko_use_workaround,
            initrd_default_compression=initrd_default_compression,
            initrd_include_extra_modules_conf=initrd_include_extra_modules_conf,
            initrd_tool_pass_root=initrd_tool_pass_root,
            install_dir=install_dir,
            stage_dir=stage_dir,
        ),
        'echo "Initramfs build finished!"',
    ]


### Install


def _parse_kernel_release_cmd(stage_dir: str) -> List[str]:
    """Set kernel release from module/<release> directory name."""
    return [
        'echo "Parsing kernel release..."',
        f"KERNEL_RELEASE=$(ls {stage_dir}/modules)",
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


# pylint: disable-next=too-many-arguments
def _get_initrd_install_command(
    initrd_compression: Optional[str],
    initrd_compression_options: Optional[List[str]],
    initrd_firmware: Optional[List[str]],
    initrd_addons: Optional[List[str]],
    initrd_overlay: Optional[str],
    initrd_stage_firmware: bool,
    build_efi_image: bool,
    initrd_ko_use_workaround: bool,
    initrd_default_compression: str,
    initrd_include_extra_modules_conf: bool,
    initrd_tool_pass_root: bool,
    install_dir: str,
    stage_dir: str,
) -> List[str]:

    return [
        'echo "building initramfs..."',
        "",
        *_parse_kernel_release_cmd(stage_dir=stage_dir),
        "",
        *_make_initrd_cmd(
            initrd_compression=initrd_compression,
            initrd_compression_options=initrd_compression_options,
            initrd_firmware=initrd_firmware,
            initrd_addons=initrd_addons,
            initrd_overlay=initrd_overlay,
            initrd_stage_firmware=initrd_stage_firmware,
            build_efi_image=build_efi_image,
            initrd_ko_use_workaround=initrd_ko_use_workaround,
            initrd_default_compression=initrd_default_compression,
            initrd_include_extra_modules_conf=initrd_include_extra_modules_conf,
            initrd_tool_pass_root=initrd_tool_pass_root,
            install_dir=install_dir,
            stage_dir=stage_dir,
        ),
        "",
    ]


### build dependencies


def add_snappy_ppa(with_sudo=False) -> None:  # noqa: C901
    """Setup snappy ppa on the host"""
    # Add ppa necessary to build initrd.
    # TODO: reimplement once snapcraft allows to the plugins to add custom ppa.
    # For the moment we need to handle this as part of the get_build_packages()
    # call and add ppa manually.

    # Building of the initrd requires custom tools available in
    # ppa:snappy-dev/image.
    if with_sudo:
        cmd = ["sudo", "apt-get", "install", "-y", "software-properties-common"]
    else:
        cmd = ["apt-get", "install", "-y", "software-properties-common"]
    proc = subprocess.run(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise ValueError(f"error installing package: {proc.stderr.decode().strip()}")

    proc = subprocess.run(
        ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    if not proc.stdout or proc.stdout.decode().find("snappy-dev/image/ubuntu") == -1:
        # check if we need to import key
        try:
            proc = subprocess.run(
                ["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
            )
        except subprocess.CalledProcessError as error:
            # Export shouldn't exit with failure based on testing
            raise ValueError(
                f"error to check for key={_SNAPPY_DEV_KEY_FINGERPRINT}: {error.output}"
            ) from error

        apt_key_output = proc.stdout.decode()
        if "BEGIN PGP PUBLIC KEY BLOCK" in apt_key_output:
            logger.info("key for ppa:snappy-dev/image already imported")

        if with_sudo:
            cmd = ["sudo", "apt-key"]
        else:
            cmd = ["apt-key"]

        cmd.extend(
            [
                "adv",
                "--keyserver",
                "keyserver.ubuntu.com",
                "--recv-keys",
                _SNAPPY_DEV_KEY_FINGERPRINT,
            ],
        )

        if "nothing exported" in apt_key_output:
            logger.info("importing key for ppa:snappy-dev/image")
            # first import key for the ppa
            try:
                subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    check=True,
                )
            except subprocess.CalledProcessError as error:
                raise ValueError(
                    f"Failed to add ppa key: {_SNAPPY_DEV_KEY_FINGERPRINT}: {error.output}"
                ) from error

        # add ppa itself
        logger.warning("adding ppa:snappy-dev/image to handle initrd builds")
        if with_sudo:
            cmd = ["sudo", "add-apt-repository", "-y", "ppa:snappy-dev/image"]
        else:
            cmd = ["add-apt-repository", "-y", "ppa:snappy-dev/image"]
        subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=True,
        )
