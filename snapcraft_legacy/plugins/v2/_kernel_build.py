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
import subprocess
import sys
import textwrap
from typing import List, Optional

logger = logging.getLogger(__name__)


_compression_command = {"gz": "gzip", "lz4": "lz4", "xz": "xz", "zstd": "zstd"}
_compressor_options = {"gz": "-7", "lz4": "-l -9", "xz": "-7", "zstd": "-1 -T0"}

_ZFS_URL = "https://github.com/openzfs/zfs"
_SNAPPY_DEV_KEY_FINGERPRINT = "F1831DDAFC42E99D"


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


def _get_initrd_kernel_modules(
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
        """
        echo "Getting ubuntu-core-initrd...."
        # only download u-c-initrd deb if needed
        if [ ! -e ${{UC_INITRD_DEB}} ]; then
        	download_core_initrd {arch} ${{UC_INITRD_DEB}}
        fi
        """.format(
            arch=target_arch
        )
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
        """
        echo "Getting snapd deb for snap bootstrap..."
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        if [ ! -e ${{SNAPD_UNPACKED_SNAP}} ]; then
        	download_snap_bootstrap {arch} ${{SNAPD_UNPACKED_SNAP}}
        fi
        """.format(
            arch=target_arch
        )
    )
    return [cmd]


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
        conf_cmd = textwrap.dedent(
            """	echo "Assembling Ubuntu config..."
	branch=$(cut -d'.' -f 2- < ${{KERNEL_SRC}}/debian/debian.env)
	baseconfigdir=${{KERNEL_SRC}}/debian.${{branch}}/config
	archconfigdir=${{KERNEL_SRC}}/debian.${{branch}}/config/${{DEB_ARCH}}
	commonconfig=${{baseconfigdir}}/config.common.ports
	ubuntuconfig=${{baseconfigdir}}/config.common.ubuntu
	archconfig=${{archconfigdir}}/config.common.${{DEB_ARCH}}
	flavourconfig=${{archconfigdir}}/config.flavour.{config_flavour}
    cat ${{commonconfig}} ${{ubuntuconfig}} ${{archconfig}} ${{flavourconfig}} \
> {dest_dir}/.config 2>/dev/null || true""".format(
                config_flavour=config_flavour, dest_dir=dest_dir
            )
        )
        cmd.extend([conf_cmd])
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


def _get_configure_command(
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
                "${initrd_installed_kernel_modules}",
                "${initrd_configured_kernel_modules}",
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


# pylint: disable-next=too-many-arguments
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
        configured_modules = "$(cat ${initramfs_ko_modules_conf})"
    else:
        configured_modules = "${initrd_configured_kernel_modules}"

    cmd_prepare_modules_feature.extend(
        [
            'echo "Configuring ubuntu-core-initramfs.conf with supported modules"',
            'echo "If module is not included in initrd, do not include it"',
            "initramfs_conf_dir=${uc_initrd_feature_kernel_modules}/usr/lib/modules-load.d",
            "mkdir -p ${initramfs_conf_dir}",
            "initramfs_conf=${initramfs_conf_dir}/ubuntu-core-initramfs.conf",
            'echo "# configures modules" > ${initramfs_conf}',
            " ".join(
                [
                    "for",
                    "m",
                    "in",
                    configured_modules,
                ]
            ),
            "do",
            " ".join(
                [
                    "\tif [",
                    "-n",
                    f'"$(modprobe -n -q --show-depends -d {install_dir} -S "${{KERNEL_RELEASE}}" ${{m}})"',
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
                '"${SNAPD_UNPACKED_SNAP}"',
                '"usr/lib/snapd/snap-bootstrap"',
                '"${uc_initrd_feature_snap_bootstratp}"',
            ]
        ),
        " ".join(
            [
                "link_files",
                '"${SNAPD_UNPACKED_SNAP}"',
                '"usr/lib/snapd/info"',
                '"${uc_initrd_feature_snap_bootstratp}"',
            ]
        ),
        " ".join(
            [
                "cp",
                "${SNAPD_UNPACKED_SNAP}/usr/lib/snapd/info",
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
                    ' kernel-initrd-stage-firmware: true/false option"',
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
            f"{install_dir}/lib/modules/${{KERNEL_RELEASE}}",
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
    make_cmd: List[str],
    make_targets: List[str],
    make_install_targets: List[str],
    target_arch: str,
    target_arch_triplet: str,
    config_file: Optional[str],
    config_flavour: Optional[str],
    defconfig: List[str],
    configs: Optional[List[str]],
    device_trees: Optional[List[str]],
    initrd_modules: Optional[List[str]],
    configured_modules: Optional[List[str]],
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
    enable_zfs_support: bool,
    enable_perf: bool,
    project_dir: str,
    source_dir: str,
    build_dir: str,
    install_dir: str,
    stage_dir: str,
) -> List[str]:
    # kernel source can be either CRAFT_PART_SRC or CRAFT_PROJECT_DIR
    return [
        f"[ -d {source_dir}/kernel ] && KERNEL_SRC={source_dir} || KERNEL_SRC={project_dir}",
        'echo "PATH=$PATH"',
        'echo "KERNEL_SRC=${KERNEL_SRC}"',
        "",
        *_get_initrd_kernel_modules(
            initrd_modules=initrd_modules,
            configured_modules=configured_modules,
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
        *_clone_zfs_cmd(
            enable_zfs=enable_zfs_support,
            dest_dir=build_dir,
        ),
        "",
        *_clean_old_build_cmd(dest_dir=install_dir),
        "",
        *_get_configure_command(
            make_cmd=make_cmd,
            config_file=config_file,
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
            device_trees=device_trees,
            make_cmd=make_cmd.copy(),
            make_install_targets=make_install_targets,
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
            build_dir=build_dir,
            install_dir=install_dir,
            stage_dir=stage_dir,
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
        # if kernel already exists, replace it, we are probably re-running
        # build
        f"[ -e {install_dir}/kernel.img ] && rm -rf {install_dir}/kernel.img",
        (
            "ln -f ${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET} "
            f"{install_dir}/${{KERNEL_IMAGE_TARGET}}-${{KERNEL_RELEASE}}"
        ),
        f"ln -f ${{KERNEL_BUILD_ARCH_DIR}}/${{KERNEL_IMAGE_TARGET}} {install_dir}/kernel.img",
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


def _copy_dtbs_cmd(device_trees: Optional[List[str]], install_dir: str) -> List[str]:
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

        cmd.append(
            f"ln -f ${{KERNEL_BUILD_ARCH_DIR}}/dts/{dtb} {install_dir}/dtbs/{install_dtb}"
        )

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
            rm {install_dir}/modules/*/build {install_dir}/modules/*/source
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
def _get_post_install_cmd(
    device_trees: Optional[List[str]],
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
    build_dir: str,
    install_dir: str,
    stage_dir: str,
) -> List[str]:
    return [
        "",
        *_parse_kernel_release_cmd(build_dir=build_dir),
        "",
        *_copy_vmlinuz_cmd(install_dir=install_dir),
        "",
        *_copy_system_map_cmd(build_dir=build_dir, install_dir=install_dir),
        "",
        *_copy_dtbs_cmd(
            device_trees=device_trees,
            install_dir=install_dir,
        ),
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


# pylint: disable-next=too-many-arguments
def _get_install_command(
    device_trees: Optional[List[str]],
    make_cmd: List[str],
    make_install_targets: List[str],
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
    build_dir: str,
    install_dir: str,
    stage_dir: str,
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
            device_trees=device_trees,
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
            build_dir=build_dir,
            install_dir=install_dir,
            stage_dir=stage_dir,
        ),
    )

    # install .config as config-$version
    cmd.extend(_install_config_cmd(build_dir=build_dir, install_dir=install_dir))

    cmd.extend(_arrange_install_dir_cmd(install_dir=install_dir))

    return cmd


### build dependencies


def add_snappy_ppa(with_sudo=False) -> None:  # noqa: C901
    # Add ppa necessary to build initrd.
    # TODO: reimplement once snapcraft allows to the plugins
    # to add custom ppa.
    # For the moment we need to handle this as part of the
    # get_build_packages() call and add ppa manually.

    # Building of the initrd requires custom tools available in
    # ppa:snappy-dev/image.

    proc = subprocess.run(
        ["apt-get", "install", "-y", "software-properties-common"],
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
