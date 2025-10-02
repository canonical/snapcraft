#!/bin/sh

parse_args() {
  for arg; do
    case ${arg} in
      kernel-kdefconfig=*)     kernel_kdefconfig="${arg#*=}"   ;; # valid: list
      kernel-kconfigflavour=*) kernel_kconfigflavour=${arg#*=} ;; # valid: string, trumps kdefconfig
      kernel-kconfigs=*)       kernel_kconfigs=${arg#*=}       ;; # valid: list, trumps kconfigflavour
      kernel-enable-zfs=*)     kernel_enable_zfs=${arg#*=}     ;; # valid: list, true|false
      kernel-enable-perf=*)    kernel_enable_perf=${arg#*=}    ;; # valid: list, true|false
      *) echo "err: invalid option: '${arg}'" ;;
    esac
  done
}

# Remove artifacts from prior builds
cleanup() {
  echo "Cleaning previous build first..."

  rm -rf "${CRAFT_PART_INSTALL}/modules"
  unlink "${CRAFT_PART_INSTALL}/lib/modules"
}

# Create specified defconfig
gen_defconfig() {
  make -j1                      \
       -C "${CRAFT_PART_SRC}"   \
        O="${CRAFT_PART_BUILD}" \
        "${kernel_kdefconfig}"
}

# Update config with flavour
gen_flavour_config() {
  OLDPWD="${PWD}"
  ubuntuconfig="${CRAFT_PART_SRC}/CONFIGS/${CRAFT_ARCH_BUILD_FOR}-config.flavour.${kernel_kconfigflavour}"
  cd "${CRAFT_PART_SRC}"

  # Generate the configs
  # Don't let failures stop us
  fakeroot debian/rules clean genconfigs || true

  cat "${ubuntuconfig}" > "${CRAFT_PART_BUILD}/.config"

  fakeroot debian/rules clean

  rm -rf CONFIGS

  cd "${OLDPWD}"
}

# Add any specified kconfig overrides
add_kconfigs() {
  kconfigs="${1}"
  _kconfigs="$(echo "${kconfigs}" | sed -e 's/,/\n/g')"

  echo "Applying extra config...."

  # Need to echo the list twice to force all Kconfigs to pickup the change
  echo  "${_kconfigs}"                  >  "${CRAFT_PART_BUILD}/.config_snap"
  cat   "${CRAFT_PART_BUILD}/.config"   >> "${CRAFT_PART_BUILD}/.config_snap"
  echo  "${_kconfigs}"                  >> "${CRAFT_PART_BUILD}/.config_snap"
  mv -f "${CRAFT_PART_BUILD}/.config_snap" "${CRAFT_PART_BUILD}/.config"
}

# Make sure the config is valid
remake_config() {
  bash -c 'yes "" || true' |
    make -j1                      \
         -C "${CRAFT_PART_SRC}"   \
          O="${CRAFT_PART_BUILD}" \
          oldconfig
}

# Check for Ubuntu Core and snap specific options
check_config() {
  echo "${_required_boot}"     \
       "${_required_generic}"  \
       "${_required_security}" \
       "${_required_snappy}"   \
       "${_required_systemd}"  | while read -r config; do
    if ! grep "^CONFIG_${config}=" "${CRAFT_PART_BUILD}/.config"; then
      printf '*** WARNING ***\n'
      printf 'Your kernel config is missing:\n'
      printf '%s\n' "${config}"
      printf 'Which is recommended or required by Ubuntu Core!\n'
      printf 'Continuing, but you should check your .config\n'
    fi
  done
}

# Gather kernel release info to encode in kernel artifacts
release_info() {
  echo "Gathering release information"
  DEBIAN="${CRAFT_PART_SRC}/debian"
  src_pkg_name=$(sed -n '1s/^\(.*\) (.*).*$/\1/p'                                    "${DEBIAN}/changelog")
  release=$(     sed -n '1s/^'"${src_pkg_name}"'.*(\(.*\)-.*).*$/\1/p'               "${DEBIAN}/changelog")
  revision=$(    sed -n '1s/^'"${src_pkg_name}"'\ .*('"${release}"'-\(.*\)).*$/\1/p' "${DEBIAN}/changelog")
  abinum=${revision%.*}
  uploadnum=${revision##*.}
  abi_release="${release}-${abinum}"
}

fetch_zfs() {
  echo "Cloning ZFS for ${UBUNTU_SERIES}"
  if [ ! -d "${CRAFT_PART_BUILD}/zfs" ]; then
    git clone \
      --depth 1 \
      --branch "applied/ubuntu/${UBUNTU_SERIES}" \
      https://git.launchpad.net/ubuntu/+source/zfs-linux \
      "${CRAFT_PART_BUILD}/zfs"
  fi
}

build_zfs() {
  echo "Building zfs modules..."
  cd "${CRAFT_PART_BUILD}/zfs"

  ./configure \
    --with-linux="${CRAFT_PART_SRC}"         \
    --with-linux-obj="${CRAFT_PART_BUILD}"   \
    --host="${CRAFT_ARCH_TRIPLET_BUILD_FOR}" \
    --with-config=kernel

  make -j "${CRAFT_PARALLEL_BUILD_COUNT}"
  make install DESTDIR="${CRAFT_PART_INSTALL}/zfs"

  mv -f "${CRAFT_PART_INSTALL}/zfs/lib/modules/${kver}/extra" \
        "${CRAFT_PART_INSTALL}/modules/${kver}"

  rm -rf "${CRAFT_PART_INSTALL}/zfs"
}

build_perf() {
  echo "Building perf binary..."

  mkdir -p "${CRAFT_PART_BUILD}/tools/perf"

  # Override source and build directories
  make -j "${CRAFT_PARALLEL_BUILD_COUNT}" \
       -C "${CRAFT_PART_SRC}/tools/perf"  \
        O="${CRAFT_PART_BUILD}/tools/perf"

  install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" "${CRAFT_PART_INSTALL}/bin/perf"
}

redepmod() {
  echo "Rebuilding module dependencies"
  depmod -b "${CRAFT_PART_INSTALL}" "${kver}"
}

run() {
  # Cleanup previous builds
  if [ -e "${CRAFT_PART_INSTALL}/modules" ] ||\
     [ -L "${CRAFT_PART_INSTALL}/lib/modules" ]; then
    cleanup
  fi

  ### Setup
  # Create new config if one does not exist
  [ -e "${CRAFT_PART_BUILD}/.config" ] || {
    # Privilege a specified flavour over all else
    if [ -n "${kernel_kconfigflavour}" ] && [ "${kernel_kconfigflavour}" != "generic" ]; then
        echo "Using Ubuntu config flavour ${kernel_kconfigflavour}"
        gen_flavour_config
      # Privilege specified config(s) over most
      elif [ -n "${kernel_kdefconfig}" ] && [ "${kernel_kdefconfig}" != "defconfig" ]; then
        echo "Using defconfig: ${kernel_kdefconfig}"
        gen_defconfig
    # Choose a default otherwise
    else
      echo "Using generic Ubuntu config flavour"
      gen_flavour_config
    fi

    # Add any crafter-specified configs
    if [ -n "${kernel_kconfigs}" ]; then
      add_kconfigs "${kernel_kconfigs}"
    fi

    # Rebuild config to a valid one
    echo "Remaking oldconfig...."
    remake_config
  }

  # Double check the config is valid
  echo "Checking config for expected options..."
  check_config


  ### Build
  echo "Building kernel..."
  # We want build_target to split as it isn't supposed to be a single arg
  # shellcheck disable=2086
  if [ -n "${kernel_kconfigflavour}" ]; then
    # Set release ABI information if a flavour is specified
    release_info
    make -j "${CRAFT_PARALLEL_BUILD_COUNT}"         \
         -C "${CRAFT_PART_SRC}"                     \
          O="${CRAFT_PART_BUILD}"                   \
          KERNELVERSION="${abi_release}-${kernel_kconfigflavour}" \
          KBUILD_BUILD_VERSION="${uploadnum}"       \
          CONFIG_DEBUG_SECTION_MISMATCH=y           \
          LOCALVERSION=                             \
          localver-extra=                           \
          CFLAGS_MODULE="-DPKG_ABI=${abinum}"       \
          ${build_target}
  else
    make -j "${CRAFT_PARALLEL_BUILD_COUNT}" \
         -C "${CRAFT_PART_SRC}"             \
          O="${CRAFT_PART_BUILD}"           \
          ${build_target}
  fi

  echo "Kernel build finished!"

  ### Install
  # Install the kernel modules, stripped
  echo "Installing kernel modules..."
  make -j "${CRAFT_PARALLEL_BUILD_COUNT}"        \
       -C "${CRAFT_PART_SRC}"                    \
        O="${CRAFT_PART_BUILD}"                  \
        INSTALL_MOD_PATH="${CRAFT_PART_INSTALL}" \
        INSTALL_MOD_STRIP=1                      \
        modules_install

  # Install device trees, if required
  if [ "${CRAFT_ARCH_BUILD_FOR}" != "amd64" ]; then
    echo "Installing device trees..."
    make -j "${CRAFT_PARALLEL_BUILD_COUNT}"              \
         -C "${CRAFT_PART_SRC}"                          \
          O="${CRAFT_PART_BUILD}"                        \
          INSTALL_DTBS_PATH="${CRAFT_PART_INSTALL}/dtbs" \
          dtbs_install
  fi

  # kver depends on if release information is known or not, so
  # the version varies by if we specified a flavour or not.
  kver="$(cat "${CRAFT_PART_BUILD}/include/config/kernel.release")"

  if [ "${kernel_enable_zfs}" = "True" ]; then
    fetch_zfs
    build_zfs && redepmod
  fi || echo "Not building ZFS"

  if [ "${kernel_enable_perf}" = "True" ]; then
    build_perf
  fi || echo "Not building perf"

  # Install kernel artifacts
  echo "Copying kernel image..."
  mv -f "arch/${ARCH}/boot/${KERNEL_IMAGE}" "${CRAFT_PART_INSTALL}/kernel.img"

  echo "Copying System map..."
  cp -f "${CRAFT_PART_BUILD}/System.map" "${CRAFT_PART_INSTALL}/System.map-${kver}"

  echo "Copying kernel config..."
  cp -f "${CRAFT_PART_BUILD}/.config" "${CRAFT_PART_INSTALL}/config-${kver}"

  # Remove symlinks lib/modules/$kver/{build,source}
  unlink "${CRAFT_PART_INSTALL}/lib/modules/${kver}/build"
  unlink "${CRAFT_PART_INSTALL}/lib/modules/${kver}/source"

  echo "Finalizing install directory..."
  # Usually under $INSTALL_MOD_PATH/lib/ but snapd expects modules/
  mv -f "${CRAFT_PART_INSTALL}/lib/modules" "${CRAFT_PART_INSTALL}"

  # If there is firmware dir, move it to snap root as snapd expects firmware/
  # This could have been from stage packages or from kernel build
  if [ -d "${CRAFT_PART_INSTALL}/lib/firmware" ]; then
    mv -f "${CRAFT_PART_INSTALL}/lib/firmware" "${CRAFT_PART_INSTALL}"
  fi

  # Create symlinks for canonical paths to modules, firmware
  ln -sf ../modules  "${CRAFT_PART_INSTALL}/lib/modules"
  ln -sf ../firmware "${CRAFT_PART_INSTALL}/lib/firmware"
}

main() {
  set -eux

  # Kernel configs required or strongly encouraged for Ubuntu Core and snaps
  # TODO: should these be embedded these in the plugin?
  readonly _required_generic="
  DEVTMPFS
  DEVTMPFS_MOUNT
  TMPFS_POSIX_ACL
  IPV6
  SYSVIPC
  SYSVIPC_SYSCTL
  VFAT_FS
  NLS_CODEPAGE_437
  NLS_ISO8859_1"

  readonly _required_security="
  SECURITY
  SECURITY_APPARMOR
  SYN_COOKIES
  STRICT_DEVMEM
  DEFAULT_SECURITY_APPARMOR
  SECCOMP
  SECCOMP_FILTER"

  readonly _required_snappy="
  RD_LZMA
  KEYS
  ENCRYPTED_KEYS
  SQUASHFS
  SQUASHFS_XATTR
  SQUASHFS_XZ
  SQUASHFS_LZO"

  readonly _required_systemd="
  DEVTMPFS
  CGROUPS
  INOTIFY_USER
  SIGNALFD
  TIMERFD
  EPOLL
  NET
  SYSFS
  PROC_FS
  FHANDLE
  BLK_DEV_BSG
  NET_NS
  IPV6
  AUTOFS4_FS
  TMPFS_POSIX_ACL
  TMPFS_XATTR
  SECCOMP"

  readonly _required_boot="SQUASHFS"

  # Set of valid targets specified by plugin
  readonly build_target="${KERNEL_IMAGE} ${KERNEL_TARGET}"

  parse_args "$@"
  run
}

main "$@"
