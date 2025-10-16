#!/bin/sh

# parse_args parses arguments passed to this script
parse_args() {
  for arg; do
    case ${arg} in
      kernel-kdefconfig=*)
      # kernel_kdefconfig is a list one or more kernel defconfigs
      # Default value is "[defconfig]".
      kernel_kdefconfig="${arg#*=}"           ;;
      kernel-kconfigflavour=*)
      # kernel_kconfigflavour is a single Ubuntu-specific kernel flavour and supersedes defconfig
      # Default value is "generic".
      kernel_kconfigflavour=${arg#*=}         ;;
      kernel-kconfigs=*)
      # kernel_kconfigs is a list of of kernel kconfigs to override in the generated config
      kernel_kconfigs=${arg#*=}               ;;
      kernel-enable-zfs=*)
      # enable_zfs builds the zfs-linux package for the kernel if true
      # Default value is "False".
      kernel_enable_zfs=${arg#*=}             ;;
      kernel-enable-perf=*)
      # enable_perf builds the perf binary if true
      # Default value is "False".
      kernel_enable_perf=${arg#*=}            ;;
      *) echo "err: invalid option: '${arg}'" ;;
    esac
  done
}

# cleanup removes artifacts from prior builds
cleanup() {
  echo "Cleaning previous build first..."

  rm -rf "${CRAFT_PART_INSTALL}/modules"
  unlink "${CRAFT_PART_INSTALL}/lib/modules"
}

# gen_defconfig creates a config from one or more defconfigs
gen_defconfig() {
  make -j1                      \
       -C "${CRAFT_PART_SRC}"   \
        O="${CRAFT_PART_BUILD}" \
        "${kernel_kdefconfig}"
}

# gen_flavour_config generates a kernel config based on the chosen flavour
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

# add_kconfigs adds any specified config options
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

# remake_config ensures the generated config is valid
remake_config() {
  bash -c 'yes "" || true' |
    make -j1                      \
         -C "${CRAFT_PART_SRC}"   \
          O="${CRAFT_PART_BUILD}" \
          oldconfig
}

# check_config checks if Ubuntu Core and snap specific options are set
check_config() {
  echo "${required_boot}"     \
       "${required_generic}"  \
       "${required_security}" \
       "${required_snappy}"   \
       "${required_systemd}"  | while read -r config; do
    if ! grep "^CONFIG_${config}=" "${CRAFT_PART_BUILD}/.config"; then
      printf '*** WARNING ***\n'
      printf 'Your kernel config is missing:\n'
      printf '%s\n' "${config}"
      printf 'Which is recommended or required by Ubuntu Core!\n'
      printf 'Continuing, but you should check your .config\n'
    fi
  done
}

# release_info gathers kernel release info to encode in kernel artifacts
release_info() {
  echo "Gathering release information"
  DEBIAN="${CRAFT_PART_SRC}/debian"
  src_pkg_name=$(sed -n '1s/^\(.*\) (.*).*$/\1/p'                                    "${DEBIAN}/changelog")
  release=$(     sed -n '1s/^'"${src_pkg_name}"'.*(\(.*\)-.*).*$/\1/p'               "${DEBIAN}/changelog")
  revision=$(    sed -n '1s/^'"${src_pkg_name}"'\ .*('"${release}"'-\(.*\)).*$/\1/p' "${DEBIAN}/changelog")
  abinum=${revision%%.*}
  uploadnum=${revision##*.}
  abi_release="${release}-${abinum}"
}

# fetch_zfs downloads the zfs-linux package source for the target release
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

# build_zfs builds the zfs kernel modules for the target kernel
build_zfs() {
  echo "Building zfs modules..."
  cd "${CRAFT_PART_BUILD}/zfs"

  # In case some build files are patched, regenerate them
  ./autogen.sh

  # Importantly the paths are passed directly, so they must be fully qualified
  # paths and not relative ones
  ./configure \
    --with-linux="${CRAFT_PART_SRC}"         \
    --with-linux-obj="${CRAFT_PART_BUILD}"   \
    --host="${CRAFT_ARCH_TRIPLET_BUILD_FOR}" \
    --with-config=kernel

  # Only build the relevant module files
  make -j "${CRAFT_PARALLEL_BUILD_COUNT}" \
       -C "$PWD/module"                   \
        modules

  # Install the modules to the module tree
  # -j1 to avoid a weird error if you install "too fast"
  make -j1                              \
       -C "$PWD/module"                 \
        INSTALL_MOD_DIR=.               \
        DESTDIR="${CRAFT_PART_INSTALL}" \
        install
}

# build_perf builds the perf binary
build_perf() {
  echo "Building perf binary..."

  mkdir -p "${CRAFT_PART_BUILD}/tools/perf"

  # Override source and build directories
  make -j "${CRAFT_PARALLEL_BUILD_COUNT}" \
       -C "${CRAFT_PART_SRC}/tools/perf"  \
        O="${CRAFT_PART_BUILD}/tools/perf"

  install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" "${CRAFT_PART_INSTALL}/bin/perf"
}

# redepmod reruns depmod for the entire built kernel's module tree
redepmod() {
  echo "Rebuilding module dependencies"
  depmod -b "${CRAFT_PART_INSTALL}" "${kver}"
}

# run executes the meat of this script
run() {
  # Cleanup previous builds
  if [ -e "${CRAFT_PART_INSTALL}/modules" ] ||
     [ -L "${CRAFT_PART_INSTALL}/lib/modules" ]; then
    cleanup
  fi

  ### Setup
  # Create new config if one does not exist
  # A config COULD be supplied by the user if they add a .config to CRAFT_PART_BUILD
  # before the plugin is called. This is intended for debugging or testing and not
  # intended for normal consumers of this plugin.
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

  # Double check the config against Ubuntu Core and snapd options
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
    build_zfs
  fi || echo "Not building ZFS"

  if [ "${kernel_enable_perf}" = "True" ]; then
    build_perf
  fi || echo "Not building perf"

  # This information gets deleted by some modules, like zfs. We still need it
  # for e.g. initrd builds.
  echo "Copying some module information..."
  cp -f "${CRAFT_PART_BUILD}/modules.order"   \
        "${CRAFT_PART_BUILD}/modules.builtin" \
        "${CRAFT_PART_INSTALL}/lib/modules/${kver}"

  # Install kernel artifacts
  echo "Copying kernel image..."
  mv -f "${CRAFT_PART_BUILD}/arch/${ARCH}/boot/${KERNEL_IMAGE}" \
        "${CRAFT_PART_INSTALL}/kernel.img-${kver}"

  ln -sf "kernel.img-${kver}" "${CRAFT_PART_INSTALL}/kernel.img"

  echo "Copying System map..."
  cp -f "${CRAFT_PART_BUILD}/System.map" \
        "${CRAFT_PART_INSTALL}/System.map-${kver}"

  echo "Copying kernel config..."
  cp -f "${CRAFT_PART_BUILD}/.config" \
        "${CRAFT_PART_INSTALL}/config-${kver}"

  # Run depmod to ensure all modules are accounted for
  redepmod

  # Remove symlinks lib/modules/$kver/{build,source}
  # It's possible that these are not even installed, however
  unlink "${CRAFT_PART_INSTALL}/lib/modules/${kver}/build"  || true
  unlink "${CRAFT_PART_INSTALL}/lib/modules/${kver}/source" || true

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

# main sets some important variables and kicks off the script
main() {
  set -eux

  # This script is used by both legacy and current behavior. If the new
  # variables are unset fallback to old ones and use new names in the script.
  : "${CRAFT_PART_SRC:=$SNAPCRAFT_PART_SRC}"
  : "${CRAFT_PART_BUILD:=$SNAPCRAFT_PART_BUILD}"
  : "${CRAFT_PART_INSTALL:=$SNAPCRAFT_PART_INSTALL}"
  : "${CRAFT_ARCH_BUILD_FOR:=$SNAPCRAFT_ARCH_BUILD_FOR}"
  : "${CRAFT_PARALLEL_BUILD_COUNT:=$SNAPCRAFT_PARALLEL_BUILD_COUNT}"
  : "${CRAFT_ARCH_TRIPLET_BUILD_FOR:=$SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}"

  # Get the build environment's VERSION_CODENAME as this should match our target
  # shellcheck disable=1091
  . /etc/os-release

  # UBUNTU_SERIES should match the host build environment
  UBUNTU_SERIES="${VERSION_CODENAME}"

  # required_boot are Kconfigs required for booting Ubuntu Core
  required_boot="SQUASHFS"

  # required_generic are Kconfigs required in general
  required_generic="
  DEVTMPFS
  DEVTMPFS_MOUNT
  TMPFS_POSIX_ACL
  IPV6
  SYSVIPC
  SYSVIPC_SYSCTL
  VFAT_FS
  NLS_CODEPAGE_437
  NLS_ISO8859_1"

  # required_security are Kconfigs for sandboxing support
  required_security="
  SECURITY
  SECURITY_APPARMOR
  SYN_COOKIES
  STRICT_DEVMEM
  DEFAULT_SECURITY_APPARMOR
  SECCOMP
  SECCOMP_FILTER"

  # required_snappy are Kconfigs for snap support
  required_snappy="
  RD_LZMA
  KEYS
  ENCRYPTED_KEYS
  SQUASHFS
  SQUASHFS_XATTR
  SQUASHFS_XZ
  SQUASHFS_LZO"

  # required_systemd are Kconfigs for systemd
  required_systemd="
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

  # build_target is a list of valid kernel targets specified by plugin
  build_target="${KERNEL_IMAGE} ${KERNEL_TARGET}"

  readonly required_generic  \
           required_security \
           required_snappy   \
           required_systemd  \
           required_boot     \
           build_target

  parse_args "$@"
  run
}

main "$@"
