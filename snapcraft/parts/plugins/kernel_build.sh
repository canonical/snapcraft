#!/bin/sh

# parse_args parses arguments passed to this script
parse_args() {
  for arg; do
    case ${arg} in
      kernel-kdefconfig=*)
      # kernel_kdefconfig is a list one or more kernel defconfigs
      # Default value is "defconfig".
      kernel_kdefconfig=${arg#*=}             ;;
      kernel-kconfigflavour=*)
      # kernel_kconfigflavour is a single Ubuntu-specific kernel flavour and supersedes defconfig
      # Default value is "generic".
      kernel_kconfigflavour=${arg#*=}         ;;
      kernel-kconfigs=*)
      # kernel_kconfigs is a list of of kernel kconfigs to override in the generated config
      kernel_kconfigs=${arg#*=}               ;;
      kernel-tools=*)
      # kernel_tools specifies a list of tools to build
      # Default value is "".
      kernel_tools=${arg#*=}                  ;;
      kernel-ubuntu-release-name=*)
      # kernel_ubuntu_release_name specifies the specific release to build from
      kernel_ubuntu_release_name=${arg#*=}    ;;
      kernel-ubuntu-binary-package=*)
      # kernel_ubuntu_binary_package specifies if prebuilt debs should be used
      kernel_ubuntu_binary_package=${arg#*=}  ;;
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
  # kernel-kdefconfig is passed as comma-separated
  _kernel_kdefconfig="$(echo "${kernel_kdefconfig}" | sed -e 's/,/ /g')"
  # We want _kernel_kdefconfig to split as it isn't supposed to be a single arg
  # shellcheck disable=SC2086
  make -j1                      \
       -C "${KERNEL_SRC}"       \
        O="${CRAFT_PART_BUILD}" \
        ${_kernel_kdefconfig}
}

# gen_flavour_config generates a kernel config based on the chosen flavour
gen_flavour_config() {
  OLDPWD="${PWD}"
  ubuntuconfig="${KERNEL_SRC}/CONFIGS/${CRAFT_ARCH_BUILD_FOR}-config.flavour.${kernel_kconfigflavour}"
  cd "${KERNEL_SRC}"

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
  # kernel-kconfigs is passed as comma-separated
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
         -C "${KERNEL_SRC}"       \
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
  DEBIAN="${KERNEL_SRC}/debian"
  src_pkg_name=$(sed -n '1s/^\(.*\) (.*).*$/\1/p'                                    "${DEBIAN}/changelog")
  release=$(     sed -n '1s/^'"${src_pkg_name}"'.*(\(.*\)-.*).*$/\1/p'               "${DEBIAN}/changelog")
  revision=$(    sed -n '1s/^'"${src_pkg_name}"'\ .*('"${release}"'-\(.*\)).*$/\1/p' "${DEBIAN}/changelog")
  abinum=${revision%%.*}
  uploadnum=${revision##*.}
  abi_release="${release}-${abinum}"
}

# build_tool builds a specified tool
build_tool() {
  _tool="${1}"

  case $_tool in
    bpf)      _tool=bpf/bpftool    ;;
    cpupower) _tool=power/cpupower ;;
    *)                             ;;
  esac

  echo "Building tool" "${_tool#*/}"

  mkdir -p "${CRAFT_PART_BUILD}/tools/${_tool}"

  if [ "$_tool" = "bpf/bpftool" ]; then
    # bpf is weird and won't build correctly if we aren't in the tools directory
    cd "${KERNEL_SRC}/tools/bpf"
    make -j "${CRAFT_PARALLEL_BUILD_COUNT}"      \
          O="${CRAFT_PART_BUILD}/tools/${_tool}" \
         -C bpftool all

    make DESTDIR="${CRAFT_PART_INSTALL}" \
      -C bpftool install
    cd "$OLDPWD"
    else
      make -j "${CRAFT_PARALLEL_BUILD_COUNT}" \
           -C "${KERNEL_SRC}/tools/${_tool}"  \
            O="${CRAFT_PART_BUILD}/tools/${_tool}"

    make DESTDIR="${CRAFT_PART_INSTALL}" \
      -C "${KERNEL_SRC}/tools/${_tool}" install
  fi

  install -Dm0755 "${CRAFT_PART_BUILD}/tools/${_tool}/${_tool#*/}" "${CRAFT_PART_INSTALL}/bin/${_tool#*/}"
}
# redepmod reruns depmod for the entire built kernel's module tree
redepmod() {
  _kver="${1}"
  _flavour="${2}"
  echo "Rebuilding module dependencies"
  depmod -b "${CRAFT_PART_INSTALL}" "${_kver}${_flavour}"
}

# repack_deb unpacks the primary linux-image deb package for some flavour passed as $1
# as well as the corresponding firmware, modules, and modules-extras packages and then
# repacks them
repack_deb() {
  _kver="${1}"
  _flavour="${2}"

  # For Jammy and earlier releases, linux-firmware is an ":all" package
  if [ "$UBUNTU_SERIES" = "jammy" ]; then
       apt download linux-firmware:all
  else apt download "linux-firmware:${CRAFT_ARCH_BUILD_FOR}"
  fi

  # Download the linux image binary and the corresponding modules
  apt download "linux-image-${_kver}${_flavour}:${CRAFT_ARCH_BUILD_FOR}"  \
              "linux-modules-${_kver}${_flavour}:${CRAFT_ARCH_BUILD_FOR}" \
              "linux-modules-extra-${_kver}${_flavour}:${CRAFT_ARCH_BUILD_FOR}"

  # Unpack the debs into the expected locations
  for deb in *.deb; do
    dpkg -x "${deb}" "${CRAFT_PART_INSTALL}"
  done

  mv -f "${CRAFT_PART_INSTALL}/boot/vmlinuz-${_kver}${_flavour}" \
    "${CRAFT_PART_INSTALL}/kernel.img-${_kver}${_flavour}"

  ln -sf "kernel.img-${_kver}${_flavour}" "${CRAFT_PART_INSTALL}/kernel.img"

  rm -rf "${CRAFT_PART_INSTALL:?}/usr"
}

# setup_kernel will create a kernel config if one does not exist as specified by the
# relevant part options, so long as the user hasn't sneakily added a config they'd
# prefer to use in $CRAFT_PART_BUILD. Providing a config in this fashion is not an
# explicitly supported avenue and is intended for iteration and testing.
setup_kernel() {
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
}

# build_kernel performs the build steps to create a kernel and any modules or dtbs
build_kernel() {
  echo "Building kernel..."
  # We want build_target to split as it isn't supposed to be a single arg
  # shellcheck disable=2086
  # If kconfigflavour has been chosen, we're building an Ubuntu kernel.
  # Otherwise, we're building a "regular" kernel.
  if [ -n "${kernel_kconfigflavour}" ]; then
    # Set release ABI information if a flavour is specified
    release_info
    make -j "${CRAFT_PARALLEL_BUILD_COUNT}"         \
         -C "${KERNEL_SRC}"                         \
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
         -C "${KERNEL_SRC}"                 \
          O="${CRAFT_PART_BUILD}"           \
          ${build_target}
  fi

  echo "Kernel build finished!"
}

# install_kernel installs the modules and any dtbs if required. to the proper location
install_kernel() {
  ### Install
  # Install the kernel modules, stripped
  echo "Installing kernel modules..."
  make -j "${CRAFT_PARALLEL_BUILD_COUNT}"        \
       -C "${KERNEL_SRC}"                        \
        O="${CRAFT_PART_BUILD}"                  \
        INSTALL_MOD_PATH="${CRAFT_PART_INSTALL}" \
        INSTALL_MOD_STRIP=1                      \
        modules_install

  # Install device trees, if required
  if [ "${CRAFT_ARCH_BUILD_FOR}" != "amd64" ]; then
    echo "Installing device trees..."
    make -j "${CRAFT_PARALLEL_BUILD_COUNT}"              \
         -C "${KERNEL_SRC}"                              \
          O="${CRAFT_PART_BUILD}"                        \
          INSTALL_DTBS_PATH="${CRAFT_PART_INSTALL}/dtbs" \
          dtbs_install
  fi
}

# pack_kernel moves the built artifacts into the correct place in the final snap
# hierarchy and cleans up some dangling files
pack_kernel() {
  _kver="${1}"

  # This information gets deleted by some modules, like zfs. We still need it
  # for e.g. initrd builds.
  echo "Copying some module information..."
  cp -f "${CRAFT_PART_BUILD}/modules.order"   \
        "${CRAFT_PART_BUILD}/modules.builtin" \
        "${CRAFT_PART_INSTALL}/lib/modules/${_kver}"

  # Install kernel artifacts
  echo "Copying kernel image..."
  cp -f "${CRAFT_PART_BUILD}/arch/${ARCH}/boot/${KERNEL_IMAGE}" \
        "${CRAFT_PART_INSTALL}/kernel.img-${_kver}"

  echo "Copying System map..."
  cp -f "${CRAFT_PART_BUILD}/System.map" \
        "${CRAFT_PART_INSTALL}/System.map-${_kver}"

  echo "Copying kernel config..."
  cp -f "${CRAFT_PART_BUILD}/.config" \
        "${CRAFT_PART_INSTALL}/config-${_kver}"

  ln -sf "kernel.img-${_kver}" "${CRAFT_PART_INSTALL}/kernel.img"

  # Remove symlinks lib/modules/${_kver}/{build,source}
  # It's possible that these are not even installed, however
  unlink "${CRAFT_PART_INSTALL}/lib/modules/${_kver}/build"  || true
  unlink "${CRAFT_PART_INSTALL}/lib/modules/${_kver}/source" || true
}

# create_snap_structure migrates the modules and firmware from their canonical paths to
# the one expected by snapd
create_snap_structure() {
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

# run executes the meat of this script
run() {
  # Cleanup previous builds
  if [ -e "${CRAFT_PART_INSTALL}/modules" ] ||
     [ -L "${CRAFT_PART_INSTALL}/lib/modules" ]; then
    cleanup
  fi

  # Set kconfigflavour
  if [ -n "${kernel_kconfigflavour}" ]; then
    kconfigflavour="-${kernel_kconfigflavour}"
  fi

  if [ "$kernel_ubuntu_binary_package" = "True" ]; then
    # Get the total version string and cut the upload number
    kver="$(apt info "linux-image${kconfigflavour}" | grep '^Version: ' | cut -d' ' -f2)"
    kver="${kver%.*}"
    # linux-image-${kconfigflavour} is of the form x.y.z.a-b, but ver in
    # linux-modules-<ver>-${kconfigflavour} is of the form x.y.z-a-b on Jammy
    # trim -b and swap .a for -a
    if [ "${UBUNTU_SERIES}" = "jammy" ]; then
      kver="$(echo "$kver" | sed -E 's/(.*)\./\1-/')"
    fi

    repack_deb "${kver}" "${kconfigflavour}"
  elif [ "$kernel_ubuntu_binary_package" = "False" ]; then
    # Ensure the config is setup properly
    setup_kernel

    # Perform the build
    build_kernel

    # Install kernel image, modules, dtbs
    install_kernel

    # kver depends on if release information is known or not, so
    # the version varies by if we specified a flavour or not.
    kver="$(cat "${CRAFT_PART_BUILD}/include/config/kernel.release")"

    # Cleanup final snap packaging
    pack_kernel "${kver}"

    if [ -n "${kernel_tools}" ]; then
      kernel_tools="$(echo "$kernel_tools" | sed -e 's/,/ /g')"
      for _tool in $kernel_tools; do
        build_tool "$_tool"
      done
    fi
  fi

  # Regardless of kernel source, the final structure looks the same in a snap
  create_snap_structure

  # Run depmod to ensure all modules are accounted for
  redepmod "${kver}" "${kconfigflavour}"
}

# main sets some important variables and kicks off the script
main() {
  set -ex

  # /etc/os-release is guaranteed as build host is Ubuntu
  # shellcheck disable=SC1091
  . /etc/os-release
  UBUNTU_SERIES="${UBUNTU_CODENAME}"

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

  # The "source" can come from many places; either we're:
  #   * fetching a prebuilt deb,
  #   * fetching from a specific release,
  #   * using a specified "source" in the part, or
  #   * using the project directory source
  # The first three are self-explanatory and handled by snapcraft. If a source isn't
  # provided, perhaps the user has a kernel tree in the project root. If not, fail - we
  # need something to build.
  if [ "$kernel_ubuntu_binary_package" = "True" ]; then
    # We don't actually have a true KERNEL_SRC, just set it
    KERNEL_SRC="${CRAFT_PART_INSTALL}"
  elif [ -n "$kernel_ubuntu_release_name" ]        &&
       [ "$kernel_ubuntu_release_name" != "None" ] ||
       [ -d "${CRAFT_PART_SRC}/kernel" ]; then
    # KERNEL_SRC is the true location of the kernel source tree
    KERNEL_SRC="${CRAFT_PART_SRC}"
  elif [ -d "${CRAFT_PROJECT_DIR}/kernel" ]; then
    KERNEL_SRC="${CRAFT_PROJECT_DIR}"
  else echo "Source missing from kernel part! Please specify a source" && exit 1
  fi

  run
}

main "$@"
