#!/bin/sh

set -eu

# TODO: should these be embedded these in the plugin?
# Required kernel configs for Ubuntu Core
_required_generic="
DEVTMPFS
DEVTMPFS_MOUNT
TMPFS_POSIX_ACL
IPV6
SYSVIPC
SYSVIPC_SYSCTL
VFAT_FS
NLS_CODEPAGE_437
NLS_ISO8859_1"

_required_security="
SECURITY
SECURITY_APPARMOR
SYN_COOKIES
STRICT_DEVMEM
DEFAULT_SECURITY_APPARMOR
SECCOMP
SECCOMP_FILTER"

_required_snappy="
RD_LZMA
KEYS
ENCRYPTED_KEYS
SQUASHFS
SQUASHFS_XATTR
SQUASHFS_XZ
SQUASHFS_LZO"

_required_systemd="
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

_required_boot="SQUASHFS"

for arg; do
  case $arg in
    defconfig=*)   defconfig=${arg#*=}   ;; # valid: list
    flavour=*)     flavour=${arg#*=}     ;; # valid: string, trumps kdefconfig
    configs=*)     configs=${arg#*=}     ;; # valid: list, trumps kconfig-flavour
    enable-zfs=*)  enable_zfs=${arg#*=}  ;; # valid: list, true|false
    enable-perf=*) enable_perf=${arg#*=} ;; # valid: list, true|false
  esac
done

# Set of valid targets specified by plugin
build_target="$KERNEL_IMAGE $KERNEL_TARGET"

# Remove artifacts from prior builds
cleanup() {
  printf 'Cleaning previous build first...\n'

  rm -rf "${CRAFT_PART_INSTALL}/modules"
  unlink "${CRAFT_PART_INSTALL}/lib/modules"
}

# Create specified defconfig
gen_defconfig() {
  make -j1                    \
       -C "$CRAFT_PART_SRC"   \
        O="$CRAFT_PART_BUILD" \
        "$defconfig"
}

# Update config with flavour
gen_flavour_config() {
  OLDPWD="$PWD"
  ubuntuconfig="${CRAFT_PART_SRC}/CONFIGS/${CRAFT_ARCH_BUILD_FOR}-config.flavour.${flavour}"
  cd "${CRAFT_PART_SRC}"

  # Generate the configs
  # Don't let failures stop us
  fakeroot debian/rules clean genconfigs || true

  cat "$ubuntuconfig" > "${CRAFT_PART_BUILD}/.config"

  fakeroot debian/rules clean

  rm -rf CONFIGS

  cd "$OLDPWD"
}

# Add any specified kconfig overrides
add_kconfigs() {
  kconfigs="$1"
  _kconfigs="$(echo "$kconfigs" | sed -e 's/,/\n/g')"

  printf 'Applying extra config....'

  # Need to echo the list twice to force all Kconfigs to pickup the change
  echo  "$_kconfigs"                    >  "${CRAFT_PART_BUILD}/.config_snap"
  cat   "${CRAFT_PART_BUILD}/.config"   >> "${CRAFT_PART_BUILD}/.config_snap"
  echo  "$_kconfigs"                    >> "${CRAFT_PART_BUILD}/.config_snap"
  mv -f "${CRAFT_PART_BUILD}/.config_snap" "${CRAFT_PART_BUILD}/.config"
}

# Make sure the config is valid
remake_config() {
  bash -c 'yes "" || true' |
    make -j1                    \
         -C "$CRAFT_PART_SRC"   \
          O="$CRAFT_PART_BUILD" \
          oldconfig
}

# Check for Ubuntu Core and snap specific options
check_config() {
  echo "$_required_boot"     \
       "$_required_generic"  \
       "$_required_security" \
       "$_required_snappy"   \
       "$_required_systemd"  | while read -r config; do
    if ! grep "^CONFIG_${config}=" "${CRAFT_PART_BUILD}/.config"; then
      printf '*** WARNING ***\n'
      printf 'Your kernel config is missing:\n'
      printf '%s\n' "$config"
      printf 'Which is recommended or required by Ubuntu Core!\n'
      printf 'Continuing, but you should check your .config\n'
    fi
  done
}

# Gather kernel release info to encode in kernel artifacts
release_info() {
  printf 'Gathering release information\n'
  DEBIAN="${CRAFT_PART_SRC}/debian"
  src_pkg_name=$(sed -n '1s/^\(.*\) (.*).*$/\1/p'                                    "${DEBIAN}/changelog")
  release=$(     sed -n '1s/^'"${src_pkg_name}"'.*(\(.*\)-.*).*$/\1/p'               "${DEBIAN}/changelog")
  revision=$(    sed -n '1s/^'"${src_pkg_name}"'\ .*('"${release}"'-\(.*\)).*$/\1/p' "${DEBIAN}/changelog")
  abinum=${revision%.*}
  uploadnum=${revision##*.}
  abi_release="${release}-${abinum}"
}

fetch_zfs() {
  printf 'Cloning ZFS for %s\n' "$UBUNTU_SERIES"

  if [ ! -d "${CRAFT_PART_BUILD}/zfs" ]; then
    git clone \
      --depth 1 \
      --branch "applied/ubuntu/$UBUNTU_SERIES" \
      https://git.launchpad.net/ubuntu/+source/zfs-linux \
      "${CRAFT_PART_BUILD}/zfs"
  fi
}

build_zfs() {
  printf 'Building zfs modules...\n'
  cd "${CRAFT_PART_BUILD}/zfs"

  ./configure \
    --with-linux="$CRAFT_PART_SRC"         \
    --with-linux-obj="$CRAFT_PART_BUILD"   \
    --host="$CRAFT_ARCH_TRIPLET_BUILD_FOR" \
    --with-config=kernel

  make -j "$CRAFT_PARALLEL_BUILD_COUNT"
  make install DESTDIR="${CRAFT_PART_INSTALL}/zfs"

  mv -f "${CRAFT_PART_INSTALL}/zfs/lib/modules/${kver}/extra" \
        "${CRAFT_PART_INSTALL}/modules/${kver}"

  rm -rf "${CRAFT_PART_INSTALL}/zfs"
}

build_perf() {
  printf 'Building perf binary...\n'

  mkdir -p "${CRAFT_PART_BUILD}/tools/perf"

  # Override source and build directories
  make -j "$CRAFT_PARALLEL_BUILD_COUNT"  \
       -C "${CRAFT_PART_SRC}/tools/perf" \
        O="${CRAFT_PART_BUILD}/tools/perf"

  install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" "${CRAFT_PART_INSTALL}/bin/perf"
}

redepmod() {
  printf 'Rebuilding module dependencies\n'
  depmod -b "$CRAFT_PART_INSTALL" "$kver"
}

# Cleanup previous builds
if [ -e "${CRAFT_PART_INSTALL}/modules" ] ||\
   [ -L "${CRAFT_PART_INSTALL}/lib/modules" ]; then
  cleanup
fi

### Setup
# Create new config if one does not exist
[ -e "${CRAFT_PART_BUILD}/.config" ] || {
  # Privilege a specified flavour over all else
  if [ -n "$flavour" ] && [ "$flavour" != "generic" ]; then
      printf "Using Ubuntu config flavour %s\n" "$flavour"
      gen_flavour_config
    # Privilege specified config(s) over most
    elif [ -n "$defconfig" ] && [ "$defconfig" != "defconfig" ]; then
      printf "Using defconfig: %s\n" "$defconfig"
      gen_defconfig
  # Choose a default otherwise
  else
    printf 'Using generic Ubuntu config flavour\n'
    gen_flavour_config
  fi

  # Add any crafter-specified configs
  if [ -n "$configs" ]; then
    add_kconfigs "$configs"
  fi

  # Rebuild config to a valid one
  printf 'Remaking oldconfig....'
  remake_config
}

# Double check the config is valid
printf 'Checking config for expected options...'
check_config


### Build
printf 'Building kernel...\n'
# We want build_target to split as it isn't supposed to be a single arg
# shellcheck disable=2086
if [ -n "$flavour" ]; then
  # Set release ABI information if a flavour is specified
  release_info
  make -j "$CRAFT_PARALLEL_BUILD_COUNT"           \
       -C "$CRAFT_PART_SRC"                       \
        O="$CRAFT_PART_BUILD"                     \
        KERNELVERSION="${abi_release}-${flavour}" \
        KBUILD_BUILD_VERSION="$uploadnum"         \
        CONFIG_DEBUG_SECTION_MISMATCH=y           \
        LOCALVERSION=                             \
        localver-extra=                           \
        CFLAGS_MODULE="-DPKG_ABI=$abinum"         \
        $build_target
else
  make -j "$CRAFT_PARALLEL_BUILD_COUNT" \
       -C "$CRAFT_PART_SRC"             \
        O="$CRAFT_PART_BUILD"           \
        $build_target
fi

printf 'Kernel build finished!\n'

### Install
# Install the kernel modules, stripped
printf 'Installing kernel modules...\n'
make -j "$CRAFT_PARALLEL_BUILD_COUNT"        \
     -C "$CRAFT_PART_SRC"                    \
      O="$CRAFT_PART_BUILD"                  \
      INSTALL_MOD_PATH="$CRAFT_PART_INSTALL" \
      INSTALL_MOD_STRIP=1                    \
      modules_install

# Install device trees, if required
if [ "$CRAFT_ARCH_BUILD_FOR" != "amd64" ]; then
  printf 'Installing device trees...\n'
  make -j "$CRAFT_PARALLEL_BUILD_COUNT"                \
       -C "$CRAFT_PART_SRC"                            \
        O="$CRAFT_PART_BUILD"                          \
        INSTALL_DTBS_PATH="${CRAFT_PART_INSTALL}/dtbs" \
        dtbs_install
fi

# kver depends on if release information is known or not, so
# the version varies by if we specified a flavour or not.
kver="$(cat "${CRAFT_PART_BUILD}/include/config/kernel.release")"

if [ "$enable_zfs" = "True" ]; then
  fetch_zfs
  build_zfs && redepmod
fi || printf 'Not building ZFS\n'

if [ "$enable_perf" = "True" ]; then
  build_perf
fi || printf 'Not building perf\n'

# Install kernel artifacts
printf 'Copying kernel image...\n'
mv -f "arch/${ARCH}/boot/${KERNEL_IMAGE}" "${CRAFT_PART_INSTALL}/kernel.img"

printf 'Copying System map...\n'
cp -f "${CRAFT_PART_BUILD}/System.map" "${CRAFT_PART_INSTALL}/System.map-${kver}"

printf 'Copying kernel config...\n'
cp -f "${CRAFT_PART_BUILD}/.config" "${CRAFT_PART_INSTALL}/config-${kver}"

# Do some final cleanup for Ubuntu Core
# Remove symlinks modules/*/build and modules/*/source
rm -rf "${CRAFT_PART_INSTALL}/modules/"*/build \
       "${CRAFT_PART_INSTALL}/modules/"*/source

printf 'Finalizing install directory...\n'
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
