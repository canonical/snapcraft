#!/bin/sh

# parse_args parses arguments passed to this script
parse_args() {
  # initrd-build-efi-image|initrd-efi-image-{key,cert} are from snapcraft/parts/plugins/v2/initrd.py
  for arg; do
    case "${arg}" in
      initrd-modules=*)
      # initrd_modules is a list of modules by name to add to the initrd
      initrd_modules="${arg#*=}"              ;;
      initrd-firmware=*)
      # initrd_firmware is a list of firmware files relative to CRAFT_STAGE to add to the initrd
      initrd_firmware="${arg#*=}"             ;;
      initrd-addons=*)
      # initrd_addons is a list of files relative to CRAFT_STAGE to add to the initrd
      initrd_addons="${arg#*=}"               ;;
      initrd-build-efi-image=*)
      # initrd_build_efi_image if true builds an EFI file instead of an initrd.img
      # Default value is "False".
      initrd_build_efi_image="${arg#*=}"      ;;
      initrd-efi-image-key=*)
      # initrd_efi_image_key is a key file used to sign the EFI UKI relative to CRAFT_STAGE
      # Default value is /usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key
      initrd_efi_image_key="${arg#*=}"        ;;
      initrd-efi-image-cert=*)
      # initrd_efi_image_cert is a cert file used to sign the EFI UKI relative to CRAFT_STAGE
      # Default value is /usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem
      initrd_efi_image_cert="${arg#*=}"       ;;
      *) echo "err: invalid option: '${arg}'" ;;
    esac
  done
}

# mnt wraps the mount command
mnt() {
  dest="$1"; shift
  mountpoint "${dest}" || mount "$@" "${dest}"
}

# umnt wraps the umount command
umnt() {
  dir="$1"; shift
  mountpoint "${dir}" || umount "$@" "${dir}"
}

# clean kills processes and unmounts certain paths
clean() {
  if [ ! -d "${INITRD_ROOT}" ]; then
    echo "No chroot to clean"
    return
  fi

  # ensure no chroot processes are left running
  # Some processes are unkillable; don't fail because of it as it's why we are
  # lazy mounting in the first place
  for pid in /proc/*; do
    if [ -e "${pid}/root" ] && [ "$(readlink -f "${pid}/root")" = "${INITRD_ROOT}" ]; then
      echo "Killing PID ${pid} inside ${INITRD_ROOT} chroot"
      kill -9 "${pid}" || continue
    fi
  done

  umnt "${INITRD_ROOT}/dev/pts"
  umnt "${INITRD_ROOT}/dev/null"
  umnt "${INITRD_ROOT}/dev/zero"
  umnt "${INITRD_ROOT}/dev/full"
  umnt "${INITRD_ROOT}/dev/random"
  umnt "${INITRD_ROOT}/dev/urandom"
  umnt "${INITRD_ROOT}/dev/tty"
  umnt "${INITRD_ROOT}/dev"
  umnt "${INITRD_ROOT}/proc"
  umnt "${INITRD_ROOT}/run"
  umnt "${INITRD_ROOT}/sys"
}

# chroot_setup creates the chroot base and mounts certain filesystems from host
chroot_setup() {
    # Make sure no initrd chroot is lingering
    [ -e "${INITRD_ROOT}" ] && rm -rf "${INITRD_ROOT}"

    tar_base_url=https://cdimage.ubuntu.com/ubuntu-base
    tar_release="${UBUNTU_SERIES}/daily/current"
    tar_name="${UBUNTU_SERIES}-base-${CRAFT_ARCH_BUILD_FOR}.tar.gz"
    tar_url="${tar_base_url}/${tar_release}/${tar_name}"
    ubuntu_base="ubuntu-base-${UBUNTU_SERIES}-${CRAFT_ARCH_BUILD_FOR}.tar.gz"

    curl --output "${ubuntu_base}" "${tar_url}"

    # Extract chroot base
    mkdir -p "${INITRD_ROOT}"
    tar --extract --file "${ubuntu_base}" --directory "${INITRD_ROOT}"

    # Ensure networking in chroot
    cp --no-dereference /etc/resolv.conf "${INITRD_ROOT}/etc/resolv.conf"

    # /dev/null isn't in the chroot base but it is used to mask some systemd service units
    touch "${INITRD_ROOT}/dev/null"

    # This is a minimum viable collection of mounts.
    # Even though we try to settle any existing processes, on some systems this isn't
    # sufficient for ensuring an unmount can happen right now. Therefore, unmount lazily
    # to ensure we don't emit an error for no Good Reason and make sure the kernel
    # cleans up outstanding mounts when all PIDs and FDs are no longer relying on it.
    mnt "${INITRD_ROOT}/dev"         -o bind,lazy /dev
    mnt "${INITRD_ROOT}/dev/full"    -o bind,lazy /dev/full
    mnt "${INITRD_ROOT}/dev/null"    -o bind,lazy /dev/null
    mnt "${INITRD_ROOT}/dev/pts"     -o bind,lazy /dev/pts
    mnt "${INITRD_ROOT}/dev/random"  -o bind,lazy /dev/random
    mnt "${INITRD_ROOT}/dev/urandom" -o bind,lazy /dev/urandom
    mnt "${INITRD_ROOT}/dev/zero"    -o bind,lazy /dev/zero
    mnt "${INITRD_ROOT}/dev/tty"     -o bind,lazy /dev/tty
    # Normally we'd mount with -t but if we're in LXD, we have to mount from "host"
    mnt "${INITRD_ROOT}/proc"        -o bind,lazy /proc
    mnt "${INITRD_ROOT}/run"         -o bind,lazy /run
    mnt "${INITRD_ROOT}/sys"         -o bind,lazy /sys

    touch "${BASE_CREATED}"
}

# chroot_run runs command within chroot
# 1: path to chroot
# 2: command to run, must be quoted
chroot_run() {
    cmd="$1"
    chroot "${INITRD_ROOT}" /bin/bash -c "${cmd}"
}

# setup_ppa adds a PPA to chroot
# 1: fingerprint of PPA
setup_ppa() {
  fingerprint="$1"

  dirmngr_conf=/root/.gnupg/dirmngr.conf
  gpg_file=/etc/apt/keyrings/snappy-dev.gpg
  snappy_key=/usr/share/keyrings/snappy-dev.kbx
  source_file=/etc/apt/sources.list.d/snappy-dev-image.sources

  chroot_run "rm -rf /root/.gnupg ${gpg_file} ${snappy_key}"
  chroot_run "mkdir -p --mode 700 /root/.gnupg"
  chroot_run "mkdir -p            ${gpg_file%/*}"
  chroot_run "echo keyserver hkp://keyserver.ubuntu.com > ${dirmngr_conf}"
  chroot_run "gpg --homedir /root/.gnupg         \
                  --no-default-keyring           \
                  --keyring \"${snappy_key}\"    \
                  --recv-keys \"${fingerprint}\""

  chroot_run "gpg --homedir /root/.gnupg         \
                  --no-default-keyring           \
                  --keyring \"${snappy_key}\"    \
                  --export --out \"${gpg_file}\""

  cat > "${INITRD_ROOT}/${source_file}" << EOF
Types: deb
URIs: https://ppa.launchpadcontent.net/snappy-dev/image/ubuntu/
Suites: ${UBUNTU_SERIES}
Components: main
Signed-By: ${gpg_file}
EOF
}

# chroot_configure handles initial configuration of chroot for initrd builds
chroot_configure() {
  chroot_run "apt-get update"
  chroot_run "apt-get dist-upgrade -y"
  chroot_run "apt-get install --no-install-recommends -y ca-certificates gpg dirmngr gpg-agent debconf-utils lz4 xz-utils zstd"

  chroot_run "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"

  if [ "${UBUNTU_SERIES}" = "focal" ] || [ "${UBUNTU_SERIES}" = "jammy" ]; then
    # snapd deb is required
    chroot_run "apt-get install --no-install-recommends -y snapd"
    chroot_run "apt-get install --no-install-recommends -y systemd"
  else
    chroot_run "apt-get install --no-install-recommends -y libsystemd-shared"

    if [ "${CRAFT_ARCH_BUILD_FOR}" = "amd64" ]; then
      chroot_run "apt-get install --no-install-recommends -y intel-microcode amad64-microcode"
    fi
  fi

  # Install ubuntu-core-initramfs
  # A modified ubuntu-core-initramfs COULD be supplied by the user if they add a deb to
  # CRAFT_STAGE before the plugin is called. This is intended for debugging or
  # testing ubuntu-core-initramfs and not intended for normal consumers of this plugin.
  if [ -e "${CRAFT_STAGE}/ubuntu-core-initramfs.deb" ]; then
    cp -f "${CRAFT_STAGE}/ubuntu-core-initramfs.deb" "${INITRD_ROOT}"
    chroot_run "dpkg -i /ubuntu-core-initramfs.deb"
  else
    setup_ppa "${PPA_FINGERPRINT}"
  fi

  chroot_run "apt-get update"
  chroot_run "apt-get install --no-install-recommends -y ubuntu-core-initramfs"

  # TODO: does this belong in uci?
  # actual ubuntu-core initramfs build is performed in chroot
  # where tmp is not really tmpfs, avoid excessive use of cp
  # cp "-ar"/"-aR" -> cp "-lR"
  sed -i -e 's/"cp", "-ar", args./"cp", "-lR", args./g' \
         -e 's/"cp", "-aR", args./"cp", "-lR", args./g' \
    "${INITRD_ROOT}/usr/bin/ubuntu-core-initramfs"

  touch "${BASE_CONFIGURED}"
}

# add_modules adds modules and their dependencies to list of modules to include in initrd
add_modules() {
  modules="$1"

  initrd_modules_dir="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/kernel-modules"
  initrd_modules_conf="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/extra-modules.conf"
  initrd_conf_dir="${initrd_modules_dir}/usr/lib/modules-load.d"
  initrd_conf="${initrd_conf_dir}/ubuntu-core-initramfs.conf"

  # A bug in releases pre-24.04; ensure modules are properly added
  if [ "${UBUNTU_SERIES}" = "focal" ] || [ "${UBUNTU_SERIES}" = "jammy" ]; then
    modules=""
    while read -r m; do
      modules="${modules} ${m}"
    done < "${initrd_modules_conf}"
  fi

  rm -f "${initrd_modules_conf}"

  if [ "${UBUNTU_SERIES}" = "focal" ]; then
       initrd_modules_conf="${initrd_modules_conf%/*}/main/extra-modules.conf"
  else initrd_modules_conf="${initrd_modules_conf%/*}/modules/main/extra-modules.conf"
  fi

  mkdir -p "${initrd_conf_dir}"    \
           "${initrd_modules_dir}" \
           "${initrd_modules_conf%/*}"

  echo "Adding '${modules}' to ubuntu-core-initramfs.conf"

  IFS=,
  for m in ${modules}; do
    echo "${m}"
  done | sort -fuo "${initrd_modules_conf}"

  echo "Gathering module dependencies"
  echo "# configured modules" > "${initrd_conf}"
  for m in ${modules}; do
    if [ -n "$(modprobe -n -q --show-depends -d "${CRAFT_STAGE}" -S "${KERNEL_VERSION}" "${m}")" ]; then
      echo "${m}" >> "${initrd_conf}"
    fi
  done
  unset IFS
}

# install_extra adds files to initrd
# 1: "type", of addons|firmware|signing, determines search path
# 2: a comma-separated list of files or directories
install_extra() {
  type="$1"
  objects="$2"

  ramdisk_overlay_path="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-overlay"
  ramdisk_firmware_path="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-firmware"

  case $type in
    addons)   extra_path="${ramdisk_overlay_path}"           ;;
    firmware) extra_path="${ramdisk_firmware_path}/usr/lib/" ;;
    signing)  extra_path="${INITRD_ROOT}/root"               ;;
  esac

  echo "Installing specified extra files..."
  IFS=,

  # Iterate over objects list in CRAFT_STAGE and install them to extra_path
  for obj in $objects; do
    find "${CRAFT_STAGE}/${type}" -name "${obj##*/}" | while read -r oobj; do
      loc="${extra_path}/${obj%/*}"
      # If the location includes the name, strip it out
      # This can happen if the filename is just 'foo' instead of 'foo/bar'
      [ "${loc}" != "${extra_path}/${obj}" ] || loc="${loc%/*}"
      { mkdir -p "$loc" && cp -rf "${oobj}" "${loc}" ; } || {
        echo "Failed to copy ${oobj##*/} to ${loc}!"
        [ "$type" = "firmware" ] || exit 1
      }
    done || {
      echo "Extra file ${obj} not found!"
      # Fail if the failure was for an addon or key|cert; missing firmware is "okay"
      [ "$type" = "firmware" ] || exit 1
    }
  done

  unset IFS
}
# create_initrd uses ubuntu-core-initramfs to create an initrd.img
create_initrd() {
  uc_initrd_main_lib_snapd="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main/usr/lib/snapd"

  if [ -e "${CRAFT_PART_INSTALL}/initrd.img" ]; then
    rm -f "${CRAFT_PART_INSTALL}/initrd.img"
  fi

  # on >=core24 snapd version in initrd should be in top-level of kernel snap
  if [ "$UBUNTU_SERIES" = "noble" ]; then
      cp -f "${uc_initrd_main_lib_snapd}/info" \
        "${CRAFT_PART_INSTALL}/snapd-info"
  fi

  rm -rf "${INITRD_ROOT}/boot/initrd"*

  chroot_run "ubuntu-core-initramfs create-initrd \
                --kernelver \"${KERNEL_VERSION}\" \
                --output /boot/initrd.img"

  install -Dm644 "${INITRD_ROOT}/boot/initrd.img-${KERNEL_VERSION}" \
    "${CRAFT_PART_INSTALL}/initrd.img-${KERNEL_VERSION}"

  ln -sf "initrd.img-${KERNEL_VERSION}" "${CRAFT_PART_INSTALL}/initrd.img"
}

# prep_sign ensures the key and cert are available for signing the UKI
# 1. key file name
# 2. cert file name
prep_sign() {
  key="${CRAFT_STAGE}/signing/${1}"
  cert="${CRAFT_STAGE}/signing/${2}"

  if [ -e "${key}" ] && [ -e "${cert}" ]; then
    echo "Using ${key} and ${cert}"
    install_extra signing "${key},${cert}"
  else
    echo "Using snakeoil key and cert"
    cp -f "${INITRD_ROOT}/${initrd_efi_image_key}" \
      "${INITRD_ROOT}/root/${initrd_efi_image_key##*/}"
    cp -f "${INITRD_ROOT}/${initrd_efi_image_cert}" \
      "${INITRD_ROOT}/root/${initrd_efi_image_cert##*/}"
  fi
}

# create_efi creates an EFI UKI object from a kernel and initrd.img
create_efi() {
  key="$1"
  cert="$2"

  rm -f                             "${INITRD_ROOT}/boot/kernel.efi"*
  ln -f "${CRAFT_STAGE}/kernel.img" "${INITRD_ROOT}/boot/kernel.img-${KERNEL_VERSION}"

  chroot_run "ubuntu-core-initramfs create-efi    \
                --kernelver \"${KERNEL_VERSION}\" \
                --key       \"/root/${key##*/}\"  \
                --cert      \"/root/${cert##*/}\" \
                --initrd    /boot/initrd.img      \
                --kernel    /boot/kernel.img      \
                --output    /boot/kernel.efi"

  # Remove unnecessary initrd file
  rm -f  "${CRAFT_PART_INSTALL}/initrd.img-"*
  unlink "${CRAFT_PART_INSTALL}/initrd.img"

  install -Dm644 "${INITRD_ROOT}/boot/kernel.efi-${KERNEL_VERSION}" \
    "${CRAFT_PART_INSTALL}/kernel.efi-${KERNEL_VERSION}"

  ln -sf "kernel.efi-${KERNEL_VERSION}" "${CRAFT_PART_INSTALL}/kernel.efi"
}

# run executes the meat of this script
run() {
  # Ensure building the initrd in a native environment, so build within a chroot
  # Build within CRAFT_PART_SRC to avoid issues related to iterative builds
  printf 'Preparing to build initrd for arch %s using series %s in %s\n' \
    "${CRAFT_ARCH_BUILD_FOR}" "${UBUNTU_SERIES}" "${CRAFT_PART_SRC}"
  chroot_setup

  # Install kernel firmware, modules into chroot
  echo "Installing firmware and modules into chroot"
  rm -rf "${INITRD_ROOT}/usr/lib/firmware/"* \
         "${INITRD_ROOT}/usr/lib/modules"/*
  cp --archive --link --force "${KERNEL_FIRMWARE}" \
                              "${KERNEL_MODULES}"  \
                              "${INITRD_ROOT}/usr/lib"

  # Cleanup dangling links if they exist
  # The kernel plugin should have removed these, however
  unlink "${INITRD_ROOT}/usr/lib/modules/${KERNEL_VERSION}/build"  || true
  unlink "${INITRD_ROOT}/usr/lib/modules/${KERNEL_VERSION}/source" || true

  # Add modules to initrd
  # This should run even if none are supplied
  add_modules "${initrd_modules}"

  # Add any extra files from plugin options to initrd
  [ -z "${initrd_addons}"   ] || install_extra addons   "${initrd_addons}"
  [ -z "${initrd_firmware}" ] || install_extra firmware "${initrd_firmware}"

  # Configure chroot
  chroot_configure

  # Build the initrd image file
  create_initrd

  # Build the EFI image if requested
  [ "${initrd_build_efi_image}" = "False" ] || {
    prep_sign  "${initrd_efi_image_key}" "${initrd_efi_image_cert}"
    create_efi "${initrd_efi_image_key}" "${initrd_efi_image_cert}"
  }
}

# main sets some important variables and kicks off the script
main() {
  set -eux

  # This script is used by both legacy and current behavior. If the new
  # variables are unset fallback to old ones and use new names in the script.
  : "${CRAFT_STAGE:=$SNAPCRAFT_STAGE}"
  : "${CRAFT_PART_SRC:=$SNAPCRAFT_PART_SRC}"
  : "${CRAFT_PART_INSTALL:=$SNAPCRAFT_PART_INSTALL}"
  : "${CRAFT_ARCH_BUILD_FOR:=$SNAPCRAFT_ARCH_BUILD_FOR}"

  # Get the build environment's VERSION_CODENAME as this should match our target
  . /etc/os-release
  # UBUNTU_SERIES should match the host build environment
  UBUNTU_SERIES="${VERSION_CODENAME}"

  # INITRD_ROOT sets the chroot location
  INITRD_ROOT="${CRAFT_PART_SRC}/uc-initramfs-build-root"

  # PPA_FINGERPRINT is the snappy-dev PPA fingerprint providing ubuntu-core-initramfs deb
  PPA_FINGERPRINT=F1831DDAFC42E99D

  # KERNEL_VERSION determines what kernel file goes into the UKI
  KERNEL_VERSION="$(basename "${CRAFT_STAGE}/modules/"*)"
  # KERNEL_MODULES provides a path to the kernel file's corresponding modules
  KERNEL_MODULES="${CRAFT_STAGE}/modules"
  # KERNEL_FIRMWARE provides a path to the kernel firmware files
  KERNEL_FIRMWARE="${CRAFT_STAGE}/firmware"

  # BASE_CREATED tracks whether or not the chroot has been created
  BASE_CREATED="${CRAFT_PART_SRC}/.base_created"
  # BASE_CONFIGURED tracks whether or not the chroot has been configured
  BASE_CONFIGURED="${CRAFT_PART_SRC}/.base_configured"

  readonly INITRD_ROOT     \
           PPA_FINGERPRINT \
           KERNEL_VERSION  \
           KERNEL_MODULES  \
           KERNEL_FIRMWARE \
           BASE_CREATED    \
           BASE_CONFIGURED

  # clean if we fail
  trap 'clean "${INITRD_ROOT}"' EXIT INT

  parse_args "$@"
  run
}

main "$@"
