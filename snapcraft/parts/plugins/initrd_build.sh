#!/bin/sh

parse_args() {
  # Args passed from initrd.py
  # --build-efi-image|--efi-image-{key,cert} are from snapcraft/parts/plugins/v2/initrd.py
  for arg; do
    case "${arg}" in
      initrd-modules=*)         initrd_modules="${arg#*=}"         ;; # valid: list, for mod.ko pass mod
      initrd-firmware=*)        initrd_firmware="${arg#*=}"        ;; # valid: list, relative path to "${CRAFT_STAGE}"
      initrd-addons=*)          initrd_addons="${arg#*=}"          ;; # valid: list, relative path to "${CRAFT_STAGE}"
      initrd-build-efi-image=*) initrd_build_efi_image="${arg#*=}" ;; # valid: True|False
      initrd-efi-image-key=*)   initrd_efi_image_key="${arg#*=}"   ;; # valid: path/to/key.key
      initrd-efi-image-cert=*)  initrd_efi_image_cert="${arg#*=}"  ;; # valid: path/to/cert.pem
      *) echo "err: invalid option: '${arg}'" ;;
    esac
  done
}

# Simple mount wrapper
mnt() {
  dest="$1"; shift
  mountpoint "${dest}" || mount "$@" "${dest}"
}

# Simple umount wrapper
umnt() {
  dir="$1"; shift
  mountpoint "${dir}" || umount "$@" "${dir}"
}

# clean any existing mounts for the chroot function
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

# setup necessary mounts for the chroot function
chroot_setup() {
    series="${UBUNTU_SERIES}"
    arch="${CRAFT_ARCH_BUILD_FOR}"
    ubuntu_base="ubuntu-base-${series}-${arch}.tar.gz"

    # Make sure no initrd chroot is lingering
    [ -e "${INITRD_ROOT}" ] && rm -rf "${INITRD_ROOT}"

    curl --output "${ubuntu_base}" \
      "https://cdimage.ubuntu.com/ubuntu-base/${series}/daily/current/${series}-base-${arch}.tar.gz"

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

# Do some initial configuration in chroot for initrd builds
chroot_configure() {
  series="${UBUNTU_SERIES}"

  chroot_run "apt-get update"
  chroot_run "apt-get dist-upgrade -y"
  chroot_run "apt-get install --no-install-recommends -y ca-certificates gpg dirmngr gpg-agent debconf-utils lz4 xz-utils zstd"

  chroot_run "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"

  if [ "${series}" = "focal" ] || [ "${series}" = "jammy" ]; then
    # snapd deb is required
    chroot_run "apt-get install --no-install-recommends -y snapd"
    chroot_run "apt-get install --no-install-recommends -y systemd"
  else
    chroot_run "apt-get install --no-install-recommends -y libsystemd-shared"

    if [ "${CRAFT_ARCH_BUILD_FOR}" = "amd64" ]; then
      chroot_run "apt-get install --no-install-recommends -y intel-microcode amad64-microcode"
    fi
  fi

  setup_ppa "${PPA_FINGERPRINT}"

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

# run command with chroot
# 1: chroot home
# 2: command to run, must be quoted
chroot_run() {
    cmd="$1"
    chroot "${INITRD_ROOT}" /bin/bash -c "${cmd}"
}

# Add PPA where ubuntu-core-initramfs package is published
setup_ppa() {
  fingerprint="$1"

  series="${UBUNTU_SERIES}"
  dirmngr_conf=/root/.gnupg/dirmngr.conf
  gpg_file=/etc/apt/keyrings/snappy-dev.gpg
  snappy_key=/usr/share/keyrings/snappy-dev.kbx
  source_file=/etc/apt/sources.list.d/snappy-dev-image.sources

  chroot_run "rm -rf /root/.gnupg ${gpg_file} ${snappy_key}"
  chroot_run "mkdir -p --mode 700 /root/.gnupg"
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
Suites: ${series}
Components: main
Signed-By: ${gpg_file}
EOF
}

# Add kernel modules to initrd
add_modules() {
  modules="$1"

  series="${UBUNTU_SERIES}"
  initrd_modules_dir="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/kernel-modules"
  initrd_modules_conf="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/extra-modules.conf"
  initrd_conf_dir="${initrd_modules_dir}/usr/lib/modules-load.d"
  initrd_conf="${initrd_conf_dir}/ubuntu-core-initramfs.conf"

  # A bug in releases pre-24.04; ensure modules are properly added
  if [ "${series}" = "focal" ] || [ "${series}" = "jammy" ]; then
    modules=""
    while read -r m; do
      modules="${modules} ${m}"
    done < "${initrd_modules_conf}"
  fi

  rm -f "${initrd_modules_conf}"

  if [ "${series}" = "focal" ]; then
       initrd_modules_conf="${initrd_modules_conf%/*}/main/extra-modules.conf"
  else initrd_modules_conf="${initrd_modules_conf%/*}/modules/main/extra-modules.conf"
  fi

  mkdir -p "${initrd_conf_dir}"    \
           "${initrd_modules_dir}" \
           "${initrd_modules_conf%/*}"

  echo "Adding '${modules}' to ubuntu-core-initramfs.conf"

  (
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
  )
}

# Add additional files to initrd
install_extra() {
  type="$1"
  objects="$2"

  case $type in
    addons)   extra_path="${ramdisk_overlay_path}"          ;;
    firmware) extra_path="${ramdisk_firmware_path}/usr/lib" ;;
  esac

  echo "Installing specified extra files..."
  IFS=,

  for obj in $objects; do
    find "${CRAFT_STAGE}" -name "${obj##*/}" | while read -r oobj; do
      if [ -d "{$oobj}" ]; then
        loc="${extra_path}/${obj##*/}"
        { mkdir -p "$loc" && cp -rf "${oobj}" "${loc}" ; } || {
          echo "Extra file ${oobj##*/} not found!"
        }
      else
        loc="${extra_path}/${obj}"
        { mkdir -p "${loc##*/}" && cp -rf "${oobj}" "${loc}" ; } || {
          echo "Extra file ${oobj##*/} not found!"
        [ "$type" = "firmware" ] || return 1
        }
      fi
    done
  done

  unset IFS
}
# Create the initrd
create_initrd() {
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

prep_sign() {
  key="${INITRD_ROOT}/${1}"
  cert="${INITRD_ROOT}/${2}"

  for dir in "${CRAFT_STAGE}" "${CRAFT_PROJECT_DIR}"; do
    if [ -e "${dir}/${key}" ]; then
      echo "Using '${key}' in '${dir}'"
      key="${dir}/${key}"
    fi

    if [ -e "${dir}/${cert}" ]; then
      echo "Using '${cert}' in '${dir}'"
      cert="${dir}/${cert}"
    fi
  done || echo "Using snakoil key and cert"

  cp --link "${key}"  "${INITRD_ROOT}/root/${key##*/}"
  cp --link "${cert}" "${INITRD_ROOT}/root/${cert##*/}"
}

create_efi() {
  key="$1"
  cert="$2"

  rm -f                             "${INITRD_ROOT}/boot/kernel.efi"*
  ln -f "${CRAFT_STAGE}/kernel.img" "${INITRD_ROOT}/boot/kernel.img-${KERNEL_VERSION}"

  chroot_run "ubuntu-core-initramfs create-efi    \
                --kernelver \"${KERNEL_VERSION}\"           \
                --key       \"/root/${key##*/}\"  \
                --cert      \"/root/${cert##*/}\" \
                --initrd    /boot/initrd.img      \
                --kernel    /boot/kernel.img      \
                --output    /boot/kernel.efi"

  # Remove unecessary initrd file
  rm -f  "${CRAFT_PART_INSTALL}/initrd.img-"*
  unlink "${CRAFT_PART_INSTALL}/initrd.img"

  install -Dm644 "${INITRD_ROOT}/boot/kernel.efi-${KERNEL_VERSION}" \
    "${CRAFT_PART_INSTALL}/kernel.efi-${KERNEL_VERSION}"

  ln -sf "kernel.efi-${KERNEL_VERSION}" "${CRAFT_PART_INSTALL}/kernel.efi"
}

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
  unlink "${INITRD_ROOT}/usr/lib/modules/${KERNEL_VERSION}/build"
  unlink "${INITRD_ROOT}/usr/lib/modules/${KERNEL_VERSION}/source"

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

main() {
  set -eux

  # Set initrd root
  readonly INITRD_ROOT="${CRAFT_PART_SRC}/uc-initramfs-build-root"

  # snappy-dev PPA fingerprint for ubuntu-core-initramfs deb
  readonly PPA_FINGERPRINT=F1831DDAFC42E99D

  # Get the kernel version
  KERNEL_VERSION="$(basename "${CRAFT_STAGE}/modules/"*)"; readonly KERNEL_VERSION
  readonly KERNEL_MODULES="${CRAFT_STAGE}/modules"
  readonly KERNEL_FIRMWARE="${CRAFT_STAGE}/firmware"

  # Simple tracker for chroot state
  readonly BASE_CREATED="${CRAFT_PART_SRC}/.base_created"
  readonly BASE_CONFIGURED="${CRAFT_PART_SRC}/.base_configured"

  # Paths for specific additions as specified in plugin options
  readonly ramdisk_overlay_path="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-overlay"
  readonly ramdisk_firmware_path="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-firmware"

  readonly uc_initrd_main_lib_snapd="${INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main/usr/lib/snapd"

  # clean if we fail
  trap 'clean "${INITRD_ROOT}"' EXIT INT

  parse_args "$@"
  run
}

main "$@"
