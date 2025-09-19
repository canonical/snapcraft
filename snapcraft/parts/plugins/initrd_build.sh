#!/bin/sh

set -eux

# clean if we fail
trap 'clean "$chroot_home"' EXIT INT

# Set initrd root
initrd_root="${CRAFT_PART_SRC}/uc-initramfs-build-root"

# snappy-dev PPA fingerprint for ubuntu-core-initramfs deb
ppa_fingerprint=F1831DDAFC42E99D

# Get the kernel version
kver="$(basename "${CRAFT_STAGE}/modules/"*)"
kernel_modules="${CRAFT_STAGE}/modules"
kernel_firmware="${CRAFT_STAGE}/firmware"

# Simple tracker for chroot state
base_created="${CRAFT_PART_SRC}/.base_created"
base_configured="${CRAFT_PART_SRC}/.base_configured"

# Args passed from initrd.py
# --build-efi-image|--efi-image-{key,cert} are from snapcraft/parts/plugins/v2/initrd.py
for arg; do
  case $arg in
    modules=*)        rd_mods=${arg#*=}  ;; # valid: list, for mod.ko pass mod
    firmware=*)       rd_fw=${arg#*=}    ;; # valid: list, relative path to "${CRAFT_STAGE}"
    addons=*)         rd_ao=${arg#*=}    ;; # valid: list, relative path to "${CRAFT_STAGE}"
    overlay=*)        rd_ol=${arg#*=}    ;; # valid: list, relative path to "${CRAFT_STAGE}"
    efi-image=*)      efi_img=${arg#*=}  ;; # valid: True|False
    efi-image-key=*)  img_key=${arg#*=}  ;; # valid: path/to/key.key
    efi-image-cert=*) img_cert=${arg#*=} ;; # valid: path/to/cert.pem
  esac
done

# Simple mount wrapper
mnt() {
  dest=$1
  shift
  mountpoint "$dest" || mount "$@" "$dest"
}

# Simple umount wrapper
umnt() {
  dir=$1
  shift
  mountpoint "$dir" || umount "$@" "$dir"
}

# clean any existing mounts for the chroot function
clean() {
  chroot_home="$1"

  if [ -z "$chroot_home" ]; then
    echo "No chroot to clean"
    return
  fi

  # ensure no chroot processes are left running
  # Some we won't be allowed to kill
  for pid in "/proc/"*; do
    if [ -e "${pid}/root" ] && [ "$(readlink -f "${pid}/root")" = "$chroot_home" ]; then
      echo "Killing PID $pid inside $chroot_home chroot"
      kill -9 "$pid" || continue
    fi
  done

  umnt "${chroot_home}/dev/pts"
  umnt "${chroot_home}/dev/null"
  umnt "${chroot_home}/dev/zero"
  umnt "${chroot_home}/dev/full"
  umnt "${chroot_home}/dev/random"
  umnt "${chroot_home}/dev/urandom"
  umnt "${chroot_home}/dev/tty"
  umnt "${chroot_home}/dev"
  umnt "${chroot_home}/proc"
  umnt "${chroot_home}/run"
  umnt "${chroot_home}/sys"

  rm -f "$base_created" \
        "$base_configured"
}

# setup necessary mounts for the chroot function
setup_chroot() {
    chroot_home="$1"
    series="$UBUNTU_SERIES"
    arch="$CRAFT_ARCH_BUILD_FOR"
    ubuntu_base="ubuntu-base-${series}-${arch}.tar.gz"


    # Make sure no initrd chroot is lingering
    [ -e "$chroot_home" ] && rm -rf "$chroot_home"

    curl --output "$ubuntu_base" \
      "https://cdimage.ubuntu.com/ubuntu-base/${series}/daily/current/${series}-base-${arch}.tar.gz"

    # Extract chroot base
    mkdir -p "$chroot_home"
    tar --extract --file "$ubuntu_base" --directory "$chroot_home"

    # Ensure networking in chroot
    cp --no-dereference /etc/resolv.conf "${chroot_home}/etc/resolv.conf"

    # /dev/null isn't in the chroot base but it is used to mask some systemd service units
    touch "${chroot_home}/dev/null"

    # This is a minimum viable collection of mounts.
    # Even though we try to settle any existing processes, on some systems this isn't
    # sufficient for ensuring an unmount can happen right now. Therefore, unmount lazily
    # to ensure we don't emit an error for no Good Reason and make sure the kernel
    # cleans up outstanding mounts when all PIDs and FDs are no longer relying on it.
    mnt "${chroot_home}/dev"         -o bind,lazy /dev
    mnt "${chroot_home}/dev/full"    -o bind,lazy /dev/full
    mnt "${chroot_home}/dev/null"    -o bind,lazy /dev/null
    mnt "${chroot_home}/dev/pts"     -o bind,lazy /dev/pts
    mnt "${chroot_home}/dev/random"  -o bind,lazy /dev/random
    mnt "${chroot_home}/dev/urandom" -o bind,lazy /dev/urandom
    mnt "${chroot_home}/dev/zero"    -o bind,lazy /dev/zero
    mnt "${chroot_home}/dev/tty"     -o bind,lazy /dev/tty
    # Normally we'd mount with -t but if we're in LXD, we have to mount from "host"
    mnt "${chroot_home}/proc"        -o bind,lazy /proc
    mnt "${chroot_home}/run"         -o bind,lazy /run
    mnt "${chroot_home}/sys"         -o bind,lazy /sys

    touch "$base_created"
}

# Do some initial configuration in chroot for initrd builds
configure_chroot() {
  chroot_home="$1"
  series="$UBUNTU_SERIES"

  run_chroot "$chroot_home" "apt-get update"
  run_chroot "$chroot_home" "apt-get dist-upgrade -y"
  run_chroot "$chroot_home" "apt-get install --no-install-recommends -y ca-certificates gpg dirmngr gpg-agent debconf-utils lz4 xz-utils zstd"

  run_chroot "$chroot_home" "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"

  if [ "$series" = "focal" ] || [ "$series" = "jammy" ]; then
    # snapd deb is required
    run_chroot "$chroot_home" "apt-get install --no-install-recommends -y snapd"
    run_chroot "$chroot_home" "apt-get install --no-install-recommends -y systemd"
  else
    run_chroot "$chroot_home" "apt-get install --no-install-recommends -y libsystemd-shared"

    if [ "$CRAFT_ARCH_BUILD_FOR" = "amd64" ]; then
      run_chroot "$chroot_home" "apt-get install --no-install-recommends -y intel-microcode amad64-microcode"
    fi
  fi

  setup_ppa "$chroot_home" "$ppa_fingerprint"

  run_chroot "$chroot_home" "apt-get update"
  run_chroot "$chroot_home" "apt-get install --no-install-recommends -y ubuntu-core-initramfs"

  # TODO: does this belong in uci?
  # actual initramfs build is performed in chroot where tmp is not really tmpfs
  # so avoid excessive use of cp to save space
  # cp "-ar"/"-aR" -> cp "-lR"
  sed -i -e 's/"cp", "-ar", args./"cp", "-lR", args./g' \
         -e 's/"cp", "-aR", args./"cp", "-lR", args./g' \
    "${chroot_home}/usr/bin/ubuntu-core-initramfs"

  touch "$base_configured"
}

# run command with chroot
# 1: chroot home
# 2: command to run, must be quoted
run_chroot() {
    chroot_home="$1"
    cmd="$2"

    chroot "$chroot_home" /bin/bash -c "$cmd"
}

# Add PPA where ubuntu-core-initramfs package is published
setup_ppa() {
  chroot_home="$1"
  fingerprint="$2"
  series="$UBUNTU_SERIES"
  dirmngr_conf=/root/.gnupg/dirmngr.conf
  gpg_file=/etc/apt/keyrings/snappy-dev.gpg
  snappy_key=/usr/share/keyrings/snappy-dev.kbx
  source_file=/etc/apt/sources.list.d/snappy-dev-image.sources

  run_chroot "$chroot_home" "rm -rf /root/.gnupg $gpg_file $snappy_key"
  run_chroot "$chroot_home" "mkdir -p --mode 700 /root/.gnupg"
  run_chroot "$chroot_home" "echo keyserver hkp://keyserver.ubuntu.com > $dirmngr_conf"
  run_chroot "$chroot_home" "gpg --homedir /root/.gnupg         \
                                   --no-default-keyring         \
                                   --keyring \"$snappy_key\"    \
                                   --recv-keys \"$fingerprint\""

  run_chroot "$chroot_home" "gpg --homedir /root/.gnupg      \
                                   --no-default-keyring      \
                                   --keyring \"$snappy_key\" \
                                   --export --out \"$gpg_file\""

  cat > "${chroot_home}/${source_file}" << EOF
Types: deb
URIs: https://ppa.launchpadcontent.net/snappy-dev/image/ubuntu/
Suites: $series
Components: main
Signed-By: $gpg_file
EOF
}

# link files helper. Accept wild cards.
# This packs arbitrary files from a host into an eventual runtime environment.
# 1: reference dir - the origin directory in the build environment
# 2: source name   - the file or directory relative to $ref_dir to be copied
# 3: dest dir      - the target location on a rootfs with respect to /
link_files() {
  ref_dir="$1"
  src="$2"
  dest_dir="$3"

  # TODO: just don't accept wildcards, instead pass "" from callers?
  # TODO: check how all this works for more complicated use-cases (addons, overlays...)
  # If src is '*', no it isn't
  [ "$src" != '*' ] || src=""

  # We're either finding all directories, some directories, or a specific file
  # The all directories case is just copying the whole tree
  # The some directories case is a subset of the all directories case
  # The specific file case results in a single result from a find
  # This resolves all arguments to their intended sets of objects
  find "${ref_dir}/${src}" -mindepth 1 -type f | while IFS= read -r f; do
    if [ -L "$f" ]; then
         # Find the actual file relative to the reference directory
         rel_path="$(realpath -e  --relative-to="$ref_dir" "$f")"
    else rel_path="$(realpath -se --relative-to="$ref_dir" "$f")"
    fi

    # The final copy location
    fin_dest="${dest_dir}/${rel_path}"
    printf 'Installing %s to %s\n' "$f" "$fin_dest"

    # Create leading directories to final location
    mkdir -p "${fin_dest%/*}"
    # Copy files from host to dest, bail if the copy fails
    cp -rf "$f" "${dest_dir}/${rel_path%/*}" || {
      printf 'Failed to copy file %s to destination %s\n' "$f" "$fin_dest"
      return 1
    }
  done
  # if [ "$src" = '*' ] || [ -d "${ref_dir}/${src}" ]; then
  #   # Preserve any spaces in name
  #   f=$(find "${ref_dir}/${src#\**}" -mindepth 1 -type f)
  #   for ff in $f; do
  #     file_list="$file_list $ff"
  #   done
  # else file_list="${ref_dir}/${src}"
  # fi

  # for f in echo "$file_list"; do
  #   if [ -L "$f" ]; then
  #        rel_path=$(realpath -e  --relative-to="$ref_dir" "$f")
  #   else rel_path=$(realpath -se --relative-to="$ref_dir" "$f")
  #   fi

  #   printf 'Installing %s to %s/%s\n' "$f" "$dest_dir" "${rel_path%/*}"

  #   # Copy files from host to dest, bail if the copy fails
  #   [ -d "${dest_dir}/${rel_path%/*}" ] || mkdir -p "${dest_dir}/${rel_path%/*}"
  #   cp -rf "$f" "${dest_dir}/${rel_path%/*}" || {
  #     printf 'Failed to copy file %s to destination %s/%s\n' "$f" "$dest_dir" "${rel_path%/*}"
  #     return 1
  #   }
  # done
  return 0
}

# Add kernel modules to initrd
add_modules() {
  chroot_home="$1"
  modules="$2"
  series="$UBUNTU_SERIES"
  initrd_modules_dir="${chroot_home}/usr/lib/ubuntu-core-initramfs/kernel-modules"
  initrd_modules_conf="${chroot_home}/usr/lib/ubuntu-core-initramfs/extra-modules.conf"
  initrd_conf_dir="${initrd_modules_dir}/usr/lib/modules-load.d"
  initrd_conf="${initrd_conf_dir}/ubuntu-core-initramfs.conf"

  # A bug in releases pre-24.04; ensure modules are properly added
  if [ "$series" = "focal" ] || [ "$series" = "jammy" ]; then
    modules=""
    while read -r m; do
      modules="$modules $m"
    done < "$initrd_modules_conf"
  fi

  rm -f "$initrd_modules_conf"

  if [ "$series" = "focal" ]; then
       initrd_modules_conf="${initrd_modules_conf%/*}/main/extra-modules.conf"
  else initrd_modules_conf="${initrd_modules_conf%/*}/modules/main/extra-modules.conf"
  fi

  mkdir -p "$initrd_conf_dir"    \
           "$initrd_modules_dir" \
           "${initrd_modules_conf%/*}"

  printf 'Adding %s to ubuntu-core-initramfs.conf\n' "$modules"
  IFS=,

  for m in $modules; do
      echo "$m"
  done | sort -fuo "$initrd_modules_conf"

  printf 'Gathering module dependencies\n'
  echo "# configured modules" > "$initrd_conf"
  for m in $modules; do
      if [ -n "$(modprobe -n -q --show-depends -d "$CRAFT_STAGE" -S "$kver" "$m")" ]; then
          echo "$m" >> "$initrd_conf"
      fi
  done

  unset IFS
}

# Add additional firmware files to initrd
install_firmware() {
  chroot_home="$1"
  rd_fw_path=${chroot_home}/usr/lib/ubuntu-core-initramfs/uc-firmware

  mkdir -p "$rd_fw_path"

  printf 'Installing specified initrd firmware files...\n'
  IFS=,
  for fw in $rd_fw; do
    # firmware can be from kernel build or from stage
    # firmware from kernel build takes preference
    link_files "$CRAFT_STAGE"        "$fw" "${rd_fw_path}/usr/lib" ||
    link_files "$CRAFT_PART_INSTALL" "$fw" "${rd_fw_path}/usr/lib" || {
        printf 'Firmware %s is missing; ignoring it\n' "$fw"
    }
  done

  unset IFS
}

# Add arbitrary file hierarchies to initrd
install_overlay() {
  chroot_home="$1"
  rd_ol_path=${chroot_home}/usr/lib/ubuntu-core-initramfs/uc-overlay

  link_files "${CRAFT_STAGE}/${rd_ol}" '*' "$rd_ol_path"
}

# Add arbitrary files to initrd
install_addons() {
  printf 'Installing specified initrd additions...\n'
  IFS=,
  for a in $rd_ao; do
    echo "Copy overlay: $a"
    link_files "$CRAFT_STAGE" "$a" "$rd_ol_path"
  done
  unset IFS
}

# Create the initrd
create_initrd() {
  chroot_home="$1"
  uc_initrd_main_lib_snapd=${chroot_home}/usr/lib/ubuntu-core-initramfs/main/usr/lib/snapd

  # Ensure chroot has been setup
  [ -e "$base_created"    ] || setup_chroot     "$chroot_home"
  [ -e "$base_configured" ] || configure_chroot "$chroot_home"

  if [ -e "${CRAFT_PART_INSTALL}/initrd.img" ]; then
    rm -f "${CRAFT_PART_INSTALL}/initrd.img"
  fi

  # on >=core24 snapd version in initrd should be in top-level of kernel snap
  if [ "$UBUNTU_SERIES" = "noble" ]; then
      cp -f "${uc_initrd_main_lib_snapd}/info" \
        "${CRAFT_PART_INSTALL}/snapd-info"
  fi

  # TODO: fix bug in uci
  printf 'Workaround an ubuntu-core-initramfs bug...\n'
  # TODO: TBD
  IFS=,
  for feature in kernel-modules uc-firmware uc-overlay; do
    link_files "${chroot_home}/usr/lib/ubuntu-core-initramfs/${feature}" \
      '*' \
      "${chroot_home}/usr/lib/ubuntu-core-initramfs/main"
  done
  unset IFS

  rm -rf "${chroot_home}/boot/initrd"*

  run_chroot "$chroot_home"              \
    "ubuntu-core-initramfs create-initrd \
      --kernelver \"$kver\"              \
      --output /boot/initrd.img"

  install -Dm644 "${chroot_home}/boot/initrd.img-${kver}" \
    "$CRAFT_PART_INSTALL/initrd.img-${kver}"

  ln -sf "initrd.img-${kver}" "${CRAFT_PART_INSTALL}/initrd.img"
}

prep_sign() {
  chroot_home="$1"
  key="$2"
  cert="$3"

  # Use snakeoil keys  by default
  key="${chroot_home}/${key}"
  cert="${chroot_home}/${cert}"

  for dir in "$CRAFT_STAGE" "$CRAFT_PROJECT_DIR"; do
    if [ -e "${dir}/${key}" ]; then
      printf 'Using %s in %s\n' "$key" "$dir"
      key="${dir}/${key}"
    fi

    if [ -e "${dir}/${cert}" ]; then
      printf 'Using %s in %s\n' "$cert" "$dir"
      cert="${dir}/${cert}"
    fi
  done || printf 'Using snakoil key and cert\n'

  # Ensure chroot has been setup
  [ -e "$base_created"    ] || setup_chroot     "$chroot_home"
  [ -e "$base_configured" ] || configure_chroot "$chroot_home"

  cp --link "$key"  "${chroot_home}/root/${key##*/}"
  cp --link "$cert" "${chroot_home}/root/${cert##*/}"
}

create_efi() {
  chroot_home="$1"
  key="$2"
  cert="$3"

  # Ensure chroot has been setup
  [ -e "$base_created"    ] || setup_chroot     "$chroot_home"
  [ -e "$base_configured" ] || configure_chroot "$chroot_home"

  rm -f                             "${chroot_home}/boot/kernel.efi"*
  ln -f "${CRAFT_STAGE}/kernel.img" "${chroot_home}/boot/kernel.img-${kver}"

  run_chroot "$chroot_home"             \
    "ubuntu-core-initramfs create-efi   \
      --kernelver \"$kver\"             \
      --key       \"/root/${key##*/}\"  \
      --cert      \"/root/${cert##*/}\" \
      --initrd    /boot/initrd.img      \
      --kernel    /boot/kernel.img      \
      --output    /boot/kernel.efi"

  # Remove unecessary initrd file
  rm -f  "${CRAFT_PART_INSTALL}/initrd.img-"*
  unlink "${CRAFT_PART_INSTALL}/initrd.img"

  install -Dm644 "${chroot_home}/boot/kernel.efi-${kver}" \
    "${CRAFT_PART_INSTALL}/kernel.efi-${kver}"

  ln -sf "kernel.efi-${kver}" "${CRAFT_PART_INSTALL}/kernel.efi"
}

# Ensure building the initrd in a native environment, so build within a chroot
# Build within CRAFT_PART_SRC to avoid issues related to iterative builds
printf 'Preparing to build initrd for arch %s using series %s in %s\n' \
  "$CRAFT_ARCH_BUILD_FOR" "$UBUNTU_SERIES" "$CRAFT_PART_SRC"
setup_chroot "$initrd_root"

# Install kernel firmware, modules into chroot
printf 'Installing firmware and modules into chroot\n'
rm -rf "${initrd_root}/usr/lib/firmware/"* \
       "${initrd_root}/usr/lib/modules"/*
cp --archive --link --force "$kernel_firmware" "${initrd_root}/usr/lib/firmware"
cp --archive --link --force "$kernel_modules"  "${initrd_root}/usr/lib/modules"

# Cleanup dangling link
rm -rf "${initrd_root}/usr/lib/modules/"*/build

# Add modules to initrd
# This should run even if none are supplied
add_modules "$initrd_root" "$rd_mods"

# TOOD: consolidate into a single function; they're pretty simple
[ -z "$rd_fw" ] || install_firmware "$initrd_root" "$rd_fw"
[ -n "$rd_ao" ] || install_addons   "$initrd_root" "$rd_ao"
[ -n "$rd_ol" ] || install_overlay  "$initrd_root" "$rd_ol"

# Configure chroot
configure_chroot "$initrd_root"

# Build the initrd image file
create_initrd "$initrd_root"

# Build the EFI image if requested
[ "$efi_img" = "False" ] || {
  prep_sign  "$initrd_root" "$img_key" "$img_cert"
  create_efi "$initrd_root" "$img_key" "$img_cert"
}
