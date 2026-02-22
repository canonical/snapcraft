# shellcheck shell=bash
# Common function definitions
#
# Copyright 2025 林博仁(Buo-ren Lin) <buo.ren.lin@gmail.com>
# SPDX-License-Identifier: CC-BY-SA-4.0+ OR LicenseRef-Apache-2.0-If-Not-Used-In-Template-Projects

# Query the operating system distribution identifier
#
# Standard output: Result operating system distribution identifier
# Return values:
#
# * 0: OS identifier found
# * 1: Prerequisite not met
# * 2: Generic error
get_distro_identifier(){
    local operating_system_information_file=/etc/os-release

    # Out of scope
    # shellcheck source=/dev/null
    if ! source "${operating_system_information_file}"; then
        printf \
            '%s: Error: Unable to load the operating system information file.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    if ! test -v ID; then
        printf \
            '%s: Error: The ID variable assignment not found from the operating system information file(%s).\n' \
            "${FUNCNAME[0]}" \
            "${operating_system_information_file}" \
            1>&2
        return 2
    fi

    printf '%s' "${ID}"
}

# Query the operating system distribution categories
#
# Standard output: Result operating system distribution categories(or empty string if not applicable)
#
# Return values:
#
# * 0: Operation successful
# * 1: Prerequisite not met
# * 2: Generic error
get_distro_categories(){
    local operating_system_information_file=/etc/os-release

    # Out of scope
    # shellcheck source=/dev/null
    if ! source "${operating_system_information_file}"; then
        printf \
            '%s: Error: Unable to load the operating system information file.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    if ! test -v ID_LIKE; then
        # ArchLinux does not have the ID_LIKE variable in the /etc/os-release file
        ID_LIKE=
    fi

    printf '%s' "${ID_LIKE}"
}

# Check whether the specified command exists in the command search PATHs
#
# Return values:
#
# * 0: Command exists
# * 1: Command does not exist
check_command_existence(){
    local command="${1}"
    if ! command -v "${command}" >/dev/null; then
        printf \
            'Error: This program requires the "%s" command to be available in your command search PATHs.\n' \
            "${command}" \
            1>&2
        return 1
    fi

    return 0
}

# Check whether the required commands for the current distribution are available
#
# Return values:
#
# * 0: All required commands are available
# * 1: At least one required command is not available
# * 2: Generic error
check_distro_specific_required_commands(){
    local distro_id="${1}"; shift
    local distro_categories="${1}"; shift

    local -a required_commands=()
    local flag_required_command_check_failed=false

    case "${distro_categories}" in
        *debian*)
            required_commands+=(
                dpkg
                apt-get
            )
        ;;
        *rhel*)
            if ! check_command_existence dnf \
                && ! check_command_existence yum; then
                flag_required_command_check_failed=true
            fi

            required_commands+=(
                rpm
            )
        ;;
        '')
            case "${distro_id}" in
                arch)
                    required_commands+=(
                        pacman
                    )
                ;;
                *)
                    printf \
                        '%s: Error: Unsupported OS distribution: %s.\n' \
                        "${FUNCNAME[0]}" \
                        "${distro_id}" \
                        1>&2
                    return 2
                ;;
            esac
        ;;
        *)
            printf \
                '%s: Error: Unsupported OS distribution categories: %s.\n' \
                "${FUNCNAME[0]}" \
                "${distro_categories}" \
                1>&2
            return 2
        ;;
    esac

    for command in "${required_commands[@]}"; do
        if ! check_command_existence "${command}"; then
            flag_required_command_check_failed=true
        fi
    done

    if test "${flag_required_command_check_failed}" == true; then
        return 1
    fi
}

# Check whether the specified packages are installed (ArchLinux specific)
#
# Parameters:
#
# * packages...: Array of package names to check
#
# Return values:
#
# * 0: All packages are installed
# * 1: At least one package is not installed
# * 2: Generic error
check_archlinux_packages_installed(){
    local -a packages=("$@")

    if test "${#packages[@]}" -eq 0; then
        return 0
    fi

    if ! pacman -Q "${packages[@]}" &>/dev/null; then
        return 1
    fi
}

# Check whether the specified packages are installed (Debian/Ubuntu specific)
#
# Parameters:
#
# * packages...: Space-separated package names to check
#
# Return values:
#
# * 0: All packages are installed
# * 1: At least one package is not installed
# * 2: Generic error
check_debian_packages_installed(){
    local -a packages=("$@")

    if test "${#packages[@]}" -eq 0; then
        return 0
    fi

    if ! dpkg --status "${packages[@]}" &>/dev/null; then
        return 1
    fi
}

# Check whether the specified packages are installed (Red Hat-based distributions specific)
#
# Parameters:
#
# * packages...: Space-separated package names to check
#
# Return values:
#
# * 0: All packages are installed
# * 1: At least one package is not installed
# * 2: Generic error
check_redhat_packages_installed(){
    local -a packages=("$@")

    if test "${#packages[@]}" -eq 0; then
        return 0
    fi

    if ! rpm --query "${packages[@]}" &>/dev/null; then
        return 1
    fi
}

# Check whether the specified packages are installed (distribution-agnostic)
#
# Parameters:
#
# * packages...: Space-separated package names to check(or none)
#
# Return values:
#
# * 0: All packages are installed
# * 1: At least one package is not installed
# * 2: Generic error
check_distro_packages_installed(){
    local -a packages=("$@")

    if test "${#packages[@]}" -eq 0; then
        return 0
    fi

    local distro_id
    if ! distro_id="$(get_distro_identifier)"; then
        printf \
            '%s: Error: Unable to determine the OS distribution identifier.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    local distro_categories
    if ! distro_categories="$(get_distro_categories)"; then
        printf \
            '%s: Error: Unable to determine the OS distribution categories.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    case "${distro_categories}" in
        *debian*)
            check_debian_packages_installed "${packages[@]}"
        ;;
        *rhel*)
            check_redhat_packages_installed "${packages[@]}"
        ;;
        '')
            case "${distro_id}" in
                arch)
                    check_archlinux_packages_installed "${packages[@]}"
                ;;
                *)
                    printf \
                        '%s: Error: Unsupported OS distribution: %s.\n' \
                        "${FUNCNAME[0]}" \
                        "${distro_id}" \
                        1>&2
                    return 2
                ;;
            esac
        ;;
        *)
            printf \
                '%s: Error: Unsupported OS distribution categories: %s.\n' \
                "${FUNCNAME[0]}" \
                "${distro_categories}" \
                1>&2
            return 2
        ;;
    esac
}

# print progress report message with additional styling
#
# Positional parameters:
#
# progress_msg: Progress report message text
# separator_char: Character used in the separator
print_progress(){
    local progress_msg="${1}"; shift
    local separator_char
    if test "${#}" -gt 0; then
        if test "${#1}" -ne 1; then
            printf -- \
                '%s: FATAL: The separator_char positional parameter only accept a single character as its argument.\n' \
                "${FUNCNAME[0]}" \
                1>&2
            exit 99
        fi
        separator_char="${1}"; shift
    else
        separator_char=-
    fi

    local separator_string=
    local -i separator_length

    # NOTE: COLUMNS shell variable is not available in
    # non-noninteractive shell
    # FIXME: This calculation is not correct for double-width characters
    # (e.g. 中文)
    # https://www.reddit.com/r/bash/comments/gynqa0/how_to_determine_character_width_for_special/
    separator_length="${#progress_msg}"

    # Reduce costly I/O operations
    local separator_block_string=
    local -i \
        separator_block_length=10 \
        separator_blocks \
        separator_remain_units
    separator_blocks="$(( separator_length / separator_block_length ))"
    separator_remain_units="$(( separator_length % separator_block_length ))"

    local -i i j k
    for ((i = 0; i < separator_block_length; i = i + 1)); do
        separator_block_string+="${separator_char}"
    done
    for ((j = 0; j < separator_blocks; j = j + 1)); do
        separator_string+="${separator_block_string}"
    done
    for ((k = 0; k < separator_remain_units; k = k + 1)); do
        separator_string+="${separator_char}"
    done

    printf \
        '\n%s\n%s\n%s\n' \
        "${separator_string}" \
        "${progress_msg}" \
        "${separator_string}"
}

# Generate or refresh the RedHat software management system's local
# cache when necessary
refresh_redhat_local_cache(){
    if ! check_running_user; then
        printf \
            '%s: Error: The running user check has failed.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    if command -v dnf >/dev/null; then
        if ! dnf makecache; then
            printf \
                '%s: Error: Unable to refresh the DNF local cache.\n' \
                "${FUNCNAME[0]}" \
                1>&2
            return 2
        fi
    elif command -v yum >/dev/null; then
        if ! yum makecache; then
            printf \
                '%s: Error: Unable to refresh the YUM local cache.\n' \
                "${FUNCNAME[0]}" \
                1>&2
            return 2
        fi
    else
        printf \
            '%s: Error: No suitable package manager commands are found.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi
}

# Generate or refresh the Debian software management system's local cache
# when necessary
refresh_debian_local_cache(){
    if ! check_running_user; then
        printf \
            '%s: Error: The running user check has failed.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    local -a required_commands=(
        # For determining the current time
        date

        # For determining the APT local cache creation time
        stat
    )
    local required_command_check_failed=false
    for command in "${required_commands[@]}"; do
        if ! command -v "${command}" >/dev/null; then
            printf \
                '%s: Error: This function requires the "%s" command to be available in your command search PATHs.\n' \
                "${FUNCNAME[0]}" \
                "${command}" \
                1>&2
            required_command_check_failed=true
        fi
    done
    if test "${required_command_check_failed}" == true; then
        printf \
            '%s: Error: Required command check failed.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    local apt_archive_cache_mtime_epoch
    if ! apt_archive_cache_mtime_epoch="$(
        stat \
            --format=%Y \
            /var/lib/apt/lists
        )"; then
        printf \
            'Error: Unable to query the modification time of the APT software sources cache directory.\n' \
            1>&2
        return 2
    fi

    local current_time_epoch
    if ! current_time_epoch="$(
        date +%s
        )"; then
        printf \
            'Error: Unable to query the current time.\n' \
            1>&2
        return 2
    fi

    if test "$((current_time_epoch - apt_archive_cache_mtime_epoch))" -lt 86400; then
        printf \
            'Info: The last refresh time is less than 1 day, skipping...\n'
    else
        printf \
            'Info: Refreshing the APT local package cache...\n'
        if ! apt-get update; then
            printf \
                'Error: Unable to refresh the APT local package cache.\n' \
                1>&2
            return 2
        fi
    fi
}

refresh_package_manager_local_cache(){
    local distro_id="${1}"; shift
    local distro_categories="${1}"; shift

    print_progress \
        'Refreshing the package manager local cache...'

    if ! check_distro_specific_required_commands \
        "${distro_id}" \
        "${distro_categories}"; then
        printf \
            'Error: Package manager command check failed.\n' \
            1>&2
        return 1
    fi

    case "${distro_categories}" in
        *rhel*)
            if ! refresh_redhat_local_cache; then
                printf \
                    "Error: Unable to refresh the RedHat software management system's local cache.\\n" \
                    1>&2
                return 2
            fi
        ;;
        *debian*)
            if ! refresh_debian_local_cache; then
                printf \
                    "Error: Unable to refresh the Debian software management system's local cache.\\n" \
                    1>&2
                return 2
            fi
        ;;
        *)
            printf \
                '%s: Error: The OS distribution category "%s" is not supported.\n' \
                "${FUNCNAME[0]}" \
                "${distro_id}" \
                1>&2
            return 1
        ;;
    esac
}

detect_local_region_code(){
    local utility
    if command -v curl >/dev/null; then
        utility=curl
    elif command -v wget >/dev/null; then
        utility=wget
    else
        printf \
            '%s: Error: Neither "curl" nor "wget" command is available for detecting the local region code.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    local -a utility_opts=()
    case "${utility}" in
        curl)
            utility_opts=(
                # Return non-zero exit status when HTTP error occurs
                --fail

                # Do not show progress meter but keep error messages
                --silent
                --show-error

                # Avoid hanged service
                --max-time 15
            )
        ;;
        wget)
            utility_opts=(
                # Output to the standard output device
                --output-document=-

                # Don't output debug messages
                --quiet

                # Avoid hanged service
                --timeout=15
            )
        ;;
        *)
            printf \
                '%s: Error: Unsupported utility "%s" for detecting the local region code.\n' \
                "${FUNCNAME[0]}" \
                "${utility}" \
                1>&2
            return 2
        ;;
    esac
    local ip_reverse_lookup_service_response
    if ! ip_reverse_lookup_service_response="$(
            "${utility}" "${utility_opts[@]}" https://ipinfo.io/json
        )"; then
        printf \
            '%s: Warning: Unable to detect the local region code(IP address reverse lookup service not available).\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 3
    fi

    local region_code
    local -a grep_opts=(
        --perl-regexp
        --only-matching
    )
    if ! region_code="$(
        grep \
            "${grep_opts[@]}" \
            '(?<="country": ")[[:alpha:]]+' \
            <<<"${ip_reverse_lookup_service_response}"
        )"; then
        printf \
            '%s: Warning: Unable to parse out the local region code.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 4
    fi

    # The returned region code may be capitalized, normalize it.
    region_code="${region_code,,*}"

    printf '%s' "${region_code}"
    return 0
}

switch_ubuntu_local_mirror(){
    print_progress 'Switching to use the local Ubuntu software archive mirror to minimize package installation time...'

    if test -v CI; then
        printf \
            'Info: CI environment detected, will not attempt to change the software sources.\n'
        return 0
    fi

    local -a mirror_patch_dependency_pkgs=(
        # For sending HTTP request to third-party IP address lookup
        # services
        curl

        # For parsing IP address lookup response
        grep

        # For patching APT software source definition list
        sed
    )
    if ! check_distro_packages_installed "${mirror_patch_dependency_pkgs[@]}"; then
        printf \
            'Info: Installing the runtime dependencies packages for the mirror patching functionality...\n'
        if ! install_distro_packages "${mirror_patch_dependency_pkgs[@]}"; then
            printf \
                'Error: Unable to install the runtime dependencies packages for the mirror patching functionality.\n' \
                1>&2
            return 2
        fi
    fi

    printf \
        'Info: Detecting local region code...\n'
    local region_code
    if ! region_code="$(detect_local_region_code)"; then
        printf \
            'Warning: Unable to detect the local region code, falling back to the current settings.\n' \
            1>&2
        return 0
    else
        printf \
            'Info: Local region code determined to be "%s".\n' \
            "${region_code}"
    fi

    if test -n "${region_code}"; then
        printf \
            'Info: Checking whether the local Ubuntu archive mirror exists...\n'
        local -a curl_opts=(
            # Return non-zero exit status when HTTP error occurs
            --fail

            # Do not show progress meter but keep error messages
            --silent
            --show-error
        )
        if ! \
            curl \
                "${curl_opts[@]}" \
                "http://${region_code}.archive.ubuntu.com" \
                >/dev/null; then
            printf \
                "Warning: The local Ubuntu archive mirror doesn't seem to exist, falling back to current settings...\\n" \
                1>&2
            return 0
        else
            printf \
                'Info: The local Ubuntu archive mirror service seems to be available, using it.\n'
        fi
    fi

    local sources_list_file_legacy=/etc/apt/sources.list
    local sources_list_file_deb822=/etc/apt/sources.list.d/ubuntu.sources
    local sources_list_file
    if test -e "${sources_list_file_deb822}"; then
        sources_list_file="${sources_list_file_deb822}"
    else
        sources_list_file="${sources_list_file_legacy}"
    fi
    if ! grep -q "${region_code}.archive.u" "${sources_list_file}"; then
        printf \
            'Info: Switching to use the local APT software repository mirror...\n'
        if ! \
            sed \
                --regexp-extended \
                --in-place \
                "s@//([[:alpha:]]+\\.)?archive\\.ubuntu\\.com@//${region_code}.archive.ubuntu.com@g" \
                "${sources_list_file}"; then
            printf \
                'Error: Unable to switch to use the local APT software repository mirror.\n' \
                1>&2
            return 2
        fi

        printf \
            'Info: Refreshing the local APT software archive cache...\n'
        if ! apt-get update; then
            printf \
                'Error: Unable to refresh the local APT software archive cache.\n' \
                1>&2
            return 2
        fi
    fi
}

# Check whether the running user is acceptible
#
# Return values:
#
# * 0: Check success
# * 1: Prerequisite failed
# * 2: Generic error
# * 3: Check failed
check_running_user(){
    local -a required_commands=(
        # For querying the current username
        whoami
    )
    local required_command_check_failed=false
    for command in "${required_commands[@]}"; do
        if ! command -v "${command}" >/dev/null; then
            printf \
                '%s: Error: This function requires the "%s" command to be available in your command search PATHs.\n' \
                "${FUNCNAME[0]}" \
                "${command}" \
                1>&2
            required_command_check_failed=true
        fi
    done
    if test "${required_command_check_failed}" == true; then
        printf \
            '%s: Error: Required command check failed.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    printf 'Info: Checking running user...\n'
    if test "${EUID}" -ne 0; then
        printf \
            'Error: This program requires to be run as the superuser(root).\n' \
            1>&2
        return 2
    else
        local running_user
        if ! running_user="$(whoami)"; then
            printf \
                "Error: Unable to query the running user's username.\\n" \
                1>&2
            return 2
        fi
        printf \
            'Info: The running user is acceptable(%s).\n' \
            "${running_user}"
    fi
}

# Install specified distribution packages using Debian-specific
# interfaces
install_debian_packages(){
    if test "${#}" -eq 0; then
        return 0
    fi

    local -a packages=("${@}"); set --

    # Silence warnings regarding unavailable debconf frontends
    export DEBIAN_FRONTEND=noninteractive

    if ! apt-get install -y "${packages[@]}"; then
        return 2
    fi
}

# Install specified distribution packages using RedHat-specific
# interfaces
install_redhat_packages(){
    if test "${#}" -eq 0; then
        return 0
    fi

    local -a packages=("${@}"); set --

    if command -v dnf >/dev/null; then
        if ! dnf install -y "${packages[@]}"; then
            return 2
        fi
    elif command -v yum >/dev/null; then
        if ! yum install -y "${packages[@]}"; then
            return 2
        fi
    else
        printf \
            '%s: Error: Neither "dnf" nor "yum" command is available for package installation.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi
}

# Install specified distribution packages using ArchLinux-specific
# interfaces
install_archlinux_packages(){
    if test "${#}" -eq 0; then
        return 0
    fi

    local -a packages=("${@}"); set --

    if ! pacman -S --noconfirm "${packages[@]}"; then
        return 2
    fi
}

# Install specified distribution packages
#
# Return values:
#
# * 0: Operation completed successfully
# * 1: Prerequisite failed
# * 2: Generic error
# * 3: Install failed
install_distro_packages(){
    if test "${#}" -eq 0; then
        return 0
    fi

    local -a packages=("${@}"); set --

    if ! check_running_user; then
        printf \
            '%s: Error: The running user check has failed.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    local distro_id
    if ! distro_id="$(get_distro_identifier)"; then
        printf \
            '%s: Error: Unable to determine the OS distribution identifier.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    local distro_categories
    if ! distro_categories="$(get_distro_categories)"; then
        printf \
            '%s: Error: Unable to determine the OS distribution categories.\n' \
            "${FUNCNAME[0]}" \
            1>&2
        return 1
    fi

    case "${distro_categories}" in
        *debian*)
            if ! install_debian_packages "${packages[@]}"; then
                return 1
            fi
        ;;
        *rhel*)
            if ! install_redhat_packages "${packages[@]}"; then
                return 1
            fi
        ;;
        '')
            case "${distro_id}" in
                arch)
                    if ! install_archlinux_packages "${packages[@]}"; then
                        return 1
                    fi
                ;;
                debian)
                    if ! install_debian_packages "${packages[@]}"; then
                        return 1
                    fi
                ;;
                *)
                    printf \
                        '%s: Error: Unsupported OS distribution: %s.\n' \
                        "${FUNCNAME[0]}" \
                        "${distro_id}" \
                        1>&2
                    return 2
                ;;
            esac
        ;;
        *)
            printf \
                '%s: Error: Unsupported OS distribution categories: %s.\n' \
                "${FUNCNAME[0]}" \
                "${distro_categories}" \
                1>&2
            return 2
        ;;
    esac
}

workaround_git_dubious_ownership_error(){
    local project_dir="${1}"; shift

    local -a required_packages=(
        # Required for the workarounding Git's "detected dubious ownership..." error
        git
    )
    if test "${#required_packages[@]}" -gt 0; then
        if ! check_distro_packages_installed "${required_packages[@]}"; then
            if ! install_distro_packages "${required_packages[@]}"; then
                printf \
                    'Error: Unable to install the required packages for the current distribution.\n' \
                    1>&2
                return 2
            fi
        fi
    fi

    local project_git_dir="${project_dir}/.git"
    local project_git_dir_uid
    if ! project_git_dir_uid="$(stat --format '%u' "${project_git_dir}")"; then
        printf \
            'Error: Unable to query the owner user ID of the project Git directory.\n' \
            1>&2
        return 1
    fi

    if test "${project_git_dir_uid}" != "${SUDO_UID:-"${UID}"}"; then
        if test -v SUDO_UID; then
            if ! sudo -u "${SUDO_UID}" git config --global --get safe.directory &>/dev/null; then
                printf \
                    "Warning: Working around Git's \"detected dubious ownership...\" error...\\n" \
                    1>&2
                if ! sudo -u "${SUDO_UID}" git config --global --add safe.directory "${project_dir}"; then
                    printf \
                        "Error: Unable to set Git's \"safe.directory\" config.\\n" \
                        1>&2
                    return 2
                fi
            fi
        else
            if ! git config --global --get safe.directory &>/dev/null; then
                printf \
                    "Warning: Working around Git's \"detected dubious ownership...\" error...\\n" \
                    1>&2
                if ! git config --global --add safe.directory "${project_dir}"; then
                    printf \
                        "Error: Unable to set Git's \"safe.directory\" config.\\n" \
                        1>&2
                    return 2
                fi
            fi
        fi
    fi
}
