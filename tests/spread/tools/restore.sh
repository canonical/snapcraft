#!/bin/sh -e

# Remove any snapcraft-configured apt keys and sources.
# Remove keyrings
rm -f /etc/apt/keyrings/craft-*
# Remove repo source files
rm -f /etc/apt/sources.list.d/craft-*
# Remove pin preferences
rm -f /etc/apt/preferences.d/craft-archives

# Remove all auto-installed packages
apt-get autoremove --purge -y

# Remove all non-critical snaps
snaps="$(snap list | awk '{if (NR!=1) {print $1}}')"
for snap in $snaps; do
	case "$snap" in
		"bare" | "core" | "core18" | "core20" | "core22" | "snapcraft" | "multipass" | "lxd" | "snapd")
			# Do not or cannot remove these
			;;
		*)
			snap remove --purge "$snap"
			;;
	esac
done
