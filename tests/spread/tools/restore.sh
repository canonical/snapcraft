#!/bin/sh -e

# Remove all auto-installed packages
apt-get autoremove --purge -y

# Remove all non-critical snaps
snaps="$(snap list | awk '{if (NR!=1) {print $1}}')"
for snap in $snaps; do
	case "$snap" in
		"core" | "core16" | "core18" | "snapcraft" | "multipass" | "lxd" | "snapd")
			# Do not or cannot remove these
			;;
		*)
			snap remove --purge "$snap"
			;;
	esac
done
