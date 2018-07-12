#!/bin/sh -e

# Remove all non-critical snaps
snaps="$(snap list | awk '{if (NR!=1) {print $1}}')"
for snap in $snaps; do
	case "$snap" in
		"core" | "snapcraft")
			# Don't want to remove these
			;;
		*)
			snap remove "$snap"
			;;
	esac
done
