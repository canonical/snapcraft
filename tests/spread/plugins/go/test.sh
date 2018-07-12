#!/bin/bash -e

ROOT="$(pwd)"

teardown()
{
	snapcraft clean
	rm -f ./*.snap
	cd "$ROOT"
}

# Basic build/run test
test_build_run()
{
	cd "$ROOT/snaps/go-hello/"
	snapcraft
	sudo snap install go-hello_*.snap --dangerous
	[ "$(go-hello)" = "hello world" ]
}

# Ensure classic go snaps can build
test_classic()
{
	cd "$ROOT/snaps/go-gotty/"

	# Don't bother if this is 14.04: classic snaps don't build there
	if [[ "$SPREAD_SYSTEM" = ubuntu-14* ]] ; then
		echo "Skipping this test: classic snaps don't build on 14.04"
		return 0
	fi

	# If the system isn't Ubuntu 16.06, we'll need to stage libc6
	if [[ "$SPREAD_SYSTEM" != ubuntu-16* ]] ; then
		cat <<- EOF >> snap/snapcraft.yaml
		    stage-packages: [libc6]
		EOF
	fi

	snapcraft prime

	# Ensure binaries are properly patched (assuming patchelf is installed)
	if which patchelf > /dev/null; then
		if [[ "$SPREAD_SYSTEM" = ubuntu-16* ]] ; then
			patchelf --print-interpreter prime/bin/gotty | MATCH "/snap/core/current"
		else
			patchelf --print-interpreter prime/bin/gotty | MATCH "/snap/gotty/current"
		fi
	fi
}

# Ensure multiple main packages are supported
test_multiple_main_packages()
{
	cd "$ROOT/snaps/go-with-multiple-main-packages/"
	snapcraft stage
	[ -f stage/bin/main1 ]
	[ -f stage/bin/main2 ]
	[ -f stage/bin/main3 ]
}

# Ensure multiple main packages are supported with go-packages as well
test_multiple_mains_with_go_packages()
{
	cd "$ROOT/snaps/go-with-multiple-main-packages/"
	cat <<- EOF >> snap/snapcraft.yaml
	      go-packages:
	          - main/main1
	          - main/main2
	          - main/main3
	EOF
	test_multiple_main_packages
}

# Test cross compiling
test_cross_compiling()
{
	cd "$ROOT/snaps/go-hello/"
	snapcraft stage --target-arch=armhf
	file stage/bin/go-hello | MATCH ",\s*ARM\s*,"
}

# Test cross compiling with cgo
test_cross_compiling_with_cgo()
{
	cd "$ROOT/snaps/go-cgo/"

	# This doesn't build on 14.04 (issues with cgo, it seems), so skip
	if [[ "$SPREAD_SYSTEM" = ubuntu-14* ]] ; then
		echo "Skipping this test: this snap doesn't build on 14.04"
		return 0
	fi

	snapcraft stage --target-arch=armhf
	file stage/bin/go-cgo | MATCH ",\s*ARM\s*,"
}
