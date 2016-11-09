#!/bin/sh
#
# Retry the autopkgtest run in a pull request.
# Arguments:
#   release:      The name of the Ubuntu release to test.
#   architecture: The architecture to test.
#   pr_id:        The identifier of the pull request to test.
# Environment variables:
#   SNAPCRAFT_AUTOPKGTEST_SECRET: The secret to authenticate the test execution.

if [ -z "${SNAPCRAFT_AUTOPKGTEST_SECRET}" ]; then
    echo 'Set the secret to the environment variable $SNAPCRAFT_AUTOPKGTEST_SECRET.'
    exit 1
fi

if [ "$#" != 3 ]; then
    echo "Usage: "$0" release architecture pr_id"
    exit 1
fi

release="$1"
architecture="$2"
pr_id="$3"

temp_dir="$(mktemp -d)"
trap "rm -rf ${temp_dir}" EXIT

# Download the retry script.
wget https://git.launchpad.net/~ubuntu-release/+git/autopkgtest-cloud/plain/tools/retry-github-test -O "${temp_dir}/retry-github-test"
chmod +x "${temp_dir}/retry-github-test"

# Save the secret to a file.
echo "${SNAPCRAFT_AUTOPKGTEST_SECRET}" > "${temp_dir}/sec.txt"

"${temp_dir}/retry-github-test" "https://api.github.com/repos/snapcore/snapcraft/pulls/${pr_id}" "https://autopkgtest.ubuntu.com/request.cgi?release=${release}&arch=${architecture}&package=snapcraft&ppa=snappy-dev%2Fsnapcraft-daily" "${temp_dir}/sec.txt"
