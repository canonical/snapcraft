#!/bin/bash
#
# Retry the autopkgtest run in a pull request.
# Arguments:
#   pr: The identifier of the pull request to test.
#   [release[:architecture[:test]] ...]: A list of the names of the Ubuntu releases and
#     architectures and tests to run. By default, it will launch all the tests in
#     `xenial:amd64`. If only the release name is passed as an argument, `amd64`
#     will be used as the architecture. The possible autopkgtests are `integrationtests`
#     and `snapstests`.
#
# Environment variables:
#   SNAPCRAFT_AUTOPKGTEST_SECRET: The secret to authenticate the test execution.
#
# Examples:
#   Run all the tests for pull request #123 in xenial amd64:
#     ./tools/retry_autopkgtest.sh 123
#   Run all the tests for pull request #123 in xenial armhf:
#     ./tools/retry_autopkgtest.sh 123 xenial:armhf
#   Run all the tests for pull request #123 in xenial arm64, yakkety armhf and
#   zesty amd64:
#     ./tools/retry_autopkgtest.sh 123 xenial:arm64 yakkety:armhf zesty
#   Run the integration tests for pull request #123 in xenial amd64:
#     ./tools/retry_autopkgtest.sh 123 xenial:amd64:integrationtests

if [ -z "${SNAPCRAFT_AUTOPKGTEST_SECRET}" ]; then
    echo 'Set the secret to the environment variable SNAPCRAFT_AUTOPKGTEST_SECRET.'
    exit 1
fi

if [ "$#" -lt 1 ]; then
    echo "Usage: "$0" <PR> [release[:architecture[:test]] ...]"
    exit 1
fi

pr="$1"
shift
tests=( "$@" )
[ ${#tests[@]} -eq 0 ] && tests='xenial:amd64'

temp_dir="$(mktemp -d)"
trap "rm -rf ${temp_dir}" EXIT

# Download the retry script.
wget https://git.launchpad.net/autopkgtest-cloud/plain/tools/retry-github-test -O "${temp_dir}/retry-github-test"
chmod +x "${temp_dir}/retry-github-test"

# Save the secret to a file.
echo "${SNAPCRAFT_AUTOPKGTEST_SECRET}" > "${temp_dir}/sec.txt"

for testrun in "${tests[@]}"; do
    IFS=':' read -r release architecture testsuite <<< "$testrun"
    [ -z "$architecture" ] && architecture='amd64'
    [ -n "$testsuite" ] && testname="&testname=${testsuite}"
    echo "Launching tests for the ${release} release in the ${architecture} architecture..."
    "${temp_dir}/retry-github-test" "https://api.github.com/repos/snapcore/snapcraft/pulls/${pr}" "https://autopkgtest.ubuntu.com/request.cgi?release=${release}&arch=${architecture}&package=snapcraft${testname}&ppa=snappy-dev%2Fsnapcraft-daily" "${temp_dir}/sec.txt"
done
