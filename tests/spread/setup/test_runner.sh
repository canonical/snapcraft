#!/bin/bash -e

run_tests()
{
	# shellcheck disable=SC1090
	. "$1"

	setup_function="$(declare -F | awk '$3 == "setup" {print $3}')"
	teardown_function="$(declare -F | awk '$3 == "teardown" {print $3}')"

	while read -r this_test; do
		echo "Running test: $this_test"

		if [ -n "$setup_function" ]; then
			$setup_function
		fi

		$this_test

		if [ -n "$teardown_function" ]; then
			$teardown_function
		fi
	done <<< "$(declare -F | awk '$3 ~ /test_/ {print $3}')"
}
