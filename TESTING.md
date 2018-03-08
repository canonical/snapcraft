# Snapcraft tests

## Manual tests

Snapcraft has a few manual tests documented in [manual-tests.md](manual-tests.md).
These are tests for features that are not possible to automate, or that the complexity of automating them is too high while the time it takes to run them by hand is not too much.

We try very hard to automate as much as possible and to keep the manual test suite as small as possible. Please do not add a manual test before discussing it with the team, and make sure that there are very good reasons not to automate it.

When running manual tests against the production store, make sure to use the test user email account and the well-known test prefixes, since theses snaps will pollute the store. To register a user, use an email address like snapcraft-test+<your-user-name>-<unique-id>@canonical.com (e.g., snapcraft-test+elopio-123@canonical.com). This will send the confirmation email and any notifications to the inbox of snapcraft-test@canonical.com, and the snapcraft team has the password to access that inbox. To register a snap or anything else that has a name, like tracks, keys, etc., prefix the name with test-snapcraft and your name (e.g., test-snapcraft-snap-elopio-123, test-snapcraft-track-elopio-111).

### Staging server

Snapcraft has the ability to upload snaps for publication in the Snappy Store. If you're working on a feature that requires you to interact with the store, you might want to use the staging server instead of the production store. To do that, make sure you have an account on the [staging server](https://login.staging.ubuntu.com), then run:

    source tools/staging_env.sh

You will see a prompt indicating that you are going to be talking to the staging server. Once you are done working with the staging servers you can run `deactivate`.

## Automated tests in the snapcraft repository

The snapcraft repository has multiple suites of automated tests which run the code at different levels, following different principles of what to execute, how to force different code paths, and how to verify the results.

### Static tests

The static tests suite performs a static analysis on the source code without executing it. It will catch syntax and code style errors.

### Unit tests

The unit tests is a suite of low-level white box tests. They exercise units of code to verify that the different parts work as expected in a fully isolated way. Ideally, these tests should call only public functions, objects and methods, leaving the internals of snapcraft as implementation details that can change without having to modifying any tests. To isolate the units from their environment and dependencies we can replace those with test doubles. In order to set up test doubles, we prefer dependency injection through arguments than excessive mocking. These tests can verify the results checking the output printed to the command line, checking the files created during the execution, inspecting the calls made to the test doubles and verifying that the expected exceptions were thrown.

These tests are in the `tests/unit` directory.

### Integration tests

The integration tests are a group of suites that exercise snapcraft as a black box. They are only allowed to set up the environment where snapcraft runs and create files; but for the execution phase of the test they can only run the snapcraft command or one of its subcommands. To verify the results they can check the output printed to the command line, the return value of the snapcraft command, and any files created during the execution.

These tests are in the `tests/integration` directory, with the `snapcraft.yamls` and other source files for the tests snaps in `tests/integration/snaps`.

### Slow tests

Some tests take too long. This affects the pull requests because we have to wait for a long time, and they will make Travis CI timeout because we have only 50 minutes per suite in there. The solution is to tag these tests as slow, and don't run them in all pull requests. These tests will only be run in autopkgtests.

To mark a test case as slow, set the class attribute `slow_test = True`.

To run all the tests, including the slow ones, set the environment variable `SNAPCRAFT_SLOW_TESTS=1`.

### Snaps tests

The snaps tests is a suite of high-level tests that try to simulate real-world scenarios of a user interacting with snapcraft. They cover the call to snapcraft to generate a snap file from the source files of a fully functional project, the installation of the resulting snap, and the execution of the binaries and services of this snap.

These tests are in the `snaps_tests` directory, with the sources for the test snaps in the `demos` directory.

### Setting up the environment

In order to run these tests suites, first you will need to set up your development environment. Follow the steps in the [Hacking guide](HACKING.md) to install for development.

Then, you'll need a few more dependencies:

    sudo apt install squashfs-tools xdelta3 bzr git mercurial subversion

### Running the tests

To run the static tests, execute:

    ./runtests.sh static

To run the unit tests, execute:

    ./runtests.sh tests/unit

To run the integration tests, execute:

    ./runtests.sh tests/integration

You can also run a subsuite of the unit or integration suites specifying the path to the directory.
For example:

  * To run only the unit tests for the plugins:

    ```
    ./runtests.sh tests/unit/plugins
    ```

  * To run only the integration tests for the store:

    ```
    ./runtests.sh tests/integration/store
    ```

And you can also run a single test module, test case or test function using the usual python way.
For example:

  * To run only the unit tests in the test_nodejs module:

    ```
    python3 -m unittest tests.unit.plugins.tests_nodejs
    ```

  * To run only the unit tests in the NodePluginTestCase:

    ```
    python3 -m unittest tests.unit.plugins.tests_nodejs.NodePluginTestCase
    ```

  * To run only the unit test named test_pull_executes_npm_run_commands:

    ```
    python3 -m unittest tests.unit.plugins.tests_nodejs.NodePluginTestCase.test_pull_executes_npm_run_commands
    ```

The snaps tests script has more complex arguments. For an explanation of them, run:

    python3 -m snaps_tests -h

The integration and snaps suites can be run using the snapcraft source from the repository, or using the snapacraft command installed in the system. By default, they will use the source code, so you can modify your clone of the repository and verify that your changes are correct. If instead you want to verify that the snapcraft version installed in your system is correct, run them with the environment variable `SNAPCRAFT_FROM_DEB` or `SNAPCRAFT_FROM_SNAP` set, like this:

    SNAPCRAFT_FROM_DEB=1 ./runtests.sh tests/integration

or

    SNAPCRAFT_FROM_SNAP=1 ./runtests.sh tests/integration

## Setting up the store test user

The store tests by default will start fake servers that are configured to reply like the real store does. But you can run them also against the staging and production store servers. To do that, you will need to set the `TEST_STORE` environment variable to either `staging` or `production`, and you also have to pass credentials for a valid user in that store with the environment variable `TEST_USER_EMAIL` and `TEST_USER_PASSWORD`, like this:

    TEST_STORE=staging TEST_USER_EMAIL=test@example.com TEST_USER_PASSWORD=Hola123* ./runtests.sh tests/integration/store

To prepare a user for testing, go to https://login.staging.ubuntu.com/ (or
https://login.ubuntu.com/ for the production store) and create a new user. Then
go to https://dashboard.staging.snapcraft.io/ (or
https://dashboard.staging.snapcraft.io/ for the production store) to sign the
developer agreement.

Note that most testing should be done on the staging server. If multiple tests
have to be executed on production, notify the store team before.

## Autopkgtests for the snapcraft deb

Autopkgtests are tests for the project packaged as a deb. The unit tests are run during autopkgtests while the snapcraft deb is being built. Then the resulting deb is installed, and the integration and snaps suites are executed using the installed snapcraft.

To run them locally, the easiest way is to use a LXC container. Note however that snaps in LXC need squashfuse installed, which we need to do via the `--setup-commands` option. From the root of the project, run:

    sudo apt install autopkgtest
    adt-run --unbuilt-tree . --apt-upgrade --setup-commands="apt install squashfuse -y" --- lxd ubuntu:xenial

It's possible to select only one of the suites, with:

    adt-run --unbuilt-tree . --apt-upgrade --setup-commands="apt install squashfuse -y" --testname=integrationtests --- lxd ubuntu:xenial

or:

    adt-run --unbuilt-tree . --apt-upgrade --setup-commands="apt install squashfuse -y" --testname=snapstests --- lxd ubuntu:xenial

## Spread tests for the snapcraft snap

[Spread](https://github.com/snapcore/spread) is a system to distribute tests and execute them in different backends, in parallel. We are currently using spread only to run the integration suite using the installed snapcraft snap from the edge channel.

To run them, first, download the spread binary:

    curl -s -O https://niemeyer.s3.amazonaws.com/spread-amd64.tar.gz && tar xzvf spread-amd64.tar.gz

Then, you can run them using a local LXD as the backend with:

    ./spread -v lxd:

Or, you can run them in linode if you have a `SPREAD_LINODE_KEY`, with:

    SPREAD_LINODE_KEY={key} ./spread -v linode:

## External snaps tests

The idea of the external snaps tests is to clone a repository external to snapcraft that contains a `snapcraft.yaml`, and check that snapcraft can build successfully that snap. There is a script in the snapcraft repo to help with this. You can see how to use it running:

    python3 -m external_snaps_tests --help

We have a suite of external snaps tests that runs each night using the latest snapcraft master to build a big variety of snaps. It is located in https://github.com/elopio/snapcraft-de-noche and you can add new snaps to the suite just by adding them to the `.travis.yml` file.

## Reproducible builds tests

This is an experimental suite, with still some details to define. The idea is to build a snap recording a manifest of all the details of the build. Then build the snap again, but this time using the manifest instead of the source `snapcraft.yaml`, and compare that both snaps are equal.

Currently, the suite is using the snaps of the integration suite to check the reproducibility. It is located in https://github.com/elopio/snapcraft-reproducible/
