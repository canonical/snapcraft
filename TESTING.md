# Snapcraft tests

## Manual tests

Snapcraft has a few manual tests documented in [manual-tests.md](manual-tests.md).
These are tests for features that are not possible to automate, or that the complexity of automating them is too high while the time it takes to run them by hand is not too much.

We try very hard to automate as much as possible and to keep the manual test suite as small as possible. Please do not add a manual test before discussing it with the team, and make sure that there are very good reasons not to automate it.

### Staging server

Snapcraft has the ability to upload snaps for publication in the Snappy Store. If you're working on a feature that requires you to interact with the store, you might want to use the staging server instead of the production store. To do that, make sure you have an account on the [staging server](https://login.staging.ubuntu.com), then run:

    source tools/staging_env.sh

You will see a prompt indicating that you are going to be talking to the staging server. Once you are done working with the staging servers you can run `deactivate`.

## Automated tests in the snapcraft repository

The snapcraft repository has multiple suites of automated tests which run the code at different levels, following different principles of what to execute, how to force different code paths, and how to verify the results.

### Static tests

The static tests suite performs a static analysis on the source code without executing it. It will catch syntax and code style errors.

### Unit tests

The unit tests is a suite of low-level white box tests. They excercise units of code to verify that the different parts work as expected in a fully isolated way. Ideally, these tests should call only public functions, objects and methods, leaving the internals of snapcraft as implementation details that can change without having to modifying any tests. To isolate the units from their environment and dependencies we can replace those with test doubles. In order to set up test doubles, we prefer dependency injection through arguments than excessive mocking. These tests can verify the results checking the output printed to the command line, checking the files created during the excecution, inspecting the calls made to the test doubles and verifying that the expected exceptions were thrown.

These tests are in the `snapcraft/tests` directory.

### Integration tests

The integration tests are a group of suites that excercise snapcraft as a black box. They are only allowed to set up the environment where snapcraft runs and create files; but for the execution phase of the test they can only run the snapcraft command or one of its subcommands. To verify the results they can check the output printed to the command line, the return value of the snapcraft command, and any files created during the execution.

This suite was split in three: plugins, store and other integration tests. This split is artificial, we made it just because the full suite takes more time than what Travis allows for a single job.

These tests are in the `integration_tests` directory, with the `snapcraft.yamls` and other source files for the tests snaps in `integration_tests/snaps`.

### Snaps tests

The snaps tests is a suite of high-level tests that try to simulate real-world scenarios of a user interacting with snapcraft. They cover the call to snapcraft to generate a snap file from the source files of a fully functional project, the installation of the resulting snap, and the execution of the binaries and services of this snap.

These tests are in the `snaps_tests` directory, with the sources for the test snaps in the `demos` directory.

### Setting up the environment

In order to run these tests suites, first you will need to set up your development environment. Follow the steps in the [Hacking guide](HACKING.md) to install for development.

Then, you'll need a few more dependencies:

    sudo apt install squashfs-tools xdelta3 bzr git mercurial subversion

### Running the tests

To run all the tests execute:

    ./runtests.sh

You can selectively run only one of the suites, and apply a filter:

    ./runtests.sh [static|unit|integration|plugins|store|snaps] [pattern]

Examples:

  * To run only the static tests:

    ```
    ./runtests.sh static
    ```

  * To run only the full unit test suite:

    ```
    ./runtests.sh unit
    ```

  * To run only the unit tests in the file called `test_init.py`:

    ```
    ./runtests.sh unit test_init.py
    ```

  * To run only the integration tests that interact with the store:

    ```
    ./runtests.sh store
    ```

  * To run only the integration tests for the python plugin:

    ```
    ./runtests.sh plugins *python*
    ```

  * To run the integration tests that are not related to plugins or the store:

    ```
    ./runtests.sh integration
    ```

The snaps tests script has more complex arguments. For an explanation of them, run:

    python3 -m snaps_tests -h

The integration and snaps suites can be run using the snapcraft source from the repository, or using the snapacraft command installed in the system. By default, they will use the source code, so you can modify your clone of the repository and verify that your changes are correct. If instead you want to verify that the snapcraft version installed in your system is correct, run them with the environment variable `SNAPCRAFT_FROM_INSTALLED` set, like this:

    SNAPCRAFT_FROM_INSTALLED=1 ./runtests.sh [integration|plugins|store|snaps] [pattern]

The store tests by default will start fake servers that are configured to reply like the real store does. But you can run them also against the staging and production store servers. To do that, you will need to set the `TEST_STORE` environment variable to either `staging` or `production`, and you also have to pass credentials for a valid user in that store with the environment variable `TEST_USER_EMAIL` and `TEST_USER_PASSWORD`, like this:

    TEST_STORE=staging TEST_USER_EMAIL=test@example.com TEST_USER_PASSWORD=Hola123* ./runtests.sh store [pattern]

## Autopkgtests for the snapcraft deb

Autopkgtests are tests for the project packaged as a deb. The unit tests are run during autopkgtests while the snapcraft deb is being built. Then the resulting deb is installed, and the integration and snaps suites are executed using the installed snapcraft.

To run them locally, the easiest way is to use a LXC container. From the root of the project, run:

    sudo apt install autopkgtest
    adt-run --unbuilt-tree . --apt-upgrade --- lxd ubuntu:xenial

It's possible to select only one of the suites, with:

    adt-run --unbuilt-tree . --apt-upgrade --testname=integrationtests --- lxd ubuntu:xenial

or:

    adt-run --unbuilt-tree . --apt-upgrade --testname=snapstests --- lxd ubuntu:xenial

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
