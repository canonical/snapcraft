# Snapcraft tests

## Manual tests

Snapcraft has a few manual tests documented in [manual-tests.md](manual-tests.md).
These are tests for features that are not possible to automate, or that the complexity of automating them is too high while the time it takes to run them by hand is not too much.

We try very hard to automate as much as possible and to keep the manual test suite as small as possible. Please do not add a manual test before discussing it with the team, and make sure that there are very good reasons not to automate it.

When running manual tests against the production store, make sure to use the test user email account and the well-known test prefixes, since these snaps will pollute the store. To register a user, use an email address like snapcraft-test+<your-user-name>-<unique-id>@canonical.com (e.g., snapcraft-test+elopio-123@canonical.com). This will send the confirmation email and any notifications to the inbox of snapcraft-test@canonical.com, and the snapcraft team has the password to access that inbox. To register a snap or anything else that has a name, like tracks, keys, etc., prefix the name with test-snapcraft and your name (e.g., test-snapcraft-snap-elopio-123, test-snapcraft-track-elopio-111).

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

At any time, an integration test may fail and given the use of temporary directories it can be hard to inspect what went on. When working on a specific test case you can set the environment variable `SNAPCRAFT_TEST_KEEP_DATA_PATH` to a directory path for the sepecic test.
This mechanism will only work when working with individual tests and will fail to run with a batch of them.

### Slow tests

Some tests take too long. This affects the pull requests because we have to wait for a long time, and they will make Travis CI timeout because we have only 50 minutes per suite in there. The solution is to tag these tests as slow, and don't run them in all pull requests. These tests will only be run in autopkgtests.

To mark a test case as slow, set the class attribute `slow_test = True`.

To run all the tests, including the slow ones, set the environment variable `SNAPCRAFT_SLOW_TESTS=1`.

### Snaps tests

The snaps tests is a suite of high-level tests that try to simulate real-world scenarios of a user interacting with snapcraft. They cover the call to snapcraft to generate a snap file from the source files of a fully functional project, the installation of the resulting snap, and the execution of the binaries and services of this snap.

These tests are in the `snaps_tests` directory, with the sources for the test snaps in the `demos` directory.

### Setting up the environment

In order to run these tests suites, first you will need to set up your development environment. Follow the steps in the [Hacking guide](HACKING.md) to install for development.

### Running the tests

To run the static tests, execute:

    make tests-static

To run the unit tests, execute:

    make test-units

...or use pytest directly:

    pytest tests/unit

You can also run a subsuite of the unit suites specifying the path to the directory.
For example:

  * To run only the unit tests for the plugins:

    ```
    pytest tests/unit/plugins
    ```

  * To run only the integration tests for the store:

    ```
    pytest tests/integration/store
    ```

The snaps tests script has more complex arguments. For an explanation of them, run:

    python3 -m snaps_tests -h

The integration and snaps suites can be run using the snapcraft source from the repository, or using the snapacraft command installed in the system. By default, they will use the source code, so you can modify your clone of the repository and verify that your changes are correct. If instead you want to verify that the snapcraft version installed in your system is correct, run them with the environment variable `SNAPCRAFT_PACKAGE_TYPE` set to either "snap" or "deb", like this:

    SNAPCRAFT_PACKAGE_TYPE=snap ./runtests.sh tests/integration

or

    SNAPCRAFT_PACKAGE_TYPE=type ./runtests.sh tests/integration

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

## Testing on macOS

We can currently run a minimal subset of snapcraft integration tests on macOS. They are run by travis, using a brew formula that builds the installer from the branch. Once the virtualization tool for Ubuntu on mac is finalized, the full suite can be executed.

For manual exploratory testing, the team has one mac machine available.

## Autopkgtests for the snapcraft deb

Autopkgtests are tests for the project packaged as a deb. The unit tests are run during autopkgtests while the snapcraft deb is being built. Then the resulting deb is installed, and the integration and snaps suites are executed using the installed snapcraft.


### How to run on Xenial

The easiest way is to use a LXC container. From the root of the project, run:

    sudo apt install autopkgtest
    adt-run --unbuilt-tree . --apt-upgrade --- lxd ubuntu:xenial

It's possible to select only one of the suites using `--testname`, for example:

    adt-run --unbuilt-tree . --apt-upgrade --testname=integrationtests --- lxd ubuntu:xenial


### How to run on Bionic

The easiest way is to use a LXC container. From the root of the project, run:

    sudo apt install autopkgtest
    autopkgtest . -U -- lxd ubuntu:xenial

It's possible to select only one of the suites using `--test-name`, for example:

    autopkgtest . -U --test-name=integrationtests-spread -- lxd ubuntu:xenial


## Spread tests for the snapcraft snap

[Spread](https://github.com/snapcore/spread) is a system to distribute tests and execute them in different backends, in parallel. We are currently using spread only to run the integration suite using the installed snapcraft snap from the edge channel.

To run them, first, download the spread binary:

    curl -s -O https://storage.googleapis.com/snapd-spread-tests/spread/spread-amd64.tar.gz && tar xzvf spread-amd64.tar.gz

Then, you can run them using a local LXD as the backend with:

    ./spread -v lxd:

Or, you can run them in google if you have a `SPREAD_GOOGLE_KEY`, with:

    SPREAD_GOOGLE_KEY={key} ./spread -v google:

## Testing arm

It is possible to emulate an arm64 machine on an amd64 host, which is very useful for running manual exploratory tests for snapcraft. To set it up:

1. Download the latest ubuntu arm64 uefi image from https://cloud-images.ubuntu.com/releases/16.04/release/
2. Keep a pristine copy of the image, in case you want to reset the machine, replacing <ubuntu-image> with the name of the file you downloaded on step 1:

    ```
    $ cp <ubuntu-image> <ubuntu-image>.pristine
    ```

3. Download the latest UEFI firmware image QEMU_EFI.fd from https://releases.linaro.org/components/kernel/uefi-linaro/latest/release/qemu64/
4. Create a cloud init file, replacing <launchpad-user-name> with your values:

    ```
    $ cat > cloud-data.yaml << EOF
    #cloud-config
    users:
      - name: $USER
        ssh-import-id: <launchpad-user-name>
        sudo: ['ALL=(ALL) NOPASSWD:ALL']
        groups: sudo
        shell: /bin/bash
    EOF
    ```

5. Create a cloud config disk image on the file `cloud-config.img`:

    ```
    $ sudo apt install --yes cloud-image-utils
    $ cloud-localds --disk-format qcow2 cloud-config.img cloud-data.yaml
    ```

6. Run the image in qemu, replacing <ubuntu-image> with the path of the file you downloaded on step 1.

    ```
    $ sudo apt install qemu-system-arm
    $ qemu-system-aarch64 \
        -smp 2 \
        -m 1024 \
        -M virt \
        -cpu cortex-a57 \
        -bios QEMU_EFI.fd \
        -nographic \
        -device virtio-blk-device,drive=image \
        -drive if=none,id=image,file=<ubuntu-image> \
        -device virtio-blk-device,drive=cloud \
        -drive if=none,id=cloud,file=cloud-config.img \
        -device virtio-net-device,netdev=user0 \
        -netdev user,id=user0 \
        -redir tcp:2222::22
    ```

This will show a few errors, and a weird screen while the machine boots.
TODO: research how to make it nicer, but for now, just be patient until the login prompt appears.

7. ssh into the emulated machine:

    ```
    $ ssh -p 2222 localhost
    ```

(Source: https://gist.github.com/george-hawkins/16ee37063213f348a17717a7007d2c79)

To test snapcraft on an armhf machine, currently the only simple option is to install ubuntu classic on BeagleBoard (https://elinux.org/BeagleBoardUbuntu) or on Raspberry Pi 2 (https://wiki.ubuntu.com/ARM/RaspberryPi).
