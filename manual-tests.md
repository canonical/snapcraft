# Test log in with one-time password

1. Set up an SSO account with two-factor authentication.
2. Run snapcraft logout
3. Run snapcraft login
4. Enter the email address.
5. Enter the password.
6. Enter the one-time password.

   * Check that the log in was successful.


# Test file ownership is retained

1. 'snapcraft build' a simple snap
2. sudo touch install/test-owner-file
3. sudo chown nobody:nogroup install/test-owner-file
4. sudo snapcraft prime
5. ensure that prime/test-owner-file is owned by nobody and nogroup


# Test stage package caching

1. `snapcraft pull` a snap that has `parts` with `stage-packages`.
2. Run `snapcraft clean`.
3. Verify there is cached apt data in `~/.cache/snapcraft/<hash>/`
4. Run `snapcraft pull` again and notice the download is minimal.
5. Wipe the cached apt data.
6. Run `snapcraft pull` again and notice the download is as in `1.`.
7. Run this test again, but run snapcraft on a partition separated
   from $HOME.


# Test cleanbuild with debug shell

1. Run `snapcraft cleanbuild --debug` for a snap.
2. Insert an error such that the code will fail to compile or
   mistype the name of an entry in `stage-packages`.
3. Run `snapcraft cleanbuild --debug` again.
4. Ensure you are dropped into a debug shell.
5. Exit the shell.
6. Ensure you are dropped back into your original shell session.


# Test cleanbuild with non-ascii characters in the desktop file.

1. Run `snapcraft cleanbuild` for the snap in `integration_tests/snaps/desktop-with-non-ascii`
   * Check that the build succeeds and you get the `.snap` file.


# Test cleanbuild with a remote.

1. Setup a remote as described on
   https://linuxcontainers.org/lxd/getting-started-cli/#multiple-hosts
2. Select a project to build.
3. Run `snapcraft cleanbuild --remote <remote>` where `<remote>` is
   the name you gave the remote on step 1.


# Test containerized building

1. Setup LXD as described on
   https://linuxcontainers.org/lxd/getting-started-cli/
2. Select a project <project> to build.
3. Run `SNAPCRAFT_CONTAINER_BUILDS=1 snapcraft`.
4. Run `SNAPCRAFT_CONTAINER_BUILDS=1 snapcraft clean` and observe that
   build folders as well as the container `snapcraft-<project>` is gone.


# Test cross-compilation with Go

1. Go to integration_tests/snaps/go-hello.
2. Run `snapcraft snap --target-arch=armhf`.
3. Copy the snap to a Raspberry Pi.
4. Install the snap.
5. Run `go-hello`.


# Test cross-compilation with Rust

1. Go to integration_tests/snaps/rust-hello.
2. Run `snapcraft snap --target-arch=armhf`.
3. Copy the snap to a Raspberry Pi.
4. Install the snap.
5. Run `rust-hello`.


# Test cross-compilation with Autotools

1. Go to integration_tests/snaps/autotools-hello.
2. Run `snapcraft snap --target-arch=armhf`.
3. Copy the snap to a Raspberry Pi.
4. Install the snap.
5. Run `autotools-hello`.


# Test cross-compilation with Waf

1. Go to integration_tests/snaps/waf-with-configflags.
2. Run `snapcraft snap --target-arch=armhf`.
3. Copy the snap to a Raspberry Pi.
4. Install the snap.
5. Run `waf-with-configflags`.


# Test the PC kernel.

1. Get the PC kernel source:

    $ git clone -b pc https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux-snap/+git/xenial
    $ cd xenial

2. Run `sudo snapcraft`.
3. Create a file called `pc-model.json` with the following contents:

    {
        "type": "model",
        "authority-id": "$account_id",
        "brand-id": "$account_id",
        "series": "16",
        "model": "pc",
        "architecture": "amd64",
        "gadget": "pc",
        "kernel": "$kernel_snap_path",
        "timestamp": "$date"
    }

4. Replace `$account_id` with the value from https://myapps.developer.ubuntu.com/dev/account/
5. Replace `$kernel_snap_path` with the path to the snap you just created.
6. Replace `$date` with the output of the command `date -Iseconds --utc`.
7. If you haven't created a key, run the following command, replacing
   `$key_name` with a name for your key:

    $ snap create-key $key_name
    $ snapcraft register-key

8. Sign the model:

    $ cat pc-model.json | snap sign -k $key_name > pc.model

10. Install ubuntu-image:

    $ sudo apt install ubuntu-image

11. Create the image:

    $ sudo ubuntu-image --image-size 3G -O ubuntu-core-16 pc.model --extra-snaps $kernel_snap_path

12. Start the image in kvm:

    $ kvm -smp 2 -m 1500 -netdev user,id=mynet0,hostfwd=tcp::8022-:22,hostfwd=tcp::8090-:80 -device virtio-net-pci,netdev=mynet0 -drive file=ubuntu-core-16/pc.img,format=raw

  * Check that the user can be created.
  * Check that it's possible to ssh into the vm.
  * Check that it's possible to install a snap.


# Test the dragonboard 410c kernel.

1. Download https://developer.qualcomm.com/download/db410c/linux-board-support-package-v1.2.zip
2. Extract it and copy the file `firmware.tar` to the directory `demos/96boards-kernel`.
3. Run `snapcraft snap --target-arch arm64` in the `demos/96boards-kernel` directory.
4. Create a file called `dragonboard-model.json` with the following contents:

    {
        "type": "model",
        "authority-id": "$account_id",
        "brand-id": "$account_id",
        "series": "16",
        "model": "dragonboard",
        "architecture": "arm64",
        "gadget": "dragonboard",
        "kernel": "$kernel_snap_path",
        "timestamp": "$date"
    }

5. Replace `$account_id` with the value from https://myapps.developer.ubuntu.com/dev/account/
6. Replace `$kernel_snap_path` with the path to the snap you just created.
7. Replace `$date` with the output of the command `date -Iseconds --utc`.
8. If you haven't created a key, run the following command, replacing
   `$key_name` with a name for your key:

    $ snap create-key $key_name
    $ snapcraft register-key

9. Sign the model:

    $ cat dragonboard-model.json | snap sign -k $key_name > dragonboard.model

10. Install ubuntu-image:

    $ sudo apt install ubuntu-image

11. Create the image:

    $ sudo ubuntu-image -O ubuntu-core-16 dragonboard.model --extra-snaps $kernel_snap_path

12. Insert an sdcard into the host PC.
13. Umount the sdcard partitions.
14. Flash the image, replacing sdX with the path to the sdcard:

    $ sudo dd if=ubuntu-core-16/dragonboard.img of=/dev/sdX bs=32M
    $ sync

15. Insert the sdcard into the dragonboard, and turn it on.

  * Check that the user can be created.
  * Check that it's possible to ssh into the board.
  * Check that it's possible to install a snap.


# Test installing with `pip`

1. Follow HACKING.md to install using `pip` without using --editable.
2. Make sure Snapcraft works by running `snapcraft init` followed by `snapcraft`.
3. Follow HACKING.md to install using `pip` while using --editable.
4. Repeat step 2.
