# Snapcraft

## Evaluating pull requests

Oftentimes all you want to do is see if a given pull request solves the issue you were having (you reported that bug, right?). You can always run that pull request from source if you want (see below), but there's an easier way: use `snapcraft-pr`. First of all, you need it installed:

    sudo snap install --classic snapcraft-pr

Every pull request on github has an associated ID, which is used across the project to refer to it in comments, etc. It's clearly displayed on each pull request, but you can also determine it from the URL, for example, the ID of `https://github.com/snapcore/snapcraft/pull/2094` is `2094`.

Once you've determined the pull request ID, you can tell `snapcraft-pr` that you want to use it by running:

    snapcraft-pr.init <pull request id>

Once that has completed, you can now use that version of snapcraft almost exactly like how you would normally use the `snapcraft` command, but instead of using `snapcraft`, use `snapcraft-pr <pull request id>`. For example, if you would normally run `snapcraft cleanbuild`, run `snapcraft-pr <pull request id> cleanbuild`.

If the pull request gets updated after you've already done this, you can update your version by simply running `snapcraft-pr.init <pull request id>` again.


## Installing from pip

First install a few dependencies:

    sudo apt install gcc g++ make python3-dev python3-venv libffi-dev libsodium-dev libapt-pkg-dev squashfs-tools patchelf execstack rpm2cpio p7zip-full

Create and activate a new virtual environment:

    mkdir -p ~/venv/snapcraft
    python3 -m venv ~/venv/snapcraft
    source ~/venv/snapcraft/bin/activate


Make sure pip is up-to-date:

    (snapcraft) $ pip install --upgrade pip


Install snapcraft (and its dependencies):

    (snapcraft) $ pip install -r requirements.txt .


## Running

To see all the commands and options, run `snapcraft --help`.


## Hacking

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/snapcore/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Our mailing list is snapcraft@lists.ubuntu.com
- We can also be found on the #snappy IRC channel on Freenode


### Installing for development

You'll need to install the development dependencies, and you'll also probably want to install it in `editable` mode so any changes you make take affect:

    (snapcraft) $ pip install -r requirements.txt -r requirements-devel.txt --editable .


### Testing

See the [Testing guide](TESTING.md).

### Project Layout

- **bin:** Holds the main snapcraft script. Putting this bin in your PATH or directly running scripts from it will find the rest of the source tree automatically.

- **examples:** Entering the subdirectories and running `../../bin/snapcraft snap` will generally yield interesting results. These examples will give you an idea of what snapcraft can do. These examples are not used during automated testing, they are simply for experimenting.

- **plugins:** Holds yaml metadata for the current snapcraft plugins.

- **tests:** Tests, obviously. `unit` holds Python unit tests and `plainbox` holds plainbox integration tests.

- **snapcraft:** The Python module that houses the core snapcraft logic. The `plugins` subdirectory holds the code for each plugin.


### Updating library filter

To update the list of libraries that get excluded from inclusion into a
snap run:

    ./libraries/generate_lib_list.py libraries/<release>

e.g.; to update the list for 16.04,

    ./libraries/generate_lib_list.py libraries/16.04

### Enabling debug output

Given that the `--debug` option in snapcraft is reserved for project specific debugging, enabling for the `logger.debug` calls is achieved by setting the "SNAPCRAFT_ENABLE_DEVELOPER_DEBUG" environment variable to a truthful value. Snapcraft's internal tools, e.g.; `snapraftctl` should pick up this environment variable as well.
