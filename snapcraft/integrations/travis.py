# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Snapcraft integration for Travis (CI).

This is an *EXPERIMENTAL* feature and subject to incompatible changes in
the future, please use with caution.

This command currently depends on a working `travis` CLI environment and
a previously initialized Travis project (`.travis.yml`).

Make sure your Travis project is also configured to "Build pushes", this
way every new push to `master` will result in a new snap revision in the
Store.

This operation will acquire properly attenuated Store credentials and
encrypt them for use in your testbed (`.snapcraft/travis_snapcraft.cfg`),
only Travis has the private key to decrypt it and will be only available
to branches of the same repository, not forks.

Then it will adjust Travis configuration ('.travis.yml') with the commands
to decrypt credentials during 'after_success' phase and install latest
`snapcraft` to build and release your snap (inside a snapcore/snapcraft docker
container) during the 'deploy' phase.

See the example below::

    sudo: required
    services:
    - docker
    after_success:
    - openssl aes-256-cbc -K <travis-key> -iv <travis-iv>
      -in .snapcraft/travis_snapcraft.cfg
      -out .snapcraft/snapcraft.cfg -d
    deploy:
      skip_cleanup: true
      provider: script
      script: docker run -v $(pwd):$(pwd) -t snapcore/snapcraft sh -c
        "apt update -qq && cd $(pwd) &&
        snapcraft && snapcraft push *.snap --release edge"
      on:
        branch: master

The dedicated credentials will be functional for exactly 1 year or until
a password change on the related Ubuntu One SSO account. In order to
refresh the project credentials, please run the following command::

    $ snapcraft enable-ci travis --refresh
"""
from contextlib import ExitStack, contextmanager
import logging
import os
import subprocess
import tempfile
import yaml

from snapcraft import storeapi
from snapcraft.file_utils import requires_command_success, requires_path_exists
from snapcraft.internal import project_loader
from snapcraft._store import login
from snapcraft.config import LOCAL_CONFIG_FILENAME

logger = logging.getLogger(__name__)

TRAVIS_CONFIG_FILENAME = ".travis.yml"
ENCRYPTED_CONFIG_FILENAME = ".snapcraft/travis_snapcraft.cfg"


class TravisRuntimeError(Exception):
    """Local runtime error to not confuse importlib.

    Raising `RuntimeError` from a module imported in runtime
    results in python hanging instead of just exiting.
    """


def _acquire_and_encrypt_credentials(packages, channels):
    """Acquire and encrypt Store credentials for Travis jobs."""
    # XXX cprov 20161116: Needs caveat syntax for restricting origins
    # (IP or reverse-dns) but Travis sudo-enabled containers, needed for
    # running xenial snapcraft, do not have static egress routes.
    # See https://docs.travis-ci.com/user/ip-addresses.
    logger.info("Acquiring specific authorization information ...")
    store = storeapi.StoreClient()
    # Travis cannot register new names or add new developers.
    acls = ["package_access", "package_push", "package_release"]
    if not login(
        store=store, acls=acls, packages=packages, channels=channels, save=False
    ):
        raise TravisRuntimeError("Cannot continue without logging in successfully.")

    logger.info(
        "Encrypting authorization for Travis and adjusting project to "
        'automatically decrypt and use it during "after_success".'
    )
    with tempfile.NamedTemporaryFile(mode="w") as fd:
        store.conf.save(config_fd=fd)
        fd.flush()
        os.makedirs(os.path.dirname(LOCAL_CONFIG_FILENAME), exist_ok=True)
        cmd = [
            "travis",
            "encrypt-file",
            "--force",
            "--add",
            "after_success",
            "--decrypt-to",
            LOCAL_CONFIG_FILENAME,
            fd.name,
            ENCRYPTED_CONFIG_FILENAME,
        ]
        try:
            subprocess.check_output(cmd, stderr=subprocess.PIPE)
        except subprocess.CalledProcessError as err:
            raise TravisRuntimeError(
                "`travis encrypt-file` failed: {}\n{}".format(
                    err.returncode, err.stderr.decode()
                )
            )


@contextmanager
def requires_travis_preconditions():
    """Verify all Travis CI integration preconditions."""
    required = (
        requires_command_success(
            "travis settings",
            not_found_fmt=(
                "Travis CLI (`{cmd_list[0]}`) is not available.\n"
                "Please install it before trying this command again:\n\n"
                "    $ sudo apt install ruby-dev ruby-ffi libffi-dev\n"
                "    $ sudo gem install travis\n"
            ),
            failure_fmt=(
                "Travis CLI (`{command}`) is not functional or you are not "
                "allowed to access this repository settings.\n"
                "Make sure it works correctly in your system before trying "
                "this command again."
            ),
        ),
        requires_command_success(
            "git status",
            not_found_fmt=(
                "Git (`{cmd_list[0]}`) is not available, this tool cannot "
                "verify its prerequisites.\n"
                "Please install it before trying this command again:\n\n"
                "    $ sudo apt install git\n"
            ),
            failure_fmt=(
                "The current directory is not a Git repository.\n"
                "Please switch to the desired project repository where "
                "Travis should be enabled."
            ),
        ),
        requires_path_exists(
            TRAVIS_CONFIG_FILENAME,
            error_fmt=(
                "Travis project is not initialized for the current "
                "directory.\n"
                "Please initialize Travis project (e.g. `travis init`) with "
                "appropriate parameters."
            ),
        ),
    )
    with ExitStack() as cm:
        [cm.enter_context(c) for c in required]
        yield


@requires_travis_preconditions()
def refresh(project):
    series = storeapi.constants.DEFAULT_SERIES
    project_config = project_loader.load_config(project)
    snap_name = project_config.data["name"]
    logger.info(
        'Refreshing credentials to push and release "{}" snaps '
        "to edge channel in series {}".format(snap_name, series)
    )

    packages = [{"name": snap_name, "series": series}]
    channels = ["edge"]
    _acquire_and_encrypt_credentials(packages, channels)

    logger.info(
        "Done. Please commit the changes to `{}` file.".format(
            ENCRYPTED_CONFIG_FILENAME
        )
    )


@requires_travis_preconditions()
def enable(project):
    series = storeapi.constants.DEFAULT_SERIES
    project_config = project_loader.load_config(project)
    snap_name = project_config.data["name"]
    logger.info(
        "Enabling Travis testbeds to push and release {!r} snaps "
        "to edge channel in series {!r}".format(snap_name, series)
    )

    packages = [{"name": snap_name, "series": series}]
    channels = ["edge"]
    _acquire_and_encrypt_credentials(packages, channels)

    logger.info(
        'Configuring "deploy" phase to build and release the snap in the ' "Store."
    )
    with open(TRAVIS_CONFIG_FILENAME, "r+") as fd:
        travis_conf = yaml.safe_load(fd)
        # Enable 'sudo' capability and 'docker' service.
        travis_conf["sudo"] = "required"
        services = travis_conf.setdefault("services", [])
        if "docker" not in services:
            services.append("docker")
        # Add a 'deploy' section with 'script' provider for building and
        # release the snap within a xenial docker container.
        travis_conf["deploy"] = {
            "skip_cleanup": True,
            "provider": "script",
            "script": (
                "docker run -v $(pwd):$(pwd) -t snapcore/snapcraft sh -c "
                '"apt update -qq && cd $(pwd) && '
                'snapcraft && snapcraft push *.snap --release edge"'
            ),
            "on": {"branch": "master"},
        }
        fd.seek(0)
        yaml.dump(travis_conf, fd, default_flow_style=False)

    logger.info(
        "Done. Now you just have to review and commit changes in your "
        "Travis project (`{}`).\n"
        "Also make sure you add the new `{}` file.".format(
            TRAVIS_CONFIG_FILENAME, ENCRYPTED_CONFIG_FILENAME
        )
    )
