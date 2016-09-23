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

This command currently depends on working `travis` CLI environment.

It will acquire properly attenuated Store credentials and encrypt it for
use in your testbed ('.travis_snapcraft.cfg'), only Travis has the private
key to decrypt it.

Then it will adjust Travis configuration ('.travis.yml') with the commands
to decrypt credentials and install and run `snapcraft` to release your snap
(inside a ubuntu:xenial docker container) during the 'after_success' phase.
See the example below::

    sudo: required
    services:
    - docker
    after_success:
    - openssl aes-256-cbc -K <travis-key> -iv <travis-iv>
      -in .travis_snapcraft.cfg -out .snapcraft.cfg -d
    - docker run -v $(pwd):$(pwd) -t ubuntu:xenial
      sh -c "apt update -qq && apt install snapcraft -y && cd $(pwd) &&
      snapcraft && snapcraft push *.snap --release edge"
"""
import logging
import subprocess
import tempfile
import yaml

from snapcraft import storeapi
from snapcraft.integrations import (
    requires_command_success,
    requires_path_exists,
)
from snapcraft.internal import load_config
from snapcraft._store import _login


logger = logging.getLogger(__name__)

TRAVIS_CONFIG_FILENAME = '.travis.yml'
ENCRYPTED_CONFIG_FILENAME = '.travis_snapcraft.cfg'


def _encrypt_config(config_path):
    """Encrypt given snapcraft config file for Travis jobs."""
    cmd = [
        'travis', 'encrypt-file',
        '--force',
        '--add', 'after_success',
        '--decrypt-to', '.snapcraft.cfg',
        config_path, ENCRYPTED_CONFIG_FILENAME,
    ]
    try:
        subprocess.check_output(cmd, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as err:
        raise RuntimeError(
            '`travis encrypt-file` failed: {}\n{}'.format(
                err.returncode, err.stderr.decode()))


@requires_command_success(
    'travis version',
    EnvironmentError(
        'Travis CLI (`travis`) is not available.\n'
        'Please install it (e.g. `sudo gem install travis`) '
        'before trying this command again.'),
    EnvironmentError(
        'Travis CLI (`travis version`) is not functional.\n'
        'Make sure it works correctly in your system '
        'before trying this command again.'))
@requires_command_success(
    'git status',
    EnvironmentError(
        'Git (`git`) is not available, this tool cannot verify '
        'its prerequisites.\n'
        'Please install it (e.g. `sudo apt install git`) '
        'before trying this command again.'),
    EnvironmentError(
        'The current directory is not a Git repository.\n'
        'Please switch to the desired project repository where '
        'Travis should be enabled.'))
@requires_path_exists(
    TRAVIS_CONFIG_FILENAME,
    'Travis project is not initialised for the current directory.\n'
    'Please initialise Travis project (e.g. `travis init`) with '
    'appropriate parameters.')
def enable():
    series = storeapi.constants.DEFAULT_SERIES
    project_config = load_config()
    snap_name = project_config.data['name']
    logger.info(
        'Enabling Travis testbeds to push and release "{}" snaps '
        'to edge channel in series {}'.format(snap_name, series)
    )

    packages = [{'name': snap_name, 'series': series}]
    channels = ['edge']

    # XXX cprov 20161116: Needs caveat syntax for restricting
    # origins (IP or reverse-dns) but Travis sudo-enabled containers
    # do not have static egress routes.
    # See https://docs.travis-ci.com/user/ip-addresses.

    logger.info('Acquiring specific authorization information ...')
    store = storeapi.StoreClient()
    if not _login(store, packages=packages, channels=channels, save=False):
        raise RuntimeError(
            'Cannot continue without logging in successfully.')

    logger.info(
        'Encrypting authorization for Travis and adjusting project to '
        'automatically decrypt and use it during "after_success".')

    with tempfile.NamedTemporaryFile(mode='w') as fd:
        store.conf.parser.write(fd)
        fd.flush()
        _encrypt_config(fd.name)

    with open(TRAVIS_CONFIG_FILENAME, 'r+') as fd:
        travis_conf = yaml.load(fd)
        # Enable 'sudo' capability and 'docker' service.
        travis_conf['sudo'] = 'required'
        services = travis_conf.setdefault('services', [])
        if 'docker' not in services:
            services.append('docker')
        # Append a docker-run command to build and release the snap.
        travis_conf['after_success'].append(
            'docker run -v $(pwd):$(pwd) -t ubuntu:xenial '
            'sh -c "apt-update -qq && apt install snapcraft -y && '
            'cd $(pwd) && snapcraft && snapcraft push *.snap --release edge"')
        fd.seek(0)
        yaml.dump(travis_conf, fd, default_flow_style=False)

    logger.info(
        'Done. Now you just have to review and commit changes in your '
        'Travis project (`{}`).\n'
        'Also make sure you add the new `{}` file.'.format(
            TRAVIS_CONFIG_FILENAME, ENCRYPTED_CONFIG_FILENAME))
