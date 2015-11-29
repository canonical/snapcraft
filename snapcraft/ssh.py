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

import logging
import os


logger = logging.getLogger(__name__)


def get_latest_private_key():
    """Return the latest private key in ~/.ssh.

    :returns:
        Path of the most-recently-modified private SSH key
    :raises LookupError:
        If no such key was found.

    This function tries to mimic the logic found in ``ubuntu-device-flash``. It
    will look for the most recently modified private key in the users' SSH
    configuration directory.
    """
    candidates = []
    ssh_dir = os.path.expanduser('~/.ssh/')
    for filename in os.listdir(ssh_dir):
        # Skip public keys, we want the private key
        if filename.endswith('.pub'):
            continue
        ssh_key = os.path.join(ssh_dir, filename)
        # Skip non-files
        if not os.path.isfile(ssh_key):
            continue
        # Ensure that it is a real ssh key
        with open(ssh_key, 'rb') as stream:
            if stream.readline() != b'-----BEGIN RSA PRIVATE KEY-----\n':
                continue
        candidates.append(ssh_key)
    # Sort the keys by modification time, pick the most recent key
    candidates.sort(key=lambda f: os.stat(f).st_mtime, reverse=True)
    logger.debug('Available ssh public keys: %r', candidates)
    if not candidates:
        raise LookupError('Unable to find any private ssh key')
    return candidates[0]
