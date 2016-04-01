# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


import getpass
import logging
import os
import subprocess
import tempfile

import yaml

from snapcraft import (
    config,
    storeapi,
)


logger = logging.getLogger(__name__)


def login():
    print('Enter your Ubuntu One SSO credentials.')
    email = input('Email: ')
    password = getpass.getpass('Password: ')
    otp = input('One-time password (just press enter if you don\'t use '
                'two-factor authentication): ')

    logger.info('Authenticating against Ubuntu One SSO.')
    with_macaroons = os.environ.get('SNAPCRAFT_WITH_MACAROONS', False)
    if with_macaroons:
        do_login = storeapi.login_with_macaroons
    else:
        do_login = storeapi.login
    response = do_login(email, password, token_name='snapcraft', otp=otp)
    success = response.get('success', False)

    if success:
        config.save_config(response['body'])
        logger.info('Login successful.')
    else:
        logger.info('Login failed.')


def logout():
    logger.info('Clearing credentials for Ubuntu One SSO.')
    config.clear_config()
    logger.info('Credentials cleared.')


def upload(snap_filename):
    if not os.path.exists(snap_filename):
        raise FileNotFoundError(snap_filename)
    else:
        snap_name = _get_name_from_snap_file(snap_filename)
        logger.info('Uploading existing {}.'.format(snap_filename))

        conf = config.load_config()
        storeapi.upload(snap_filename, snap_name, config=conf)


def _get_name_from_snap_file(snap_path):
    with tempfile.TemporaryDirectory() as temp_dir:
        subprocess.check_call(
            ['unsquashfs', '-d', os.path.join(temp_dir, 'squashfs-root'),
             snap_path, '-e', os.path.join('meta', 'snap.yaml')])
        with open(os.path.join(
                temp_dir, 'squashfs-root', 'meta', 'snap.yaml')) as yaml_file:
            snap_yaml = yaml.load(yaml_file)

    return snap_yaml['name']
