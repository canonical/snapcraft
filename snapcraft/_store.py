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
import subprocess
import tempfile
import os

import yaml

from snapcraft import storeapi


logger = logging.getLogger(__name__)


def _get_name_from_snap_file(snap_path):
    with tempfile.TemporaryDirectory() as temp_dir:
        output = subprocess.check_output(
            ['unsquashfs', '-d',
             os.path.join(temp_dir, 'squashfs-root'),
             snap_path, '-e', os.path.join('meta', 'snap.yaml')])
        logger.debug(output)
        with open(os.path.join(
                temp_dir, 'squashfs-root', 'meta', 'snap.yaml')
        ) as yaml_file:
            snap_yaml = yaml.load(yaml_file)

    return snap_yaml['name']


def login():
    print('Enter your Ubuntu One SSO credentials.')
    email = input('Email: ')
    password = getpass.getpass('Password: ')
    one_time_password = input(
        'One-time password (just press enter if you don\'t use two-factor '
        'authentication): ')

    logger.info('Authenticating against Ubuntu One SSO.')
    store = storeapi.StoreClient()
    try:
        store.login(
            email, password, one_time_password=one_time_password)
    except (storeapi.errors.InvalidCredentialsError,
            storeapi.errors.StoreAuthenticationError):
        logger.info('Login failed.')
        return False
    else:
        logger.info('Login successful.')
        return True


def logout():
    logger.info('Clearing credentials for Ubuntu One SSO.')
    store = storeapi.StoreClient()
    store.logout()
    logger.info('Credentials cleared.')


def register(snap_name):
    logger.info('Registering {}.'.format(snap_name))
    store = storeapi.StoreClient()
    try:
        store.register(snap_name)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise
    logger.info("Congratulations! You're now the publisher for {!r}.".format(
        snap_name))


def upload(snap_filename):
    if not os.path.exists(snap_filename):
        raise FileNotFoundError(
            'The file {!r} does not exist.'.format(snap_filename))

    logger.info('Uploading {}.'.format(snap_filename))

    snap_name = _get_name_from_snap_file(snap_filename)
    try:
        store = storeapi.StoreClient()
        tracker = store.upload(snap_name, snap_filename)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise

    result = tracker.track()

    logger.info('Revision {!r} of {!r} created.'.format(
        result['revision'], snap_name))


def download(snap_name, channel, download_path, arch):
    """Download snap from the store to download_path"""
    try:
        store = storeapi.StoreClient()
        store.download(snap_name, channel, download_path, arch)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise
    except storeapi.errors.SnapNotFoundError:
        raise RuntimeError(
            'Snap {name} for {arch} cannot be found'
            ' in the {channel} channel'.format(name=snap_name, arch=arch,
                                               channel=channel))
    except storeapi.errors.SHAMismatchError:
        raise RuntimeError(
            'Failed to download {} at {} (mismatched SHA)'.format(
                snap_name, download_path))
