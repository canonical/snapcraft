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
    storeapi,
)


logger = logging.getLogger(__name__)


def login():
    print('Enter your Ubuntu One SSO credentials.')
    email = input('Email: ')
    password = getpass.getpass('Password: ')
    one_time_password = input('One-time password (just press enter if you'
                              ' don\'t use two-factor authentication): ')

    logger.info('Authenticating against Ubuntu One SSO.')
    store = storeapi.SCAClient()
    response = store.login(email, password, one_time_password)
    success = response.get('success', False)

    if success:
        logger.info('Login successful.')
    else:
        logger.info('Login failed.')


def logout():
    logger.info('Clearing credentials for Ubuntu One SSO.')
    store = storeapi.SCAClient()
    store.logout()
    logger.info('Credentials cleared.')


def register_name(snap_name):
    logger.info('Registering {}.'.format(snap_name))
    store = storeapi.SCAClient()
    if store.conf.get('package_upload') is None:
        logger.info('Registration failed.')
        logger.info(
            'No valid credentials found. Have you run "snapcraft login"?')
        return

    response = store.register_name(snap_name)
    if response.ok:
        logger.info('Congrats! You\'re now the publisher for "{}".'.format(
            snap_name))
    else:
        logger.info('Registration failed.')


def upload(snap_filename):
    if not os.path.exists(snap_filename):
        raise FileNotFoundError(snap_filename)

    snap_name = _get_name_from_snap_file(snap_filename)
    logger.info('Uploading existing {}.'.format(snap_filename))

    try:
        store = storeapi.SCAClient()
        result = store.upload(snap_filename, snap_name)
    except storeapi.InvalidCredentials:
        logger.info('No valid credentials found.'
                    ' Have you run "snapcraft login"?')
        return

    success = result.get('success', False)
    errors = result.get('errors', [])
    app_url = result.get('application_url', '')
    revision = result.get('revision')

    # Print another newline to make sure the user sees the final result of the
    # upload (success/failure).
    logger.info('')

    if success:
        message = 'Application uploaded successfully'
        if revision:
            message += ' (as revision {})'.format(revision)

        logger.info(message)
    else:
        logger.info('Upload did not complete.')

    if errors:
        logger.info('Some errors were detected:\n\n%s\n',
                    '\n'.join(str(error) for error in errors))

    if app_url:
        logger.info('Please check out the application at: %s\n',
                    app_url)


def download(snap_name, channel, download_path, arch):
    """Download snap from the store to download_path"""
    try:
        store = storeapi.SCAClient()
        store.download(snap_name, channel, download_path, arch)
    except storeapi.InvalidCredentials:
        logger.info('No valid credentials found.'
                    ' Have you run "snapcraft login"?')
    except storeapi.SnapNotFound:
        raise RuntimeError(
            'Snap {name} for {arch} cannot be found'
            ' in the {chanel} channel'.format(name=snap_name, arch=arch,
                                              channel=channel))
    except storeapi.SHAMismatch:
        raise RuntimeError(
            'Failed to download {} at {} (mismatched SHA)'.format(
                snap_name, download_path))


def _get_name_from_snap_file(snap_path):
    with tempfile.TemporaryDirectory() as temp_dir:
        subprocess.check_call(
            ['unsquashfs', '-d', os.path.join(temp_dir, 'squashfs-root'),
             snap_path, '-e', os.path.join('meta', 'snap.yaml')])
        with open(os.path.join(
                temp_dir, 'squashfs-root', 'meta', 'snap.yaml')) as yaml_file:
            snap_yaml = yaml.load(yaml_file)

    return snap_yaml['name']
