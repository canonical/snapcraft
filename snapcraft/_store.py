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

from snapcraft import storeapi


logger = logging.getLogger(__name__)


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
        response = store.register(snap_name)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise
    if response.ok:
        logger.info(
            "Congratulations! You're now the publisher for {!r}.".format(
                snap_name))
    else:
        raise RuntimeError('Registration failed.')


def upload(snap_filename):
    if not os.path.exists(snap_filename):
        raise FileNotFoundError(
            'The file {!r} does not exist.'.format(snap_filename))

    logger.info('Uploading {}.'.format(snap_filename))

    try:
        store = storeapi.StoreClient()
        result = store.upload(snap_filename)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise

    success = result.get('success', False)
    errors = result.get('errors', [])
    app_url = result.get('application_url', '')
    revision = result.get('revision')

    # Print another newline to make sure the user sees the final result of the
    # upload (success/failure).
    print()

    if success:
        message = 'Application uploaded successfully'
        if revision:
            message = '{} (as revision {})'.format(message, revision)
        logger.info(message)
    else:
        logger.info('Upload did not complete.')

    if errors:
        logger.info('Some errors were detected:\n\n{}\n'.format(
            '\n'.join(str(error) for error in errors)))

    if app_url:
        logger.info('Please check out the application at: {}\n'.format(
                    app_url))

    return success


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
