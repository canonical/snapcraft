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
import io
import logging
import subprocess
import tempfile
import os

import gpgme
from tabulate import tabulate
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


def _login(store, acls=None, save=True):
    print('Enter your Ubuntu One SSO credentials.')
    email = input('Email: ')
    password = getpass.getpass('Password: ')
    one_time_password = input(
        'One-time password (just press enter if you don\'t use two-factor '
        'authentication): ')

    logger.info('Authenticating against Ubuntu One SSO.')
    try:
        store.login(
            email, password, one_time_password=one_time_password, acls=acls,
            save=save)
    except (storeapi.errors.InvalidCredentialsError,
            storeapi.errors.StoreAuthenticationError):
        logger.info('Login failed.')
        return False
    else:
        logger.info('Login successful.')
        return True


def login():
    store = storeapi.StoreClient()
    return _login(store)


def logout():
    logger.info('Clearing credentials for Ubuntu One SSO.')
    store = storeapi.StoreClient()
    store.logout()
    logger.info('Credentials cleared.')


def _is_usable_subkey(subkey):
    """Can we use `subkey`?

    A subkey is usable if it is not expired/revoked/disabled/invalid, if it
    is RSA, and if it is at least 4096 bits.
    """
    if subkey.expired or subkey.revoked or subkey.disabled or subkey.invalid:
        return False
    # We're currently only interested in RSA >= 4096-bit.
    if subkey.pubkey_algo != gpgme.PK_RSA or subkey.length < 4096:
        return False
    return True


def _is_usable_key(key):
    """Can we use `key`?

    A key is usable if its main subkey (the first) is usable, if it has
    UIDs, if it is not disabled, and if it has signing capabilities.
    """
    if not key.subkeys or not key.uids or key.disabled or not key.can_sign:
        return False
    return _is_usable_subkey(key.subkeys[0])


def _get_usable_secret_keys(query=None):
    context = gpgme.Context()
    for key in context.keylist(query, True):
        if _is_usable_key(key):
            yield key


def _format_fingerprint(fingerprint):
    if len(fingerprint) != 40:
        # Should never happen; GPG fingerprints are SHA-1 with length 40.
        raise RuntimeError(
            "GPG fingerprint has unexpected length %d" % len(fingerprint))
    chunks = [fingerprint[i:i + 4] for i in range(0, 40, 4)]
    return '{}  {}'.format(' '.join(chunks[:5]), ' '.join(chunks[5:]))


_format_key_algorithm = {
    gpgme.PK_RSA: 'R',
}


def _format_key(key):
    mainkey = key.subkeys[0]
    return '%d%s: %s' % (
        mainkey.length, _format_key_algorithm[mainkey.pubkey_algo],
        _format_fingerprint(mainkey.fpr))


def _select_key(keys):
    if len(keys) > 1:
        print('Select a key:')
        print()
        width = len(str(len(keys)))
        for i, key in enumerate(keys):
            print('{index:{width}d}. {key}'.format(
                index=i + 1, key=_format_key(key), width=width))
            print('{blank:{width}}  {uid}'.format(
                blank='', uid=key.uids[0].uid, width=width))
        print()
        while True:
            try:
                keynum = int(input('Key number: ')) - 1
            except ValueError:
                continue
            if keynum >= 0 and keynum < len(keys):
                return keys[keynum]
    else:
        return keys[0]


def register_key(query):
    keys = list(_get_usable_secret_keys(query=query))
    if not keys:
        raise RuntimeError(
            'You have no usable GPG secret keys matching "{}".'.format(query))
    key = _select_key(keys)
    fingerprint = key.subkeys[0].fpr
    context = gpgme.Context()
    key_data = io.BytesIO()
    context.export(fingerprint.encode('ascii'), key_data)
    store = storeapi.StoreClient()
    if not _login(store, acls=['modify_account_key'], save=False):
        raise RuntimeError("Cannot continue without logging in successfully.")
    logger.info('Registering GPG key ...')
    store.register_key(key_data.getvalue())
    logger.info(
        'Done. The GPG key {} will be expected for signing your '
        'assertions.'.format(_format_fingerprint(fingerprint)))


def register(snap_name, is_private=False):
    logger.info('Registering {}.'.format(snap_name))
    store = storeapi.StoreClient()
    try:
        store.register(snap_name, is_private)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise
    logger.info("Congratulations! You're now the publisher for {!r}.".format(
        snap_name))


def push(snap_filename, release_channels=None):
    """Push a snap_filename to the store.

    If release_channels is defined it also releases it to those channels if the
    store deems the uploaded snap as ready to release.
    """
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
    # This is workaround until LP: #1599875 is solved
    if 'revision' in result:
        logger.info('Revision {!r} of {!r} created.'.format(
            result['revision'], snap_name))
    else:
        logger.info('Uploaded {!r}'.format(snap_name))
    tracker.raise_for_code()

    if release_channels:
        release(snap_name, result['revision'], release_channels)


def _get_text_for_opened_channels(opened_channels):
    if len(opened_channels) == 1:
        return 'The {!r} channel is now open.'.format(opened_channels[0])
    else:
        channels = ('{!r}'.format(channel) for channel in opened_channels[:-1])
        return 'The {} and {!r} channels are now open.'.format(
            ', '.join(channels), opened_channels[-1])


def _get_text_for_channel(channel):
    if channel['info'] == 'none':
        channel_text = (channel['channel'], '-', '-')
    elif channel['info'] == 'tracking':
        channel_text = (channel['channel'], '^', '^')
    elif channel['info'] == 'specific':
        channel_text = (
            channel['channel'],
            channel['version'],
            channel['revision'],
        )
    else:
        raise RuntimeError('Unexpected channel info {!r}.'.format(
            channel['info']))

    return channel_text


def release(snap_name, revision, release_channels):
    try:
        store = storeapi.StoreClient()
        channels = store.release(snap_name, revision, release_channels)
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise

    if 'opened_channels' in channels:
        logger.info(
            _get_text_for_opened_channels(channels['opened_channels']))
        # There should be an empty line between the open channels
        # message and what follows
        print()
    channel_map = channels['channel_map']
    parsed_channels = [_get_text_for_channel(c) for c in channel_map]
    tabulated_channels = tabulate(parsed_channels,
                                  headers=['Channel', 'Version', 'Revision'])
    # This does not look good in green so we print instead
    print(tabulated_channels)


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
