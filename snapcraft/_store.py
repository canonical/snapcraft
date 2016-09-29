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

from contextlib import contextmanager
import getpass
import json
import logging
import os
import subprocess
import tempfile

from tabulate import tabulate
import yaml

from snapcraft import storeapi
from snapcraft.internal import repo


logger = logging.getLogger(__name__)


def _get_data_from_snap_file(snap_path):
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
    return snap_yaml


def _login(store, acls=None, save=True):
    print('Enter your Ubuntu One SSO credentials.')
    email = input('Email: ')
    password = getpass.getpass('Password: ')

    try:
        try:
            store.login(email, password, acls=acls, save=save)
            print()
            logger.info(
                'We strongly recommend enabling multi-factor authentication: '
                'https://help.ubuntu.com/community/SSO/FAQs/2FA')
        except storeapi.errors.StoreTwoFactorAuthenticationRequired:
            one_time_password = input('Second-factor auth: ')
            store.login(
                email, password, one_time_password=one_time_password,
                acls=acls, save=save)
    except (storeapi.errors.InvalidCredentialsError,
            storeapi.errors.StoreAuthenticationError):
        print()
        logger.info('Login failed.')
        return False
    else:
        print()
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


@contextmanager
def _requires_login():
    try:
        yield
    except storeapi.errors.InvalidCredentialsError:
        logger.error('No valid credentials found.'
                     ' Have you run "snapcraft login"?')
        raise


def _get_usable_keys(name=None):
    keys = json.loads(subprocess.check_output(
        ['snap', 'keys', '--json'], universal_newlines=True))
    if keys is not None:
        for key in keys:
            if name is None or name == key['name']:
                yield key


def _select_key(keys):
    if len(keys) > 1:
        print('Select a key:')
        print()
        tabulated_keys = tabulate(
            [(i + 1, key['name'], key['sha3-384'])
             for i, key in enumerate(keys)],
            headers=["Number", "Name", "SHA3-384 fingerprint"],
            tablefmt="plain")
        print(tabulated_keys)
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


def _export_key(name, account_id):
    return subprocess.check_output(
        ['snap', 'export-key', '--account={}'.format(account_id), name],
        universal_newlines=True)


def list_keys():
    if not repo.is_package_installed('snapd'):
        raise EnvironmentError(
            'The snapd package is not installed. In order to use `list-keys`, '
            'you must run `apt install snapd`.')
    keys = list(_get_usable_keys())
    store = storeapi.StoreClient()
    with _requires_login():
        account_info = store.get_account_information()
    enabled_keys = {
        account_key['public-key-sha3-384']
        for account_key in account_info['account_keys']}
    tabulated_keys = tabulate(
        [('*' if key['sha3-384'] in enabled_keys else '-',
          key['name'], key['sha3-384'],
          '' if key['sha3-384'] in enabled_keys else '(not enabled)')
         for key in keys],
        headers=["", "Name", "SHA3-384 fingerprint", ""],
        tablefmt="plain")
    print(tabulated_keys)


def register_key(name):
    if not repo.is_package_installed('snapd'):
        raise EnvironmentError(
            'The snapd package is not installed. In order to use '
            '`register-key`, you must run `apt install snapd`.')
    keys = list(_get_usable_keys(name=name))
    if not keys:
        if name is not None:
            raise RuntimeError(
                'You have no usable key named "{}".'.format(name))
        else:
            raise RuntimeError('You have no usable keys.')
    key = _select_key(keys)
    store = storeapi.StoreClient()
    if not _login(store, acls=['modify_account_key'], save=False):
        raise RuntimeError('Cannot continue without logging in successfully.')
    logger.info('Registering key ...')
    account_info = store.get_account_information()
    account_key_request = _export_key(key['name'], account_info['account_id'])
    store.register_key(account_key_request)
    logger.info(
        'Done. The key "{}" ({}) may be used to sign your assertions.'.format(
            key['name'], key['sha3-384']))


def register(snap_name, is_private=False):
    logger.info('Registering {}.'.format(snap_name))
    store = storeapi.StoreClient()
    with _requires_login():
        store.register(snap_name, is_private)
    logger.info("Congratulations! You're now the publisher for {!r}.".format(
        snap_name))


def _generate_snap_build(authority_id, snap_id, grade, key_name,
                         snap_filename):
    """Return the signed snap-build declaration for a snap on disk."""
    cmd = [
        'snap', 'sign-build',
        '--developer-id=' + authority_id,
        '--snap-id=' + snap_id,
        '--grade=' + grade
    ]
    if key_name:
        cmd.extend(['-k', key_name])
    cmd.append(snap_filename)
    try:
        return subprocess.check_output(cmd)
    except subprocess.CalledProcessError:
        raise RuntimeError(
            'Failed to sign build assertion for {}.'.format(snap_filename))


def sign_build(snap_filename, key_name='default', local=False):
    if not repo.is_package_installed('snapd'):
        raise EnvironmentError(
            'The snapd package is not installed. In order to use '
            '`sign-build`, you must run `apt install snapd`.')

    if not os.path.exists(snap_filename):
        raise FileNotFoundError(
            'The file {!r} does not exist.'.format(snap_filename))

    snap_series = storeapi.constants.DEFAULT_SERIES
    snap_yaml = _get_data_from_snap_file(snap_filename)
    snap_name = snap_yaml['name']
    grade = snap_yaml.get('grade', 'stable')

    store = storeapi.StoreClient()
    with _requires_login():
        try:
            info = store.get_account_information()
            authority_id = info['account_id']
            snap_id = info['snaps'][snap_series][snap_name]['snap-id']
        except KeyError:
            raise RuntimeError(
                'Your account lacks permission to assert builds for this '
                'snap. Make sure you are logged in as the publisher of '
                '\'{}\' for series \'{}\'.'.format(snap_name, snap_series))

    snap_build_path = snap_filename + '-build'
    if os.path.isfile(snap_build_path):
        with open(snap_build_path, 'rb') as fd:
            snap_build_content = fd.read()
        logger.info(
            'A signed build assertion for this snap already exists.')
    else:
        with open(snap_build_path, 'w+') as fd:
            snap_build_content = _generate_snap_build(
                authority_id, snap_id, grade, key_name, snap_filename)
            fd.write(snap_build_content.decode())
        logger.info(
            'Build assertion {} saved to disk.'.format(snap_build_path))

    if not local:
        store.push_snap_build(snap_id, snap_build_content.decode())
        logger.info(
            'Build assertion {} pushed to the Store.'.format(snap_build_path))


def push(snap_filename, release_channels=None):
    """Push a snap_filename to the store.

    If release_channels is defined it also releases it to those channels if the
    store deems the uploaded snap as ready to release.
    """
    if not os.path.exists(snap_filename):
        raise FileNotFoundError(
            'The file {!r} does not exist.'.format(snap_filename))

    logger.info('Uploading {}.'.format(snap_filename))

    snap_yaml = _get_data_from_snap_file(snap_filename)
    snap_name = snap_yaml['name']
    store = storeapi.StoreClient()
    with _requires_login():
        tracker = store.upload(snap_name, snap_filename)

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


def _format_channel_map(channel_map, arch):
    return [
        (printable_arch,) + _get_text_for_channel(channel)
        for printable_arch, channel in zip(
            [arch] + [''] * len(channel_map), channel_map)]


def _get_text_for_current_channels(channels, current_channels):
    return ', '.join(
        channel + ('*' if channel in current_channels else '')
        for channel in channels) or '-'


def release(snap_name, revision, release_channels):
    store = storeapi.StoreClient()
    with _requires_login():
        channels = store.release(snap_name, revision, release_channels)

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
    store = storeapi.StoreClient()
    try:
        with _requires_login():
            store.download(snap_name, channel, download_path, arch)
    except storeapi.errors.SnapNotFoundError:
        raise RuntimeError(
            'Snap {name} for {arch} cannot be found'
            ' in the {channel} channel'.format(name=snap_name, arch=arch,
                                               channel=channel))
    except storeapi.errors.SHAMismatchError:
        raise RuntimeError(
            'Failed to download {} at {} (mismatched SHA)'.format(
                snap_name, download_path))


def history(snap_name, series, arch):
    store = storeapi.StoreClient()

    with _requires_login():
        history = store.get_snap_history(snap_name, series, arch)

    parsed_revisions = [
        (rev['revision'], rev['timestamp'], rev['arch'], rev['version'],
         _get_text_for_current_channels(
            rev['channels'], rev['current_channels']))
        for rev in history]
    tabulated_revisions = tabulate(
        parsed_revisions,
        headers=['Rev.', 'Uploaded', 'Arch', 'Version', 'Channels'])
    print(tabulated_revisions)


def status(snap_name, series, arch):
    store = storeapi.StoreClient()

    with _requires_login():
        status = store.get_snap_status(snap_name, series, arch)

    parsed_channels = [
        channel
        for arch, channel_map in sorted(status.items())
        for channel in _format_channel_map(channel_map, arch)]

    tabulated_channels = tabulate(
        parsed_channels, headers=['Arch', 'Channel', 'Version', 'Revision'])
    print(tabulated_channels)
