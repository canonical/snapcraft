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
import datetime
import getpass
import json
import logging
import os
import re
import subprocess
import tempfile

from subprocess import Popen

from tabulate import tabulate
import yaml

from snapcraft import storeapi
from snapcraft.internal import (
    cache,
    repo,
)


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
          '' if key['sha3-384'] in enabled_keys else '(not registered)')
         for key in keys],
        headers=["", "Name", "SHA3-384 fingerprint", ""],
        tablefmt="plain")
    print(tabulated_keys)


def create_key(name):
    if not repo.is_package_installed('snapd'):
        raise EnvironmentError(
            'The snapd package is not installed. In order to use '
            '`create-key`, you must run `apt install snapd`.')
    if not name:
        name = 'default'
    keys = list(_get_usable_keys(name=name))
    if keys:
        # `snap create-key` would eventually fail, but we can save the user
        # some time in this obvious error case by not bothering to talk to
        # the store first.
        raise RuntimeError('You already have a key named "{}".'.format(name))
    store = storeapi.StoreClient()
    try:
        account_info = store.get_account_information()
        enabled_names = {
            account_key['name']
            for account_key in account_info['account_keys']}
    except storeapi.errors.InvalidCredentialsError:
        # Don't require a login here; if they don't have valid credentials,
        # then they probably also don't have a key registered with the store
        # yet.
        enabled_names = set()
    if name in enabled_names:
        raise RuntimeError(
            'You have already registered a key named "{}".'.format(name))
    subprocess.check_call(['snap', 'create-key', name])


def _maybe_prompt_for_key(name):
    keys = list(_get_usable_keys(name=name))
    if not keys:
        if name is not None:
            raise RuntimeError(
                'You have no usable key named "{}".\nSee the keys available '
                'in your system with `snapcraft keys`.'.format(name))
        else:
            raise RuntimeError(
                'You have no usable keys.\nPlease create at least one key '
                'with `snapcraft create-key` for use with snap.')
    return _select_key(keys)


def register_key(name):
    if not repo.is_package_installed('snapd'):
        raise EnvironmentError(
            'The snapd package is not installed. In order to use '
            '`register-key`, you must run `apt install snapd`.')
    key = _maybe_prompt_for_key(name)
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


def sign_build(snap_filename, key_name=None, local=False):
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
        account_info = store.get_account_information()

    try:
        authority_id = account_info['account_id']
        snap_id = account_info['snaps'][snap_series][snap_name]['snap-id']
    except KeyError:
        raise RuntimeError(
            'Your account lacks permission to assert builds for this '
            'snap. Make sure you are logged in as the publisher of '
            '\'{}\' for series \'{}\'.'.format(snap_name, snap_series))

    snap_build_path = snap_filename + '-build'
    if os.path.isfile(snap_build_path):
        logger.info(
            'A signed build assertion for this snap already exists.')
        with open(snap_build_path, 'rb') as fd:
            snap_build_content = fd.read()
    else:
        key = _maybe_prompt_for_key(key_name)
        if not local:
            is_registered = [
                a for a in account_info['account_keys']
                if a['public-key-sha3-384'] == key['sha3-384']
            ]
            if not is_registered:
                raise RuntimeError(
                    'The key {!r} is not registered in the Store.\n'
                    'Please register it with `snapcraft register-key {!r}` '
                    'before signing and pushing signatures to the '
                    'Store.'.format(key['name'], key['name']))
        snap_build_content = _generate_snap_build(
            authority_id, snap_id, grade, key['name'], snap_filename)
        with open(snap_build_path, 'w+') as fd:
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

    if os.environ.get('DELTA_UPLOADS_EXPERIMENTAL') and 'revision' in result:
        snap_cache = cache.SnapCache()
        snap_cache.cache(snap_filename, result['revision'])

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
    tabulated_channels = tabulate(
        parsed_channels, numalign='left',
        headers=['Channel', 'Version', 'Revision'],
        tablefmt='plain')
    # This does not look good in green so we print instead
    print(tabulated_channels)


def _tabulated_status(status):
    """Tabulate status (architecture-specific channel-maps)."""
    def _format_channel_map(channel_map, arch):
        return [
            (printable_arch,) + _get_text_for_channel(channel)
            for printable_arch, channel in zip(
                    [arch] + [''] * len(channel_map), channel_map)]

    parsed_channels = [
        channel
        for arch, channel_map in sorted(status.items())
        for channel in _format_channel_map(channel_map, arch)]
    return tabulate(
        parsed_channels, numalign='left',
        headers=['Arch', 'Channel', 'Version', 'Revision'],
        tablefmt='plain')


def close(snap_name, channel_names):
    """Close one or more channels for the specific snap."""
    snap_series = storeapi.constants.DEFAULT_SERIES

    store = storeapi.StoreClient()

    with _requires_login():
        info = store.get_account_information()

    try:
        snap_id = info['snaps'][snap_series][snap_name]['snap-id']
    except KeyError:
        raise RuntimeError(
            'Your account lacks permission to close channels for this snap. '
            'Make sure the logged in account has upload permissions on '
            '\'{}\' in series \'{}\'.'.format(snap_name, snap_series))

    closed_channels, status = store.close_channels(snap_id, channel_names)

    tabulated_status = _tabulated_status(status)
    print(tabulated_status)

    print()
    if len(closed_channels) == 1:
        msg = 'The {} channel is now closed.'.format(closed_channels[0])
    else:
        msg = 'The {} and {} channels are now closed.'.format(
            ', '.join(closed_channels[:-1]), closed_channels[-1])
    logger.info(msg)


def download(snap_name, channel, download_path, arch):
    """Download snap from the store to download_path"""
    store = storeapi.StoreClient()
    try:
        with _requires_login():
            store.download(snap_name, channel, download_path, arch)
    except storeapi.errors.SHAMismatchError:
        raise RuntimeError(
            'Failed to download {} at {} (mismatched SHA)'.format(
                snap_name, download_path))


def status(snap_name, series, arch):
    store = storeapi.StoreClient()

    with _requires_login():
        status = store.get_snap_status(snap_name, series, arch)

    tabulated_status = _tabulated_status(status)
    print(tabulated_status)


def _get_text_for_current_channels(channels, current_channels):
    return ', '.join(
        channel + ('*' if channel in current_channels else '')
        for channel in channels) or '-'


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
        parsed_revisions, numalign='left',
        headers=['Rev.', 'Uploaded', 'Arch', 'Version', 'Channels'],
        tablefmt='plain')
    print(tabulated_revisions)


def gated(snap_name):
    """Print list of snaps gated by snap_name."""
    store = storeapi.StoreClient()
    # Get data for the gating snap
    with _requires_login():
        snaps = store.get_account_information().get('snaps', {})

    release = storeapi.constants.DEFAULT_SERIES
    # Resolve name to snap-id
    try:
        snap_id = snaps[release][snap_name]['snap-id']
    except KeyError:
        raise storeapi.errors.SnapNotFoundError(snap_name)

    validations = store.get_validations(snap_id)

    if validations:
        table_data = []
        for v in validations:
            name = v['approved-snap-name']
            revision = v['approved-snap-revision']
            if revision == '-':
                revision = None
            required = str(v.get('required', True))
            # Currently timestamps have microseconds, which look bad
            timestamp = v['timestamp']
            if '.' in timestamp:
                timestamp = timestamp.split('.')[0] + 'Z'
            table_data.append([name, revision, required, timestamp])
        tabulated = tabulate(
            table_data, headers=['Name', 'Revision', 'Required', 'Approved'],
            tablefmt="plain", missingval='-')
        print(tabulated)
    else:
        print('There are no validations for snap {!r}'.format(snap_name))


def validate(snap_name, validations, revoke=False, key=None):
    """Generate, sign and upload validation assertions."""

    # Check validations format
    _check_validations(validations)

    store = storeapi.StoreClient()

    # Need the ID of the logged in user.
    with _requires_login():
        account_info = store.get_account_information()
    authority_id = account_info['account_id']

    # Get data for the gating snap
    release = storeapi.constants.DEFAULT_SERIES
    try:
        snap_id = account_info['snaps'][release][snap_name]['snap-id']
    except KeyError:
        raise storeapi.errors.SnapNotFoundError(snap_name)

    # Then, for each requested validation, generate assertion
    for validation in validations:
        gated_name, rev = validation.split('=', 1)
        approved_data = store.cpi.get_package(gated_name, 'stable')
        assertion = {
            'type': 'validation',
            'authority-id': authority_id,
            'series': release,
            'snap-id': snap_id,
            'approved-snap-id': approved_data['snap_id'],
            'approved-snap-revision': rev,
            'timestamp': datetime.datetime.utcnow().isoformat() + 'Z',
            'revoked': "false"
        }
        if revoke:
            assertion['revoked'] = "true"

        assertion = _sign_validation(validation, assertion, key)

        # Save assertion to a properly named file
        fname = '{}-{}-r{}.assertion'.format(snap_name, gated_name, rev)
        with open(fname, 'wb') as f:
            f.write(assertion)

        store.push_validation(snap_id, assertion)


validation_re = re.compile('^[^=]+=[0-9]+$')


def _check_validations(validations):
    invalids = [v for v in validations if not validation_re.match(v)]
    if invalids:
        for v in invalids:
            logger.error('Invalid validation request "{}", format must be'
                         ' name=revision'.format(v))
        raise RuntimeError()


def _sign_validation(validation, assertion, key):
    cmdline = ['snap', 'sign']
    if key:
        cmdline += ['-k', key]
    snap_sign = Popen(
        cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    data = json.dumps(assertion).encode('utf8')
    logger.info('Signing validation {}'.format(validation))
    assertion, err = snap_sign.communicate(input=data)
    if snap_sign.returncode != 0:
        err = err.decode('ascii', errors='replace')
        raise RuntimeError('Error signing assertion: {!s}'.format(err))
    return assertion
