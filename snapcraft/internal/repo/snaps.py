# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import contextlib
import logging
import sys
import urllib
from subprocess import check_call, check_output, CalledProcessError

import requests_unixsocket
from requests import exceptions

from . import errors


logger = logging.getLogger(__name__)
_SNAPD_URI_ROOT_TEMPLATE = 'http+unix://%2Frun%2Fsnapd.socket/v2/{}'


class SnapPackage:
    """SnapPackage acts as a mediator to install or refresh a snap.

    It uses information provided by snapd implicitly referring to the local
    and remote stores to obtain information about the snap, such as its
    confinment value and channel availability.

    This information can also be used to determine if a snap should be
    installed or refreshed.

    There are risks of the data falling out of date between the query and the
    requested action given that it is not possible to hold a global lock on
    snapd and the store data can change in between validation and execution.
    """

    @classmethod
    def is_valid_snap(cls, snap):
        return cls(snap).is_valid()

    @classmethod
    def is_snap_installed(cls, snap):
        return cls(snap).installed

    def __init__(self, snap):
        """Lifecycle handler for a snap of the format <snap-name>/<channel>."""
        self.name, self.channel = _get_parsed_snap(snap)
        self._original_channel = self.channel
        if not self.channel or self.channel == 'stable':
            self.channel = 'latest/stable'

        # This store information from a local request
        self._local_snap_info = None
        # And this stores information from a remote request
        self._store_snap_info = None

        self._is_installed = None
        self._is_in_store = None

    @property
    def installed(self):
        if self._is_installed is None:
            self._is_installed = self.get_local_snap_info() is not None
        return self._is_installed

    @property
    def in_store(self):
        if self._is_in_store is None:
            self._is_in_store = self.get_store_snap_info() is not None
        return self._is_in_store

    def get_local_snap_info(self):
        """Returns a local payload for the snap.

        Validity of the results are determined by checking self.installed."""
        if self._is_installed is None:
            with contextlib.suppress(exceptions.HTTPError):
                self._local_snap_info = _get_local_snap_info(self.name)
        return self._local_snap_info

    def get_store_snap_info(self):
        """Returns a store payload for the snap.

        Validity of the results are determined by checking self.remote."""
        if self._is_in_store is None:
            with contextlib.suppress(exceptions.HTTPError):
                self._store_snap_info = _get_store_snap_info(self.name)
        return self._store_snap_info

    def _get_store_channels(self):
        snap_store_info = self.get_store_snap_info()
        if not self.in_store:
            return dict()

        return snap_store_info['channels']

    def get_current_channel(self):
        current_channel = ''
        if self.installed:
            local_snap_info = self.get_local_snap_info()
            current_channel = local_snap_info['channel']
            if current_channel == 'stable':
                current_channel = 'latest/stable'
        return current_channel

    def is_classic(self):
        store_channels = self._get_store_channels()
        return store_channels[self.channel]['confinement'] == 'classic'

    def is_valid(self):
        """Check if the snap is valid."""
        if not self.in_store:
            return False
        store_channels = self._get_store_channels()
        return self.channel in store_channels.keys()

    def install(self):
        """Installs the snap onto the system."""
        snap_install_cmd = []
        if _snap_command_requires_sudo():
            snap_install_cmd = ['sudo']
        snap_install_cmd.extend(['snap', 'install', self.name])
        if self._original_channel:
            snap_install_cmd.extend(['--channel', self._original_channel])
        if self.is_classic:
            # TODO make this a user explicit choice
            snap_install_cmd.extend(['--classic'])
        try:
            check_call(snap_install_cmd)
        except CalledProcessError as install_error:
            raise errors.SnapInstallError(snap_name=self.name,
                                          snap_channel=self.channel)

    def refresh(self):
        """Refreshes a snap onto a channel on the system."""
        snap_refresh_cmd = []
        if _snap_command_requires_sudo():
            snap_refresh_cmd = ['sudo']
        snap_refresh_cmd.extend(['snap', 'refresh', self.name,
                                 '--channel', self.channel])
        if self.is_classic:
            # TODO make this a user explicit choice
            snap_refresh_cmd.extend(['--classic'])
        try:
            check_call(snap_refresh_cmd)
        except CalledProcessError as install_error:
            raise errors.SnapRefreshError(snap_name=self.name,
                                          snap_channel=self.channel)


def install_snaps(snaps_list):
    """Install snaps of the format <snap-name>/<channel>."""
    for snap in snaps_list:
        snap_pkg = SnapPackage(snap)
        if not snap_pkg.installed and snap_pkg.in_store:
            snap_pkg.install()
        elif snap_pkg.get_current_channel() != snap_pkg.channel:
            snap_pkg.refresh()


def _snap_command_requires_sudo():
    # snap whoami returns - if the user is not logged in.
    output = check_output(['snap', 'whoami'])
    whoami = output.decode(sys.getfilesystemencoding())
    requires_root = False
    try:
        requires_root = whoami.split(':')[1].strip() == '-'
    # A safeguard if the output changes
    except IndexError:
        requires_root = True
    if requires_root:
        logger.warning('snapd is not logged in, snap install '
                       'commands will use sudo')
    return requires_root


def _get_parsed_snap(snap):
    if '/' in snap:
        snap_name = snap[:snap.find('/')]
        snap_channel = snap[snap.find('/')+1:]
    else:
        snap_name = snap
        snap_channel = ''
    return snap_name, snap_channel


def _get_local_snap_info(snap_name):
    slug = 'snaps/{}'.format(urllib.parse.quote(snap_name, safe=''))
    url = _SNAPD_URI_ROOT_TEMPLATE.format(slug)
    with requests_unixsocket.Session() as session:
        snap_info = session.get(url)
    snap_info.raise_for_status()
    return snap_info.json()['result']


def _get_store_snap_info(snap_name):
    # This logic uses /v2/find returns an array of results, given that
    # we do a strict search either 1 result or a 404 will be returned.
    slug = '{}{}'.format('find?name=', urllib.parse.quote(snap_name, safe=''))
    url = _SNAPD_URI_ROOT_TEMPLATE.format(slug)
    with requests_unixsocket.Session() as session:
        snap_info = session.get(url)
    snap_info.raise_for_status()
    return snap_info.json()['result'][0]
