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
import json
import subprocess
from copy import deepcopy
from datetime import datetime

from . import StoreClient
from . import errors
from . import constants


class _BaseAssertion:

    @property
    def publisher_id(self):
        if not self._account_info:
            self._account_info = self._store_client.get_account_information()
        return self._account_info['account_id']

    @property
    def snap_id(self):
        if not self._account_info:
            self._account_info = self._store_client.get_account_information()
        snaps = self._account_info['snaps'][self._release]
        try:
            return snaps[self._snap_name]['snap-id']
        except KeyError:
            raise errors.SnapNotFoundError(self._snap_name)

    def __init__(self, *, snap_name, signing_key):
        self._store_client = StoreClient()
        self._snap_name = snap_name
        self._signing_key = signing_key
        self._release = constants.DEFAULT_SERIES
        self._account_info = None
        self.assertion = None
        self.signed_assertion = None

    def get(self):
        # The store adds a header key which is not consistent with the endpoint
        # which we need to pop as it is not understood by snap sign.
        if self.assertion:
            return self.assertion
        store_assertion = self._store_client.get_assertion(
            self.snap_id, self._assertion_type)
        self.assertion = list(store_assertion.values())[0]
        return self.assertion

    def sign(self):
        cmdline = ['snap', 'sign']
        if self._signing_key:
            cmdline += ['-k', self._signing_key]
        snap_sign = subprocess.Popen(
            cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)
        data = json.dumps(self.assertion).encode('utf8')
        assertion, err = snap_sign.communicate(input=data)
        if snap_sign.returncode != 0:
            err = err.decode('ascii', errors='replace')
            raise errors.StoreAssertionError(
                snap_name=self._snap_name,
                endpoint=self._assertion_type, error=err)

        self.signed_assertion = assertion
        return assertion

    def push(self, force=False):
        """Push an assertion to the store, signing if necessary."""
        if not self.signed_assertion:
            self.sign()
        self._store_client.push_assertion(
            self.snap_id, self.signed_assertion, self._assertion_type,
            force=force)


class DeveloperAssertion(_BaseAssertion):

    _assertion_type = 'developers'

    def new_assertion(self, *, developers):
        """Create a new assertion with developers based out of the current one.
        """
        new_assertion = deepcopy(self.assertion)
        new_assertion['developers'] = developers

        # The revision should be incremented, to avoid `invalid-revision`
        # errors.
        new_assertion['revision'] = str(int(
            self.assertion.get('revision', '0'))+1)

        # There is a possibility that the `authority-id` to be `canonical`,
        # which should be changed to the `publisher_id` to match the signing
        # key.
        new_assertion['authority-id'] = self.publisher_id

        new_instance = DeveloperAssertion(snap_name=self._snap_name,
                                          signing_key=self._signing_key)
        # Reference the already fetched information
        new_instance._account_info = self._account_info
        new_instance.assertion = new_assertion
        return new_instance

    def get(self):
        """Returns a dict containing the developer assertion for snap_name.

        If time_format is 'human' the time will be reformatted into a human
        friendly format.

        The data that comes from the store query looks as follows:
        {'snap_developer': {
             'type': 'snap-developer',
             'authority-id': <account_id of the publisher>,
             'publisher-id': <account_id of the publisher>,
             'snap-id': 'snap_id',
             'developers': [{
                 'developer-id': 'account_id of dev-1',
                 'since': '2017-02-10T08:35:00.390258Z'
                },{
                 'developer-id': 'account_id of dev-2',
                 'since': '2017-02-10T08:35:00.390258Z',
                 'until': '2018-02-10T08:35:00.390258Z'
                }],
             }
        }

        The assertion is saved without the snap_developer header.
        """
        try:
            self.assertion = super().get()
        except errors.StoreValidationError as e:
            if e.error_list[0]['code'] != 'snap-developer-not-found':
                raise
            self.assertion = {
                'type': 'snap-developer',
                'authority-id': self.publisher_id,
                'publisher-id': self.publisher_id,
                'snap-id': self.snap_id,
                'developers': [],
            }
        # A safeguard to operate easily on the assertion
        if 'developers' not in self.assertion:
            self.assertion['developers'] = []
        return self.assertion

    def get_developers(self):
        """Return a copy of the current developers listed in the assertion."""
        return self.get()['developers'].copy()

    def is_equal(self, other_assertion):
        # We need to compare without milliseconds, so drop them from the
        # assertion.
        this_devs = self._normalize_time(
            self.assertion['developers'].copy())
        other_devs = self._normalize_time(
            other_assertion.assertion['developers'].copy())
        return this_devs == other_devs

    def _normalize_time(self, developers):
        for dev in developers:
            for range_ in ['since', 'until']:
                if range_ in dev:
                    date = datetime.strptime(dev[range_],
                                             '%Y-%m-%dT%H:%M:%S.%fZ')
                    dev[range_] = date.strftime('%Y-%m-%dT%H:%M:%S.000000Z')
        return developers
