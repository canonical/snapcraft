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
    """Private Base class to handle assertions.

    Implementations are supposed to define a class attribute to determine
    the assertion type.

    :cvar str _assertion_type: the assertion type, also treated as the endpoint
                               for the assertion on the store and the payload
                               header key for the returned data.
    """

    @property
    def publisher_id(self):
        """Return the publisher-id of a snap.
        This entry is also known as accound-id or developer-id.

        The value is lazily fetched from the store.
        """
        if not self._account_info:
            self._account_info = self._store_client.get_account_information()
        return self._account_info["account_id"]

    @property
    def snap_id(self):
        """Return the snap-id of the given snap_name.

        The value is lazily fetched from the store.
        """
        if not self._account_info:
            self._account_info = self._store_client.get_account_information()
        snaps = self._account_info["snaps"][self._release]
        try:
            return snaps[self._snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(self._snap_name)

    def __init__(self, *, snap_name, signing_key=None):
        """Create an instance to handle an assertion.

        :param str snap_name: snap name to handle assertion for.
        :param str signing_key: the name of the key to use, if not
                                provided, the default key is used.
        :ivar dict assertion: holds the actual assertion.
        :ivar dict signed_assertion: holds a signed version of assertion.
        """
        self._store_client = StoreClient()
        self._snap_name = snap_name
        self._signing_key = signing_key
        self._release = constants.DEFAULT_SERIES
        self._account_info = None
        self.assertion = None
        self.signed_assertion = None

    def get(self):
        """Get the current assertion from the store.

        :returns: the assertion corresponding to the snap_name.
        :rtype: dict
        """
        # The store adds a header key which is not consistent with the endpoint
        # which we need to pop as it is not understood by snap sign.
        if self.assertion:
            return self.assertion
        store_assertion = self._store_client.get_assertion(
            self.snap_id, self._assertion_type
        )
        self.assertion = list(store_assertion.values())[0]
        return self.assertion

    def sign(self):
        """Create a signed version of the obtained assertion.

        :returns: signed assertion document.
        """
        cmdline = ["snap", "sign"]
        if self._signing_key:
            cmdline += ["-k", self._signing_key]
        snap_sign = subprocess.Popen(
            cmdline,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        data = json.dumps(self.assertion).encode("utf8")
        assertion, err = snap_sign.communicate(input=data)
        if snap_sign.returncode != 0:
            err = err.decode("ascii", errors="replace")
            raise errors.StoreAssertionError(
                snap_name=self._snap_name, endpoint=self._assertion_type, error=err
            )

        self.signed_assertion = assertion
        return assertion

    def push(self, force=False):
        """Push the assertion to the store, signing if necessary.

        :param bool force: if True, ignore any conflict with revoked developers
                           and the snap revisions it would invalidate.
        :returns: None
        """
        if not self.signed_assertion:
            self.sign()
        self._store_client.push_assertion(
            self.snap_id, self.signed_assertion, self._assertion_type, force=force
        )


class DeveloperAssertion(_BaseAssertion):
    """Implementation of a developer assertion.
    The assertion is used to enable collaboration for a given snap_name
    by updating a snap-id's developer assertion using the store endpoint.

    The assertion content has the following structure
    {
        'type': 'snap-developer',
        'authority-id': '<account_id of the publisher or store authority-id>',
        'publisher-id': '<account_id of the publisher>',
        'snap-id': '<snap-id>',
        'developers': [{
            'developer-id': '<account-id>',
            'since': '2017-02-10T08:35:00.390258Z'
           },{
            'developer-id': '<account-id>',
            'since': '2017-02-10T08:35:00.390258Z',
            'until': '2018-02-10T08:35:00.390258Z'
           }],
        }

    """

    _assertion_type = "developers"

    def new_assertion(self, *, developers):
        """Create a new assertion with developers based out of the current one.

        The new assertion has its assertion's authority-id normalized to the
        assertion's publisher_id and the assertion's revision increased by 1.

        :param list developers: a list of a dictionary developers holding the
                                keys: developer_id (mandatory), since
                                (mandatory) and until (optional).
        :returns: a new assertion based out of the current assertion with the
                  provided developers list.
        :rtype: DeveloperAssertion
        """
        new_assertion = deepcopy(self.assertion)
        new_assertion["developers"] = developers

        # The revision should be incremented, to avoid `invalid-revision`
        # errors.
        new_assertion["revision"] = str(int(self.assertion.get("revision", "-1")))

        # There is a possibility that the `authority-id` to be `canonical`,
        # which should be changed to the `publisher_id` to match the signing
        # key.
        new_assertion["authority-id"] = self.publisher_id

        new_instance = DeveloperAssertion(
            snap_name=self._snap_name, signing_key=self._signing_key
        )
        # Reference the already fetched information
        new_instance._account_info = self._account_info
        new_instance.assertion = new_assertion
        return new_instance

    def get(self):
        """Return a dict containing the developer assertion for snap_name.

        The data that comes from the store query looks as follows:
        {'snap_developer': {
             'type': 'snap-developer',
             'authority-id': <account_id of the publisher or
                              store authority-id>,
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

        :returns: the latest developer assertion corresponding to snap_name.
        :rtype: dict
        """
        try:
            self.assertion = super().get()
        except errors.StoreValidationError as e:
            if e.error_list[0]["code"] != "snap-developer-not-found":
                raise
            self.assertion = {
                "type": "snap-developer",
                "authority-id": self.publisher_id,
                "publisher-id": self.publisher_id,
                "snap-id": self.snap_id,
                "developers": [],
            }
        # A safeguard to operate easily on the assertion
        if "developers" not in self.assertion:
            self.assertion["developers"] = []
        return self.assertion

    def get_developers(self):
        """Return a copy of the current developers listed in the assertion.

        :returns: a list of developers.
        :rtype: list
        """
        return self.get()["developers"].copy()

    def is_equal(self, other_assertion):
        """Determine equality of developer lists in each assertion.

        During the comparison, differences in milliseconds are not considered.

        :returns: Return True if the list of developers in this instances
                  assertion list is equal to the list from other_assertion.
        :rtype: bool
        """
        this_devs = self._normalize_time(self.assertion["developers"].copy())
        other_devs = self._normalize_time(
            other_assertion.assertion["developers"].copy()
        )
        return this_devs == other_devs

    def _normalize_time(self, developers):
        for dev in developers:
            for range_ in ["since", "until"]:
                if range_ in dev:
                    date = datetime.strptime(dev[range_], "%Y-%m-%dT%H:%M:%S.%fZ")
                    dev[range_] = date.strftime("%Y-%m-%dT%H:%M:%S.000000Z")
        return developers
