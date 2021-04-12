# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import pytest
from jsonschema.exceptions import ValidationError

from snapcraft.storeapi.v2 import whoami


@pytest.fixture
def whoami_account_payload():
    return {
        "email": "foo@bar.baz",
        "id": "1234567890",
        "name": "Foo from Bar",
        "username": "foo",
    }


@pytest.fixture
def whoami_payload(whoami_account_payload):
    return {
        "account": whoami_account_payload,
        "channels": None,
        "packages": None,
        "permissions": None,
    }


def test_account(whoami_account_payload):
    w = whoami.Account.unmarshal(whoami_account_payload)

    assert repr(w) == "<Account: 'foo@bar.baz'>"
    assert w.email == whoami_account_payload["email"]
    assert w.account_id == whoami_account_payload["id"]
    assert w.name == whoami_account_payload["name"]
    assert w.username == whoami_account_payload["username"]
    assert w.marshal() == whoami_account_payload


@pytest.mark.parametrize("missing", ("email", "id", "name", "username"))
def test_account_missing_data(missing, whoami_account_payload):
    whoami_account_payload.pop(missing)

    with pytest.raises(ValidationError):
        whoami.Account.unmarshal(whoami_account_payload)


def test_whoami(whoami_payload):
    a = whoami.WhoAmI.unmarshal(whoami_payload)

    assert repr(a) == "<WhoAmI: 'foo@bar.baz'>"
    assert a.account.email == whoami_payload["account"]["email"]
    assert a.account.account_id == whoami_payload["account"]["id"]
    assert a.account.name == whoami_payload["account"]["name"]
    assert a.account.username == whoami_payload["account"]["username"]
    # TODO: implement fully to migrate away from account_info.
    assert a.marshal()["account"] == whoami_payload["account"]
