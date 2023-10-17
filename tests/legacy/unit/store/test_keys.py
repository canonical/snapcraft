# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd
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
from unittest import mock

import pytest

from snapcraft_legacy import _store

NO_KEYS_OUTPUT = """\
No keys have been created on this system. See 'snapcraft create-key --help' to create a key.
No keys have been registered with this account. See 'snapcraft register-key --help' to register a key.
"""
ONLY_CREATED_KEYS_OUTPUT = """\
The following keys are available on this system:
    Name    SHA3-384 fingerprint
-   ctrl    ModelM                  (not registered)
-   alt     Dvorak                  (not registered)
No keys have been registered with this account. See 'snapcraft register-key --help' to register a key.
"""
UNAVAILABLE_KEYS_OUTPUT = """\
No keys have been created on this system. See 'snapcraft create-key --help' to create a key.
The following SHA3-384 key fingerprints have been registered but are not available on this system:
- Bach
- ModelM
"""
SOME_KEYS_OVERLAP_OUTPUT = """\
The following keys are available on this system:
    Name    SHA3-384 fingerprint
*   ctrl    ModelM
-   alt     Dvorak                  (not registered)
The following SHA3-384 key fingerprints have been registered but are not available on this system:
- Bach
"""
LOCAL_KEYS = [
    {"name": "ctrl", "sha3-384": "ModelM"},
    {"name": "alt", "sha3-384": "Dvorak"},
]
REMOTE_KEYS = [
    {"public-key-sha3-384": "Bach"},
    {"public-key-sha3-384": "ModelM"},
]
NO_KEYS = []


@pytest.mark.parametrize(
    ("usable_keys", "account_keys", "stdout"),
    [
        (NO_KEYS, NO_KEYS, NO_KEYS_OUTPUT),
        (LOCAL_KEYS, NO_KEYS, ONLY_CREATED_KEYS_OUTPUT),
        (NO_KEYS, REMOTE_KEYS, UNAVAILABLE_KEYS_OUTPUT),
        (LOCAL_KEYS, REMOTE_KEYS, SOME_KEYS_OVERLAP_OUTPUT),
    ],
)
def test_list_keys(monkeypatch, capsys, usable_keys, account_keys, stdout):
    monkeypatch.setattr(_store, "_get_usable_keys", lambda: usable_keys)
    mock_client = mock.Mock(spec_set=_store.StoreClientCLI)
    mock_client.get_account_information.return_value = {"account_keys": account_keys}
    monkeypatch.setattr(_store, "StoreClientCLI", lambda: mock_client)

    _store.list_keys()

    out, err = capsys.readouterr()

    assert out == stdout
    assert err == ""
