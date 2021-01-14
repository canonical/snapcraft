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


from unittest import mock
from unittest.mock import call

import launchpadlib
import pytest

from snapcraft.internal.repo import apt_ppa, errors


@pytest.fixture
def mock_launchpad(autouse=True):
    with mock.patch(
        "snapcraft.internal.repo.apt_ppa.Launchpad",
        spec=launchpadlib.launchpad.Launchpad,
    ) as m:
        m.login_anonymously.return_value.load.return_value.signing_key_fingerprint = (
            "FAKE-PPA-SIGNING-KEY"
        )
        yield m


def test_split_ppa_parts():
    owner, name = apt_ppa.split_ppa_parts(ppa="test-owner/test-name")

    assert owner == "test-owner"
    assert name == "test-name"


def test_split_ppa_parts_invalid():
    with pytest.raises(errors.AptPPAInstallError) as exc_info:
        apt_ppa.split_ppa_parts(ppa="ppa-missing-slash")

    assert exc_info.value._ppa == "ppa-missing-slash"


def test_get_launchpad_ppa_key_id(mock_launchpad,):
    key_id = apt_ppa.get_launchpad_ppa_key_id(ppa="ppa-owner/ppa-name")

    assert key_id == "FAKE-PPA-SIGNING-KEY"
    assert mock_launchpad.mock_calls == [
        call.login_anonymously("snapcraft", "production"),
        call.login_anonymously().load("~ppa-owner/+archive/ppa-name"),
    ]
