# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2021 Canonical Ltd
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

from textwrap import dedent

import pytest

from snapcraft.storeapi.v2 import whoami
from snapcraft.storeapi import StoreClient


@pytest.fixture
def fake_dashboard_whoami(monkeypatch):
    monkeypatch.setattr(
        StoreClient,
        "whoami",
        lambda x: whoami.WhoAmI(
            account=whoami.Account(
                email="foo@bar.baz",
                account_id="1234567890",
                name="Foo from Baz",
                username="foo",
            )
        ),
    )


@pytest.mark.usefixtures("fake_dashboard_whoami")
def test_whoami(click_run):
    result = click_run(["whoami"])

    assert result.exit_code == 0
    assert result.output == dedent(
        """\
        email:        foo@bar.baz
        developer-id: 1234567890
        """
    )
