# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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
import base64
import textwrap

import pytest
from pymacaroons import Caveat, Macaroon


@pytest.fixture
def fake_client(mocker):
    """Forces get_client to return a fake craft_store.BaseClient"""
    client = mocker.patch("craft_store.BaseClient", autospec=True)
    mocker.patch("snapcraft.commands.store.client.get_client", return_value=client)
    return client


@pytest.fixture
def fake_confirmation_prompt(mocker):
    """Fake the confirmation prompt."""
    return mocker.patch(
        "snapcraft.utils.confirm_with_user", return_value=False, autospec=True
    )


@pytest.fixture
def root_macaroon():
    return Macaroon(
        location="fake-server.com",
        signature="d9533461d7835e4851c7e3b639144406cf768597dea6e133232fbd2385a5c050",
        caveats=[
            Caveat(
                caveat_id="1234567890",
                location="fake-sso.com",
                verification_key_id="1234567890",
            )
        ],
    ).serialize()


@pytest.fixture
def discharged_macaroon():
    return Macaroon(
        location="fake-server.com",
        signature="d9533461d7835e4851c7e3b639122406cf768597dea6e133232fbd2385a5c050",
    ).serialize()


@pytest.fixture(params=["encode", "no-encode"])
def legacy_config_credentials(request):
    config = textwrap.dedent(
        f"""\
        [login.ubuntu.com]
        macaroon={root_macaroon}
        unbound_discharge={discharged_macaroon}
        """
    )

    if request.param == "encode":
        return base64.b64encode(config.encode()).decode()

    if request.param == "no-encode":
        return config

    raise RuntimeError("unhandled param")


@pytest.fixture
def legacy_config_path(
    monkeypatch, new_dir, root_macaroon, discharged_macaroon, legacy_config_credentials
):
    config_file = new_dir / "snapcraft.cfg"
    monkeypatch.setattr(
        "snapcraft.commands.store._legacy_account.LegacyUbuntuOne._CONFIG_PATH",
        config_file,
    )

    config_file.write_text(legacy_config_credentials)

    return config_file
