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

import argparse
from textwrap import dedent
from unittest.mock import ANY, call

import pytest

from snapcraft import commands

############
# Fixtures #
############


@pytest.fixture
def fake_store_register(mocker):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.register",
        autospec=True,
    )
    return fake_client


@pytest.fixture
def fake_store_get_account_info(mocker):
    # reduced payload
    data = {
        "snaps": {
            "16": {
                "test-snap-public": {
                    "private": False,
                    "since": "2016-07-26T20:18:32Z",
                    "status": "Approved",
                },
                "test-snap-private": {
                    "private": True,
                    "since": "2016-07-26T20:18:32Z",
                    "status": "Approved",
                },
                "test-snap-not-approved": {
                    "private": False,
                    "since": "2016-07-26T20:18:32Z",
                    "status": "Dispute",
                },
            }
        }
    }
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.get_account_info",
        autospec=True,
        return_value=data,
    )
    return fake_client


####################
# Register Command #
####################


@pytest.mark.usefixtures("memory_keyring")
def test_register_default(emitter, fake_confirmation_prompt, fake_store_register):
    fake_confirmation_prompt.return_value = True

    cmd = commands.StoreRegisterCommand(None)

    cmd.run(
        argparse.Namespace(
            store_id=None, private=False, yes=False, **{"snap-name": "test-snap"}
        )
    )

    assert fake_store_register.mock_calls == [
        call(ANY, "test-snap", is_private=False, store_id=None)
    ]
    emitter.assert_message("Registered 'test-snap'")
    assert fake_confirmation_prompt.mock_calls == [
        call(
            dedent(
                """\
        We always want to ensure that users get the software they expect
        for a particular name.

        If needed, we will rename snaps to ensure that a particular name
        reflects the software most widely expected by our community.

        For example, most people would expect 'thunderbird' to be published by
        Mozilla. They would also expect to be able to get other snaps of
        Thunderbird as '$username-thunderbird'.

        Would you say that MOST users will expect 'test-snap' to come from
        you, and be the software you intend to publish there?"""
            )
        )
    ]


@pytest.mark.usefixtures("memory_keyring")
def test_register_yes(emitter, fake_store_register):
    cmd = commands.StoreRegisterCommand(None)

    cmd.run(
        argparse.Namespace(
            store_id=None, private=False, yes=True, **{"snap-name": "test-snap"}
        )
    )

    assert fake_store_register.mock_calls == [
        call(ANY, "test-snap", is_private=False, store_id=None)
    ]
    emitter.assert_message("Registered 'test-snap'")


@pytest.mark.usefixtures("memory_keyring")
def test_register_no(emitter, fake_confirmation_prompt, fake_store_register):
    cmd = commands.StoreRegisterCommand(None)

    cmd.run(
        argparse.Namespace(
            store_id=None, private=False, yes=False, **{"snap-name": "test-snap"}
        )
    )

    assert fake_store_register.mock_calls == []
    emitter.assert_messages(["Snap name 'test-snap' not registered"])
    assert fake_confirmation_prompt.mock_calls == [
        call(
            dedent(
                """\
        We always want to ensure that users get the software they expect
        for a particular name.

        If needed, we will rename snaps to ensure that a particular name
        reflects the software most widely expected by our community.

        For example, most people would expect 'thunderbird' to be published by
        Mozilla. They would also expect to be able to get other snaps of
        Thunderbird as '$username-thunderbird'.

        Would you say that MOST users will expect 'test-snap' to come from
        you, and be the software you intend to publish there?"""
            )
        )
    ]


@pytest.mark.usefixtures("memory_keyring", "fake_confirmation_prompt")
def test_register_private(emitter, fake_store_register):
    cmd = commands.StoreRegisterCommand(None)

    cmd.run(
        argparse.Namespace(
            store_id=None, private=True, yes=False, **{"snap-name": "test-snap"}
        )
    )

    assert fake_store_register.mock_calls == []
    emitter.assert_progress(
        dedent(
            """\
            Even though this is private snap, you should think carefully about
            the choice of name and make sure you are confident nobody else will
            have a stronger claim to that particular name. If you are unsure
            then we suggest you prefix the name with your developer identity,
            As '$username-yoyodyne-www-site-content'."""
        ),
        permanent=True,
    )
    emitter.assert_message(
        "Snap name 'test-snap' not registered",
    )


@pytest.mark.usefixtures("memory_keyring")
def test_register_store_id(emitter, fake_store_register):
    cmd = commands.StoreRegisterCommand(None)

    cmd.run(
        argparse.Namespace(
            store_id="1234", private=False, yes=True, **{"snap-name": "test-snap"}
        )
    )

    assert fake_store_register.mock_calls == [
        call(ANY, "test-snap", is_private=False, store_id="1234")
    ]
    emitter.assert_message("Registered 'test-snap'")


#################
# Names Command #
#################


@pytest.mark.parametrize(
    "command_class",
    [
        commands.StoreNamesCommand,
        commands.StoreLegacyListCommand,
        commands.StoreLegacyListRegisteredCommand,
    ],
)
@pytest.mark.usefixtures("memory_keyring")
def test_names(emitter, fake_store_get_account_info, command_class):
    cmd = command_class(None)

    cmd.run(
        argparse.Namespace(
            store_id="1234", private=False, yes=True, **{"snap-name": "test-snap"}
        )
    )

    assert fake_store_get_account_info.mock_calls == [call(ANY)]
    if command_class.hidden:
        emitter.assert_progress("This command is deprecated: use 'names' instead")
    emitter.assert_message(
        dedent(
            """\
            Name               Since                 Visibility    Notes
            test-snap-private  2016-07-26T20:18:32Z  private       -
            test-snap-public   2016-07-26T20:18:32Z  public        -"""
        )
    )
