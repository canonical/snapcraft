# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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
import json
import re
from textwrap import dedent
from unittest.mock import ANY, call

import pytest

from snapcraft import commands, errors

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
def fake_store_get_names(mocker):
    data = [
        ("test-snap-public", "2016-07-26T20:18:32Z", "public", "-"),
        ("test-snap-private", "2016-07-26T20:18:32Z", "private", "-"),
    ]

    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.get_names",
        autospec=True,
        return_value=data,
    )
    return fake_client


####################
# Register Command #
####################


@pytest.mark.usefixtures("memory_keyring")
def test_register_default(
    emitter, fake_confirmation_prompt, fake_store_register, fake_app_config
):
    fake_confirmation_prompt.return_value = True

    cmd = commands.StoreRegisterCommand(fake_app_config)

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
def test_register_yes(emitter, fake_store_register, fake_app_config):
    cmd = commands.StoreRegisterCommand(fake_app_config)

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
def test_register_no(
    emitter, fake_confirmation_prompt, fake_store_register, fake_app_config
):
    cmd = commands.StoreRegisterCommand(fake_app_config)

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
def test_register_private(emitter, fake_store_register, fake_app_config):
    cmd = commands.StoreRegisterCommand(fake_app_config)

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
def test_register_store_id(emitter, fake_store_register, fake_app_config):
    cmd = commands.StoreRegisterCommand(fake_app_config)

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


@pytest.mark.usefixtures("memory_keyring")
class TestNames:
    """Tests for the names command."""

    @pytest.mark.parametrize(
        "command_class",
        [commands.StoreLegacyListCommand, commands.StoreLegacyListRegisteredCommand],
    )
    def test_removed_command_error(
        self, emitter, fake_store_get_names, command_class, fake_app_config
    ):
        """Error on removed commands."""
        cmd = command_class(fake_app_config)
        namespace = argparse.Namespace(
            store_id="1234",
            private=False,
            yes=True,
            format="table",
            **{"snap-name": "test-snap"},
        )
        expected = re.escape(
            f"The {command_class.name!r} command was renamed to 'names'."
        )

        with pytest.raises(errors.RemovedCommand, match=expected):
            cmd.run(namespace)

    def test_table(self, emitter, fake_store_get_names, fake_app_config):
        cmd = commands.StoreNamesCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                store_id="1234",
                private=False,
                yes=True,
                format="table",
                **{"snap-name": "test-snap"},
            )
        )

        emitter.assert_message(
            dedent(
                """\
                Name               Since                 Visibility    Notes
                test-snap-private  2016-07-26T20:18:32Z  private       -
                test-snap-public   2016-07-26T20:18:32Z  public        -"""
            )
        )

    def test_json(self, emitter, fake_store_get_names, fake_app_config):
        cmd = commands.StoreNamesCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                store_id="1234",
                private=False,
                yes=True,
                format="json",
                **{"snap-name": "test-snap"},
            )
        )

        emitter.assert_message(
            json.dumps(
                {
                    "snaps": [
                        {
                            "name": "test-snap-private",
                            "since": "2016-07-26T20:18:32Z",
                            "visibility": "private",
                            "notes": None,
                        },
                        {
                            "name": "test-snap-public",
                            "since": "2016-07-26T20:18:32Z",
                            "visibility": "public",
                            "notes": None,
                        },
                    ],
                },
                indent=4,
            )
        )

    def test_format_error(self, emitter, fake_store_get_names, fake_app_config):
        cmd = commands.StoreNamesCommand(fake_app_config)

        with pytest.raises(NotImplementedError) as exc_info:
            cmd.run(
                argparse.Namespace(
                    store_id="1234",
                    private=False,
                    yes=True,
                    format="invalid",
                    **{"snap-name": "test-snap"},
                )
            )

        assert exc_info.value.args == ("Format not implemented", "invalid")
