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
import re
from textwrap import dedent
from unittest.mock import ANY, call

import craft_cli
import pytest

from snapcraft import commands, store

############
# Fixtures #
############


@pytest.fixture
def fake_store_login(mocker):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.login",
        autospec=True,
        return_value="secret",
    )
    return fake_client


#################
# Login Command #
#################


@pytest.mark.usefixtures("memory_keyring")
def test_login(emitter, fake_store_login, fake_app_config):
    cmd = commands.StoreLoginCommand(fake_app_config)

    cmd.run(argparse.Namespace(login_with=None, experimental_login=False))

    assert fake_store_login.mock_calls == [
        call(
            ANY,
        )
    ]
    emitter.assert_message("Login successful")


def test_login_with_file_error(emitter, mocker, legacy_config_path, fake_app_config):
    legacy_config_path.write_text("secretb64")
    expected = re.escape(
        "'--with' is no longer supported. Export the auth to the environment "
        f"variable {store.constants.ENVIRONMENT_STORE_CREDENTIALS!r} instead."
    )

    cmd = commands.StoreLoginCommand(fake_app_config)

    with pytest.raises(craft_cli.errors.ArgumentParsingError, match=expected):
        cmd.run(
            argparse.Namespace(
                login_with=str(legacy_config_path), experimental_login=False
            )
        )


def test_login_with_experimental_fails(fake_app_config):
    cmd = commands.StoreLoginCommand(fake_app_config)
    expected = re.escape(
        "'--experimental-login' is no longer supported. Set SNAPCRAFT_STORE_AUTH=candid instead."
    )

    with pytest.raises(craft_cli.errors.ArgumentParsingError, match=expected):
        cmd.run(argparse.Namespace(login_with=None, experimental_login=True))


########################
# Export Login Command #
########################


def test_export_login(emitter, fake_store_login, fake_app_config):
    cmd = commands.StoreExportLoginCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            login_file="-",
            snaps=None,
            channels=None,
            acls=None,
            expires=None,
            experimental_login=False,
        )
    )

    assert fake_store_login.mock_calls == [
        call(
            ANY,
        )
    ]
    emitter.assert_message(
        "Exported login credentials:\nsecret"
        "\n\nThese credentials must be used on Snapcraft 7.2 or greater."
    )


def test_export_login_file(project_path, emitter, fake_store_login, fake_app_config):
    cmd = commands.StoreExportLoginCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            login_file="target_file",
            snaps=None,
            channels=None,
            acls=None,
            expires=None,
            experimental_login=False,
        )
    )

    assert fake_store_login.mock_calls == [
        call(
            ANY,
        )
    ]
    emitter.assert_message(
        "Exported login credentials to 'target_file'"
        "\n\nThese credentials must be used on Snapcraft 7.2 or greater."
    )
    login_file = project_path / "target_file"
    assert login_file.exists()
    assert login_file.read_text() == "secret"


def test_export_login_with_params(emitter, fake_store_login, fake_app_config):
    cmd = commands.StoreExportLoginCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            login_file="-",
            snaps="fake-snap,fake-other-snap",
            channels="stable,edge",
            acls="package_manage,package_push",
            expires="2030-12-12",
            experimental_login=False,
        )
    )

    assert fake_store_login.mock_calls == [
        call(
            ANY,
            packages=["fake-snap", "fake-other-snap"],
            channels=["stable", "edge"],
            acls=["package_manage", "package_push"],
            ttl=ANY,
        )
    ]
    emitter.assert_message(
        "Exported login credentials:\nsecret"
        "\n\nThese credentials must be used on Snapcraft 7.2 or greater."
    )


def test_export_login_with_candid(
    emitter, fake_store_login, monkeypatch, fake_app_config
):
    monkeypatch.setenv("SNAPCRAFT_STORE_AUTH", "candid")

    cmd = commands.StoreExportLoginCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            login_file="-",
            snaps="fake-snap,fake-other-snap",
            channels="stable,edge",
            acls="package_manage,package_push",
            expires="2030-12-12",
            experimental_login=False,
        )
    )

    assert fake_store_login.mock_calls == [
        call(
            ANY,
            packages=["fake-snap", "fake-other-snap"],
            channels=["stable", "edge"],
            acls=["package_manage", "package_push"],
            ttl=ANY,
        )
    ]
    emitter.assert_message(
        "Exported login credentials:\nsecret"
        "\n\nThese credentials must be used on Snapcraft 7.2 or greater."
        "\nSet 'SNAPCRAFT_STORE_AUTH=candid' for these credentials to work."
    )


def test_export_login_with_experimental_fails(fake_app_config):
    cmd = commands.StoreExportLoginCommand(fake_app_config)
    expected = re.escape(
        "'--experimental-login' is no longer supported. Set SNAPCRAFT_STORE_AUTH=candid instead."
    )

    with pytest.raises(craft_cli.errors.ArgumentParsingError, match=expected):
        cmd.run(
            argparse.Namespace(
                login_file="-",
                snaps=None,
                channels=None,
                acls=None,
                expires=None,
                experimental_login=True,
            )
        )


##################
# WhoAmI Command #
##################


def test_who(emitter, fake_client, fake_app_config):
    fake_client.whoami.return_value = {
        "account": {"email": "user@acme.org", "id": "id", "username": "user"},
        "expires": "2023-04-22T21:48:57.000",
    }

    cmd = commands.StoreWhoAmICommand(fake_app_config)

    cmd.run(argparse.Namespace())

    assert fake_client.whoami.mock_calls == [call()]
    expected_message = dedent(
        """\
        email: user@acme.org
        username: user
        id: id
        permissions: no restrictions
        channels: no restrictions
        expires: 2023-04-22T21:48:57.000Z"""
    )
    emitter.assert_message(expected_message)


def test_who_with_attenuations(emitter, fake_client, fake_app_config):
    fake_client.whoami.return_value = {
        "account": {"email": "user@acme.org", "id": "id", "username": "user"},
        "permissions": ["package_manage", "package_access"],
        "channels": ["edge", "beta"],
        "expires": "2023-04-22T21:48:57.000",
    }

    cmd = commands.StoreWhoAmICommand(fake_app_config)

    cmd.run(argparse.Namespace())

    assert fake_client.whoami.mock_calls == [call()]
    expected_message = dedent(
        """\
        email: user@acme.org
        username: user
        id: id
        permissions: package_manage, package_access
        channels: edge, beta
        expires: 2023-04-22T21:48:57.000Z"""
    )
    emitter.assert_message(expected_message)


def test_who_no_expires(emitter, fake_client, fake_app_config):
    fake_client.whoami.return_value = {
        "account": {"email": "user@acme.org", "id": "id", "username": "user"},
    }

    cmd = commands.StoreWhoAmICommand(fake_app_config)

    cmd.run(argparse.Namespace())

    assert fake_client.whoami.mock_calls == [call()]
    expected_message = dedent(
        """\
        email: user@acme.org
        username: user
        id: id
        permissions: no restrictions
        channels: no restrictions
        expires: N/A"""
    )
    emitter.assert_message(expected_message)


##################
# Logout Command #
##################


def test_logout(emitter, fake_client, fake_app_config):
    cmd = commands.StoreLogoutCommand(fake_app_config)

    cmd.run(argparse.Namespace())

    assert fake_client.logout.mock_calls == [call()]
    emitter.assert_message("Credentials cleared")
