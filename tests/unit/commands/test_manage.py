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
from unittest.mock import ANY, call

import pytest

from snapcraft import commands, errors

############
# Fixtures #
############


@pytest.fixture
def fake_store_release(mocker):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.release",
        autospec=True,
    )
    return fake_client


@pytest.fixture
def fake_store_close(mocker):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.close",
        autospec=True,
    )
    return fake_client


@pytest.fixture
def fake_store_get_account_info(mocker):
    # reduced payload
    data = {
        "snaps": {
            "16": {
                "test-snap": {
                    "snap-id": "12345678",
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


###################
# Release Command #
###################


@pytest.mark.usefixtures("memory_keyring")
def test_release(emitter, fake_store_release):
    cmd = commands.StoreReleaseCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap", revision=10, channels="edge", progressive_percentage=None
        )
    )

    assert fake_store_release.mock_calls == [
        call(
            ANY,
            snap_name="test-snap",
            revision=10,
            channels=["edge"],
            progressive_percentage=None,
        )
    ]
    emitter.assert_message("Released 'test-snap' revision 10 to channels: 'edge'")


@pytest.mark.usefixtures("memory_keyring")
def test_release_multiple_channels(emitter, fake_store_release):
    cmd = commands.StoreReleaseCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            revision=10,
            channels="edge,latest/stable,1.0/beta",
            progressive_percentage=None,
        )
    )

    assert fake_store_release.mock_calls == [
        call(
            ANY,
            snap_name="test-snap",
            revision=10,
            channels=["edge", "latest/stable", "1.0/beta"],
            progressive_percentage=None,
        )
    ]
    emitter.assert_message(
        "Released 'test-snap' revision 10 to channels: '1.0/beta', 'edge', and 'latest/stable'"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_release_progressive(emitter, fake_store_release):
    cmd = commands.StoreReleaseCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap", revision=10, channels="edge", progressive_percentage=10
        )
    )

    assert fake_store_release.mock_calls == [
        call(
            ANY,
            snap_name="test-snap",
            revision=10,
            channels=["edge"],
            progressive_percentage=10,
        )
    ]
    emitter.assert_message("Released 'test-snap' revision 10 to channels: 'edge'")


#################
# Close Command #
#################


@pytest.mark.usefixtures("memory_keyring", "fake_store_get_account_info")
def test_close(emitter, fake_store_close):
    cmd = commands.StoreCloseCommand(None)

    cmd.run(argparse.Namespace(name="test-snap", channel="edge"))

    assert fake_store_close.mock_calls == [
        call(
            ANY,
            snap_id="12345678",
            channel="edge",
        )
    ]
    emitter.assert_message("Channel 'edge' for 'test-snap' is now closed")


@pytest.mark.usefixtures("memory_keyring", "fake_store_get_account_info")
def test_close_no_snap_id(emitter):
    cmd = commands.StoreCloseCommand(None)

    with pytest.raises(errors.SnapcraftError) as raised:
        cmd.run(argparse.Namespace(name="test-unknown-snap", channel="edge"))

    assert str(raised.value) == (
        "'test-unknown-snap' not found or not owned by this account"
    )

    emitter.assert_debug(
        "KeyError('test-unknown-snap') no found in "
        "{'snaps': {'16': {'test-snap': {'snap-id': '12345678'}}}}"
    )
