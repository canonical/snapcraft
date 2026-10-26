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
import os
from unittest.mock import ANY, call

import pytest
from craft_cli.errors import ArgumentParsingError

from snapcraft import commands, errors, store
from snapcraft.store.channel_map import ChannelMap

SNAP_STATUS_PAYLOAD = {
    "channel_map_tree": {
        "latest": {
            "16": {
                "amd64": [
                    {
                        "channel": "candidate",
                        "info": "specific",
                        "revision": 10,
                        "version": "1.0",
                    },
                    {"channel": "stable", "info": "none"},
                ],
                "arm64": [
                    {
                        "channel": "candidate",
                        "info": "specific",
                        "revision": 11,
                        "version": "1.0",
                    },
                    {"channel": "stable", "info": "none"},
                ],
            }
        }
    }
}

SNAP_STATUS_PAYLOAD_PARTIAL = {
    "channel_map_tree": {
        "latest": {
            "16": {
                "amd64": [
                    {
                        "channel": "candidate",
                        "info": "specific",
                        "revision": 10,
                        "version": "1.0",
                    },
                ],
                "arm64": [
                    {"channel": "candidate", "info": "none"},
                ],
            }
        }
    }
}


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


@pytest.fixture
def fake_store_upload_metadata(mocker):
    return mocker.patch(
        "snapcraft.store.StoreClientCLI.upload_metadata",
        autospec=True,
    )


@pytest.fixture
def fake_store_get_snap_status(mocker):
    return mocker.patch(
        "snapcraft.store.StoreClientCLI.get_snap_status",
        autospec=True,
        return_value=SNAP_STATUS_PAYLOAD,
    )


@pytest.fixture
def fake_store_get_snap_status_partial(mocker):
    return mocker.patch(
        "snapcraft.store.StoreClientCLI.get_snap_status",
        autospec=True,
        return_value=SNAP_STATUS_PAYLOAD_PARTIAL,
    )


###################
# Release Command #
###################


@pytest.mark.usefixtures("memory_keyring")
def test_release(emitter, fake_store_release, fake_app_config):
    cmd = commands.StoreReleaseCommand(fake_app_config)

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
def test_release_multiple_channels(emitter, fake_store_release, fake_app_config):
    cmd = commands.StoreReleaseCommand(fake_app_config)

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
def test_release_progressive(emitter, fake_store_release, fake_app_config):
    cmd = commands.StoreReleaseCommand(fake_app_config)

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
    emitter.assert_message(
        "Released 'test-snap' revision 10 to channels: 'edge' for 10% of users"
    )


#################
# Close Command #
#################


@pytest.mark.usefixtures("memory_keyring")
def test_close(emitter, fake_store_close, fake_app_config):
    cmd = commands.StoreCloseCommand(fake_app_config)

    cmd.run(argparse.Namespace(name="test-snap", channel="edge", arch=None))

    assert fake_store_close.mock_calls == [
        call(
            ANY,
            snap_name="test-snap",
            channel="edge",
        )
    ]
    emitter.assert_message("Channel 'edge' for 'test-snap' is now closed")


@pytest.mark.usefixtures("memory_keyring", "fake_store_get_account_info")
def test_close_no_snap_id(emitter, fake_app_config):
    cmd = commands.StoreCloseCommand(fake_app_config)

    with pytest.raises(errors.SnapcraftError) as raised:
        cmd.run(argparse.Namespace(name="test-unknown-snap", channel="edge", arch=None))

    assert str(raised.value) == (
        "'test-unknown-snap' not found or not owned by this account"
    )

    emitter.assert_debug(
        "KeyError('test-unknown-snap') no found in "
        "{'snaps': {'16': {'test-snap': {'snap-id': '12345678'}}}}"
    )


def _mapped(channel, arch, revision, percentage=None):
    return {
        "channel": channel,
        "architecture": arch,
        "revision": revision,
        "expiration-date": None,
        "progressive": {
            "paused": None,
            "percentage": percentage,
            "current-percentage": None,
        },
    }


def _channel_map(channel_map, revisions):
    return ChannelMap.unmarshal(
        {
            "channel-map": channel_map,
            "revisions": [
                {"revision": rev, "version": "1.0", "architectures": arches}
                for rev, arches in revisions.items()
            ],
            "snap": {
                "name": "test-snap",
                "channels": [
                    {
                        "name": "latest/stable",
                        "track": "latest",
                        "risk": "stable",
                        "branch": None,
                        "fallback": None,
                    }
                ],
            },
        }
    )


@pytest.fixture
def fake_store_get_channel_map(mocker):
    return mocker.patch(
        "snapcraft.store.StoreClientCLI.get_channel_map",
        autospec=True,
        return_value=_channel_map(
            [
                _mapped("latest/stable", "amd64", 10),
                _mapped("latest/stable", "arm64", 11),
                _mapped("latest/stable", "armhf", 12),
                _mapped("latest/candidate", "arm64", 13),
            ],
            {10: ["amd64"], 11: ["arm64"], 12: ["armhf"], 13: ["arm64"]},
        ),
    )


@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.parametrize("channel", ["stable", "latest/stable"])
def test_close_arch(
    emitter,
    fake_store_close,
    fake_store_release,
    fake_store_get_channel_map,
    fake_app_config,
    channel,
):
    cmd = commands.StoreCloseCommand(fake_app_config)

    cmd.run(argparse.Namespace(name="test-snap", channel=channel, arch=["amd64"]))

    assert fake_store_close.mock_calls == [
        call(ANY, snap_name="test-snap", channel=channel)
    ]
    assert fake_store_release.mock_calls == [
        call(ANY, "test-snap", revision=11, channels=[channel]),
        call(ANY, "test-snap", revision=12, channels=[channel]),
    ]
    emitter.assert_progress(
        f"Closing {channel!r} and releasing revisions 11 and 12 to it again",
        permanent=True,
    )
    emitter.assert_message(
        f"Channel {channel!r} for 'test-snap' is now closed for 'amd64'"
    )


@pytest.mark.usefixtures("memory_keyring", "fake_store_get_channel_map")
def test_close_arch_all(emitter, fake_store_close, fake_store_release, fake_app_config):
    cmd = commands.StoreCloseCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            name="test-snap", channel="stable", arch=["amd64", "arm64", "armhf"]
        )
    )

    assert fake_store_close.mock_calls == [
        call(ANY, snap_name="test-snap", channel="stable")
    ]
    assert fake_store_release.mock_calls == []
    emitter.assert_message(
        "Channel 'stable' for 'test-snap' is now closed for "
        "'amd64', 'arm64', and 'armhf'"
    )


@pytest.mark.usefixtures("memory_keyring", "fake_store_get_channel_map")
def test_close_arch_not_released(fake_store_close, fake_store_release, fake_app_config):
    cmd = commands.StoreCloseCommand(fake_app_config)

    with pytest.raises(errors.SnapcraftError) as raised:
        cmd.run(
            argparse.Namespace(name="test-snap", channel="candidate", arch=["amd64"])
        )

    assert str(raised.value) == (
        "Channel 'candidate' for 'test-snap' has no release for 'amd64'"
    )
    assert fake_store_close.mock_calls == []
    assert fake_store_release.mock_calls == []


@pytest.mark.usefixtures("memory_keyring")
def test_close_arch_shared_revision(
    fake_store_close, fake_store_release, fake_store_get_channel_map, fake_app_config
):
    fake_store_get_channel_map.return_value = _channel_map(
        [
            _mapped("latest/stable", "amd64", 10),
            _mapped("latest/stable", "arm64", 10),
        ],
        {10: ["amd64", "arm64"]},
    )
    cmd = commands.StoreCloseCommand(fake_app_config)

    with pytest.raises(errors.SnapcraftError) as raised:
        cmd.run(argparse.Namespace(name="test-snap", channel="stable", arch=["amd64"]))

    assert str(raised.value) == (
        "Revision 10 in 'stable' is also built for 'amd64', "
        "so it cannot be kept for the other architectures"
    )
    assert fake_store_close.mock_calls == []
    assert fake_store_release.mock_calls == []


@pytest.mark.usefixtures("memory_keyring")
def test_close_arch_progressive(
    fake_store_close, fake_store_release, fake_store_get_channel_map, fake_app_config
):
    fake_store_get_channel_map.return_value = _channel_map(
        [
            _mapped("latest/stable", "amd64", 10),
            _mapped("latest/stable", "arm64", 11),
            _mapped("latest/stable", "arm64", 12, percentage=50.0),
        ],
        {10: ["amd64"], 11: ["arm64"], 12: ["arm64"]},
    )
    cmd = commands.StoreCloseCommand(fake_app_config)

    with pytest.raises(errors.SnapcraftError) as raised:
        cmd.run(argparse.Namespace(name="test-snap", channel="stable", arch=["amd64"]))

    assert str(raised.value) == (
        "Channel 'stable' for 'test-snap' has a progressive release for 'arm64'"
    )
    assert fake_store_close.mock_calls == []
    assert fake_store_release.mock_calls == []


###########################
# SetDefaultTrack Command #
###########################


@pytest.mark.usefixtures("memory_keyring")
def test_set_default_track(emitter, fake_store_upload_metadata, fake_app_config):
    cmd = commands.StoreSetDefaultTrackCommand(fake_app_config)
    cmd.run(argparse.Namespace(snap_name="test-snap", track="test-track"))

    assert fake_store_upload_metadata.mock_calls == [
        call(
            ANY,
            snap_name="test-snap",
            metadata={"default_track": "test-track"},
            force=True,
        )
    ]
    emitter.assert_message("Default track for 'test-snap' set to 'test-track'.")


####################
# Promote Command  #
####################


@pytest.mark.usefixtures("memory_keyring")
def test_promote_with_yes(
    emitter, fake_store_get_snap_status, fake_store_release, fake_app_config
):
    cmd = commands.StorePromoteCommand(fake_app_config)
    result = cmd.run(
        argparse.Namespace(
            snap_name="test-snap",
            from_channel="candidate",
            to_channel="stable",
            yes=True,
        )
    )

    fake_store_release.assert_any_call(
        ANY, snap_name="test-snap", revision=10, channels=["stable"]
    )
    fake_store_release.assert_any_call(
        ANY, snap_name="test-snap", revision=11, channels=["stable"]
    )
    emitter.assert_message("Promotion from candidate to stable complete")
    assert result == os.EX_OK


@pytest.mark.usefixtures("memory_keyring")
def test_promote_user_confirms(
    emitter, fake_store_get_snap_status, fake_store_release, fake_app_config, mocker
):
    mocker.patch("craft_cli.emit.confirm", return_value=True)
    cmd = commands.StorePromoteCommand(fake_app_config)
    result = cmd.run(
        argparse.Namespace(
            snap_name="test-snap",
            from_channel="candidate",
            to_channel="stable",
            yes=False,
        )
    )

    assert fake_store_release.called
    emitter.assert_message("Promotion from candidate to stable complete")
    assert result == os.EX_OK


@pytest.mark.usefixtures("memory_keyring")
def test_promote_user_cancels(
    emitter, fake_store_get_snap_status, fake_store_release, fake_app_config, mocker
):
    mocker.patch("craft_cli.emit.confirm", return_value=False)
    cmd = commands.StorePromoteCommand(fake_app_config)
    result = cmd.run(
        argparse.Namespace(
            snap_name="test-snap",
            from_channel="candidate",
            to_channel="stable",
            yes=False,
        )
    )

    fake_store_release.assert_not_called()
    emitter.assert_message("Channel promotion cancelled")
    assert result == 1


@pytest.mark.usefixtures("memory_keyring")
def test_promote_same_channel_error(fake_app_config):
    cmd = commands.StorePromoteCommand(fake_app_config)

    with pytest.raises(ArgumentParsingError):
        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                from_channel="stable",
                to_channel="stable",
                yes=True,
            )
        )


@pytest.mark.usefixtures("memory_keyring")
def test_promote_partial_build_set_error(
    fake_store_get_snap_status_partial, fake_app_config
):
    cmd = commands.StorePromoteCommand(fake_app_config)

    with pytest.raises(store.errors.InvalidChannelSet):
        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                from_channel="candidate",
                to_channel="stable",
                yes=True,
            )
        )


@pytest.mark.usefixtures("memory_keyring")
def test_promote_no_releases_error(fake_store_get_snap_status, fake_app_config):
    cmd = commands.StorePromoteCommand(fake_app_config)

    with pytest.raises(store.errors.ChannelNotAvailableOnArchError):
        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                from_channel="edge",
                to_channel="stable",
                yes=True,
            )
        )


@pytest.mark.usefixtures("memory_keyring")
def test_promote_warn_snapcraft_has_tty(
    emitter,
    fake_store_get_snap_status,
    fake_store_release,
    fake_app_config,
    mocker,
    monkeypatch,
):
    monkeypatch.setenv("SNAPCRAFT_HAS_TTY", "1")
    mocker.patch("craft_cli.emit.confirm", return_value=False)
    cmd = commands.StorePromoteCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            snap_name="test-snap",
            from_channel="candidate",
            to_channel="stable",
            yes=False,
        )
    )

    emitter.assert_warning(
        "The 'SNAPCRAFT_HAS_TTY' environment variable is no longer used. "
        "Use '--yes' for non-interactive promotions."
    )
