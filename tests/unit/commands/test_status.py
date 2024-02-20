import argparse
from textwrap import dedent

import pytest

from snapcraft import commands
from snapcraft.store import channel_map
from snapcraft_legacy.storeapi.v2.releases import Releases

############
# Fixtures #
############


@pytest.fixture
def channel_map_result():
    return channel_map.ChannelMap.unmarshal(
        {
            "channel-map": [
                {
                    "architecture": "amd64",
                    "channel": "2.1/beta",
                    "expiration-date": None,
                    "revision": 19,
                    "progressive": {
                        "paused": None,
                        "percentage": None,
                        "current-percentage": None,
                    },
                },
                {
                    "architecture": "amd64",
                    "channel": "2.0/beta",
                    "expiration-date": None,
                    "revision": 18,
                    "progressive": {
                        "paused": None,
                        "percentage": None,
                        "current-percentage": None,
                    },
                },
            ],
            "revisions": [
                {"architectures": ["amd64"], "revision": 19, "version": "10"},
                {"architectures": ["amd64"], "revision": 18, "version": "10"},
            ],
            "snap": {
                "name": "snap-test",
                "channels": [
                    {
                        "branch": None,
                        "fallback": None,
                        "name": "2.1/stable",
                        "risk": "stable",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/stable",
                        "name": "2.1/candidate",
                        "risk": "candidate",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/candidate",
                        "name": "2.1/beta",
                        "risk": "beta",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/beta",
                        "name": "2.1/edge",
                        "risk": "edge",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": None,
                        "name": "2.0/stable",
                        "risk": "stable",
                        "track": "2.0",
                    },
                    {
                        "branch": None,
                        "fallback": "2.0/stable",
                        "name": "2.0/candidate",
                        "risk": "candidate",
                        "track": "2.0",
                    },
                    {
                        "branch": None,
                        "fallback": "2.0/candidate",
                        "name": "2.0/beta",
                        "risk": "beta",
                        "track": "2.0",
                    },
                    {
                        "branch": None,
                        "fallback": "2.0/beta",
                        "name": "2.0/edge",
                        "risk": "edge",
                        "track": "2.0",
                    },
                ],
                "default-track": "2.1",
                "tracks": [
                    {
                        "name": "2.0",
                        "status": "default",
                        "creation-date": "2019-10-17T14:11:59Z",
                        "version-pattern": "2\\.*",
                    },
                    {
                        "name": "latest",
                        "status": "active",
                        "creation-date": None,
                        "version-pattern": None,
                    },
                ],
            },
        }
    )


@pytest.fixture
def fake_store_get_status_map(mocker, channel_map_result):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.get_channel_map",
        autospec=True,
        return_value=channel_map_result,
    )
    return fake_client


@pytest.fixture
def list_revisions_result():
    return Releases.unmarshal(
        {
            "revisions": [
                {
                    "architectures": ["i386"],
                    "base": "core20",
                    "build_url": None,
                    "confinement": "strict",
                    "created_at": " 2016-09-27T19:23:40Z",
                    "grade": "stable",
                    "revision": 2,
                    "sha3-384": "fake-a9060ef4872ccacbfa440617",
                    "size": 20,
                    "status": "Published",
                    "version": "2.0.1",
                },
                {
                    "architectures": ["amd64"],
                    "base": "core20",
                    "build_url": None,
                    "confinement": "strict",
                    "created_at": "2016-09-27T18:38:43Z",
                    "grade": "stable",
                    "revision": 1,
                    "sha3-384": "fake-a9060ef4872ccacbfa440617",
                    "size": 20,
                    "status": "Published",
                    "version": "2.0.2",
                },
            ],
            "releases": [
                {
                    "architecture": "amd64",
                    "branch": None,
                    "channel": "latest/stable",
                    "expiration-date": None,
                    "revision": 1,
                    "risk": "stable",
                    "track": "latest",
                    "when": "2020-02-12T17:51:40.891996Z",
                },
                {
                    "architecture": "i386",
                    "branch": None,
                    "channel": "latest/stable",
                    "expiration-date": None,
                    "revision": None,
                    "risk": "stable",
                    "track": "latest",
                    "when": "2020-02-11T17:51:40.891996Z",
                },
                {
                    "architecture": "amd64",
                    "branch": None,
                    "channel": "latest/edge",
                    "expiration-date": None,
                    "revision": 1,
                    "risk": "stable",
                    "track": "latest",
                    "when": "2020-01-12T17:51:40.891996Z",
                },
            ],
        }
    )


@pytest.fixture
def fake_store_list_revisions(mocker, list_revisions_result):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.list_revisions",
        autospec=True,
        return_value=list_revisions_result,
    )

    return fake_client


##################
# Status Command #
##################


@pytest.mark.usefixtures("memory_keyring", "fake_store_get_status_map")
def test_default(emitter):
    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         19          -\n"
        "                 edge       ↑          ↑           -\n"
        "2.0      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         18          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_following(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map = [
        channel_map.MappedChannel(
            channel="2.1/stable",
            architecture="amd64",
            expiration_date="2020-02-03T20:58:37Z",
            revision=20,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    ]
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["amd64"], revision=20, version="10")
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      amd64   stable     10         20          -\n"
        "                 candidate  ↑          ↑           -\n"
        "                 beta       ↑          ↑           -\n"
        "                 edge       ↑          ↑           -"
        ""
    )


@pytest.mark.usefixtures("memory_keyring")
def test_no_releases(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map = []

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message("This snap has no released revisions")


@pytest.mark.usefixtures("memory_keyring")
def test_progressive(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/beta",
            architecture="amd64",
            expiration_date="2020-02-03T20:58:37Z",
            revision=20,
            progressive=channel_map.Progressive(
                paused=None, percentage=10.0, current_percentage=7.2
            ),
        )
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["amd64"], revision=20, version="11")
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         19          93→90%\n"
        "                            11         20          7→10%\n"
        "                 edge       ↑          ↑           -\n"
        "2.0      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         18          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_arch(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/beta",
            architecture="s390x",
            expiration_date=None,
            revision=99,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["s390x"], revision=99, version="10")
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=["s390x"],
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      s390x   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         99          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_multiple_arch(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/beta",
            architecture="s390x",
            expiration_date=None,
            revision=98,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    )
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/beta",
            architecture="arm64",
            expiration_date=None,
            revision=99,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["s390x"], revision=98, version="10")
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["arm64"], revision=99, version="10")
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=["s390x", "arm64"],
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      arm64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         99          -\n"
        "                 edge       ↑          ↑           -\n"
        "         s390x   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         98          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_track(emitter, fake_store_get_status_map):
    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=["2.0"],
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.0      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         18          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_multi_track(emitter, fake_store_get_status_map):
    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=["2.0", "2.1"],
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         19          -\n"
        "                 edge       ↑          ↑           -\n"
        "2.0      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         18          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_arch_and_track(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/beta",
            architecture="s390x",
            expiration_date=None,
            revision=99,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    )
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.0/beta",
            architecture="s390x",
            expiration_date=None,
            revision=98,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["s390x"], revision=98, version="10")
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["s390x"], revision=99, version="10")
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=["s390x"],
            track=["2.1"],
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      s390x   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         99          -\n"
        "                 edge       ↑          ↑           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_branch(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/stable/hotfix1",
            architecture="amd64",
            expiration_date="2020-02-03T20:58:37Z",
            revision=20,
            progressive=channel_map.Progressive(
                paused=None, percentage=None, current_percentage=None
            ),
        )
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["am64"], revision=20, version="10hotfix")
    )
    channel_map_result.snap.channels.append(
        channel_map.SnapChannel(
            name="2.1/stable/hotfix1",
            track="2.1",
            risk="stable",
            branch="hotfix1",
            fallback="2.1/stable",
        )
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel         Version    Revision    Progress    Expires at\n"
        "2.1      amd64   stable          -          -           -           -\n"
        "                 stable/hotfix1  10hotfix   20          -           2020-02-03T20:58:37Z\n"
        "                 candidate       -          -           -           -\n"
        "                 beta            10         19          -           -\n"
        "                 edge            ↑          ↑           -           -\n"
        "2.0      amd64   stable          -          -           -           -\n"
        "                 candidate       -          -           -           -\n"
        "                 beta            10         18          -           -\n"
        "                 edge            ↑          ↑           -           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_progressive_branch(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map.append(
        channel_map.MappedChannel(
            channel="2.1/stable/hotfix1",
            architecture="amd64",
            expiration_date="2020-02-03T20:58:37Z",
            revision=20,
            progressive=channel_map.Progressive(
                paused=None, percentage=20.0, current_percentage=12.3
            ),
        )
    )
    channel_map_result.revisions.append(
        channel_map.Revision(architectures=["am64"], revision=20, version="10hotfix")
    )
    channel_map_result.snap.channels.append(
        channel_map.SnapChannel(
            name="2.1/stable/hotfix1",
            track="2.1",
            risk="stable",
            branch="hotfix1",
            fallback="2.1/stable",
        )
    )
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel         Version    Revision    Progress    Expires at\n"
        "2.1      amd64   stable          -          -           -           -\n"
        "                 stable/hotfix1  10hotfix   20          12→20%      2020-02-03T20:58:37Z\n"
        "                 candidate       -          -           -           -\n"
        "                 beta            10         19          -           -\n"
        "                 edge            ↑          ↑           -           -\n"
        "2.0      amd64   stable          -          -           -           -\n"
        "                 candidate       -          -           -           -\n"
        "                 beta            10         18          -           -\n"
        "                 edge            ↑          ↑           -           -"
    )


@pytest.mark.usefixtures("memory_keyring")
def test_progressive_unknown(emitter, fake_store_get_status_map, channel_map_result):
    channel_map_result.channel_map[0].progressive.percentage = 10.0
    channel_map_result.channel_map[0].progressive.current_percentage = None
    fake_store_get_status_map.return_value = channel_map_result

    cmd = commands.core22.StoreStatusCommand(None)

    cmd.run(
        argparse.Namespace(
            name="test-snap",
            arch=None,
            track=None,
        )
    )

    emitter.assert_message(
        "Track    Arch    Channel    Version    Revision    Progress\n"
        "2.1      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       -          -           -\n"
        "                            10         19          ?→10%\n"
        "                 edge       ↑          ↑           -\n"
        "2.0      amd64   stable     -          -           -\n"
        "                 candidate  -          -           -\n"
        "                 beta       10         18          -\n"
        "                 edge       ↑          ↑           -"
    )


#######################
# List Tracks Command #
#######################


@pytest.mark.parametrize(
    "command_class",
    [
        commands.core22.StoreListTracksCommand,
        commands.core22.StoreTracksCommand,
    ],
)
@pytest.mark.usefixtures("memory_keyring", "fake_store_get_status_map")
def test_list_tracks(emitter, command_class):
    cmd = command_class(None)

    cmd.run(argparse.Namespace(name="test-snap"))

    emitter.assert_message(
        "Name    Status    Creation-Date         Version-Pattern\n"
        "latest  active    -                     -\n"
        "2.0     default   2019-10-17T14:11:59Z  2\\.*"
    )


##########################
# List Revisions Command #
##########################


@pytest.mark.usefixtures("memory_keyring", "fake_store_list_revisions")
def test_list_revisions(emitter):
    cmd = commands.core22.StoreListRevisionsCommand(None)

    cmd.run(argparse.Namespace(snap_name="test-snap", arch=None))

    emitter.assert_message(
        dedent(
            """\
            Rev.    Uploaded              Arches    Version    Channels
            2       2016-09-27T19:23:40Z  i386      2.0.1      -
            1       2016-09-27T18:38:43Z  amd64     2.0.2      latest/edge*,latest/stable*"""
        )
    )


@pytest.mark.usefixtures("memory_keyring", "fake_store_list_revisions")
def test_list_revisions_arch(emitter):
    cmd = commands.core22.StoreListRevisionsCommand(None)

    cmd.run(argparse.Namespace(snap_name="test-snap", arch="amd64"))

    emitter.assert_message(
        dedent(
            """\
            Rev.    Uploaded              Arches    Version    Channels
            1       2016-09-27T18:38:43Z  amd64     2.0.2      latest/edge*,latest/stable*"""
        )
    )


@pytest.mark.usefixtures("memory_keyring", "fake_store_list_revisions")
def test_list_revisions_no_release_information(emitter, list_revisions_result):
    list_revisions_result.releases = []

    cmd = commands.core22.StoreListRevisionsCommand(None)

    cmd.run(argparse.Namespace(snap_name="test-snap", arch=None))

    emitter.assert_message(
        dedent(
            """\
            Rev.    Uploaded              Arches    Version
            2       2016-09-27T19:23:40Z  i386      2.0.1
            1       2016-09-27T18:38:43Z  amd64     2.0.2"""
        )
    )
