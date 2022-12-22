import argparse
import pathlib
from unittest.mock import ANY, call

import craft_cli.errors
import pytest

from snapcraft import commands
from tests import unit

############
# Fixtures #
############


@pytest.fixture(autouse=True)
def fake_store_client_upload_file(mocker):
    fake_client = mocker.patch(
        "craft_store.BaseClient.upload_file",
        autospec=True,
        return_value="2ecbfac1-3448-4e7d-85a4-7919b999f120",
    )
    return fake_client


@pytest.fixture
def fake_store_notify_upload(mocker):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.notify_upload",
        autospec=True,
        return_value=10,
    )
    return fake_client


@pytest.fixture
def fake_store_verify_upload(mocker):
    fake_client = mocker.patch(
        "snapcraft.store.StoreClientCLI.verify_upload",
        autospec=True,
        return_value=None,
    )
    return fake_client


@pytest.fixture
def snap_file():
    return str(
        (
            pathlib.Path(unit.__file__)
            / ".."
            / ".."
            / "legacy"
            / "data"
            / "test-snap.snap"
        ).resolve()
    )


##################
# Upload Command #
##################


@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.parametrize(
    "command_class", (commands.StoreUploadCommand, commands.StoreLegacyPushCommand)
)
def test_default(
    emitter,
    fake_store_notify_upload,
    fake_store_verify_upload,
    snap_file,
    command_class,
):
    cmd = command_class(None)

    cmd.run(
        argparse.Namespace(
            snap_file=snap_file,
            channels=None,
        )
    )

    assert fake_store_verify_upload.mock_calls == [call(ANY, snap_name="basic")]
    assert fake_store_notify_upload.mock_calls == [
        call(
            ANY,
            snap_name="basic",
            upload_id="2ecbfac1-3448-4e7d-85a4-7919b999f120",
            built_at=None,
            channels=None,
            snap_file_size=4096,
        )
    ]
    emitter.assert_message("Revision 10 created for 'basic'")


@pytest.mark.usefixtures("memory_keyring")
def test_default_channels(
    emitter, fake_store_notify_upload, fake_store_verify_upload, snap_file
):
    cmd = commands.StoreUploadCommand(None)

    cmd.run(
        argparse.Namespace(
            snap_file=snap_file,
            channels="stable,edge",
        )
    )

    assert fake_store_verify_upload.mock_calls == [call(ANY, snap_name="basic")]
    assert fake_store_notify_upload.mock_calls == [
        call(
            ANY,
            snap_name="basic",
            upload_id="2ecbfac1-3448-4e7d-85a4-7919b999f120",
            built_at=None,
            channels=["stable", "edge"],
            snap_file_size=4096,
        )
    ]
    emitter.assert_message(
        "Revision 10 created for 'basic' and released to 'edge' and 'stable'"
    )


def test_invalid_file():
    cmd = commands.StoreUploadCommand(None)

    with pytest.raises(craft_cli.errors.ArgumentParsingError) as raised:
        cmd.run(
            argparse.Namespace(
                snap_file="invalid.snap",
                channels=None,
            )
        )

    assert str(raised.value) == "'invalid.snap' is not a valid file"
