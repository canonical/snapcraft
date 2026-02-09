import argparse
import pathlib
import re
import sys
from unittest.mock import ANY, call

import craft_cli.errors
import pytest

from snapcraft import cli, commands, errors
from snapcraft.commands.upload import ComponentOption
from tests import unit

############
# Fixtures #
############


@pytest.fixture(autouse=True)
def fake_store_client_upload_file(mocker):
    fake_client = mocker.patch(
        "craft_store.BaseClient.upload_file",
        autospec=True,
        # return a different upload_id for each upload
        side_effect=[
            "2ecbfac1-3448-4e7d-85a4-7919b999f120",
            "227a7e65-b29f-4e62-af1c-c1969169d396",
        ],
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
def data_path() -> pathlib.Path:
    return pathlib.Path(unit.__file__).parents[1] / "legacy" / "data"


@pytest.fixture
def snap_file(data_path):
    return str((data_path / "test-snap.snap").resolve())


@pytest.fixture
def snap_file_with_started_at(data_path):
    return str((data_path / "test-snap-with-started-at.snap").resolve())


@pytest.fixture
def snap_file_with_component(data_path):
    return str((data_path / "test-snap-with-component.snap").resolve())


@pytest.fixture
def component_file(data_path):
    return str((data_path / "test-snap-with-component+test-component.comp").resolve())


##################
# Upload Command #
##################


@pytest.mark.usefixtures("memory_keyring")
def test_default(
    emitter,
    fake_store_notify_upload,
    fake_store_verify_upload,
    snap_file,
    fake_app_config,
):
    cmd = commands.StoreUploadCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            snap_file=snap_file,
            channels=None,
            component=[],
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
            components=None,
        )
    ]
    emitter.assert_message("Revision 10 created for 'basic'")


@pytest.mark.usefixtures("memory_keyring")
def test_push_error(
    snap_file,
    fake_app_config,
):
    """Error on the removed 'push' command."""
    cmd = commands.StoreLegacyPushCommand(fake_app_config)
    expected = re.escape("The 'push' command was renamed to 'upload'.")

    with pytest.raises(errors.RemovedCommand, match=expected):
        cmd.run(
            argparse.Namespace(
                snap_file=snap_file,
                channels=None,
                component=[],
            )
        )


@pytest.mark.usefixtures("memory_keyring")
def test_built_at(
    emitter,
    fake_store_notify_upload,
    fake_store_verify_upload,
    snap_file_with_started_at,
    fake_app_config,
):
    cmd = commands.StoreUploadCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            snap_file=snap_file_with_started_at,
            channels=None,
            component=[],
        )
    )

    assert fake_store_verify_upload.mock_calls == [call(ANY, snap_name="basic")]
    assert fake_store_notify_upload.mock_calls == [
        call(
            ANY,
            snap_name="basic",
            upload_id="2ecbfac1-3448-4e7d-85a4-7919b999f120",
            built_at="2019-05-07T19:25:53.939041Z",
            channels=None,
            snap_file_size=4096,
            components=None,
        )
    ]
    emitter.assert_message("Revision 10 created for 'basic'")


@pytest.mark.usefixtures("memory_keyring")
def test_default_channels(
    emitter,
    fake_store_notify_upload,
    fake_store_verify_upload,
    snap_file,
    fake_app_config,
):
    cmd = commands.StoreUploadCommand(fake_app_config)

    cmd.run(
        argparse.Namespace(
            snap_file=snap_file,
            channels="stable,edge",
            component=[],
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
            components=None,
        )
    ]
    emitter.assert_message(
        "Revision 10 created for 'basic' and released to 'edge' and 'stable'"
    )


def test_invalid_file(fake_app_config):
    cmd = commands.StoreUploadCommand(fake_app_config)

    with pytest.raises(craft_cli.errors.ArgumentParsingError) as raised:
        cmd.run(
            argparse.Namespace(
                snap_file="invalid.snap",
                channels=None,
                component=[],
            )
        )

    assert str(raised.value) == "'invalid.snap' is not a valid file"


##################################
# Upload command with components #
##################################


def test_componentoption_convert_ok():
    """Convert as expected."""
    r = ComponentOption("test-component=test-snap+test-component_1.0.comp")
    assert r.name == "test-component"
    assert r.path == pathlib.Path("test-snap+test-component_1.0.comp")


@pytest.mark.parametrize(
    "value",
    [
        pytest.param("namefile", id="no separation"),
        pytest.param("=file", id="no name"),
        pytest.param("  =file", id="no name, really!"),
        pytest.param("name=", id="no filename"),
        pytest.param("foo=bar=15", id="invalid name"),
    ],
)
def test_componentoption_convert_error(value):
    """Error while converting."""
    with pytest.raises(ValueError) as raised:
        ComponentOption(value)
    assert str(raised.value) == ("the `--component` format must be <name>=<path>")


def test_component_missing(capsys, mocker, snap_file_with_component):
    """Raise an error if a component is missing as a command line argument."""
    mocker.patch.object(sys, "argv", ["snapcraft", "upload", snap_file_with_component])

    cli.run()

    _, err = capsys.readouterr()

    assert (
        "Missing component(s): 'test-component'. Use `--component <name>=<filename>`."
    ) in err


def test_component_unknown(capsys, mocker, snap_file_with_component):
    """Raise an error if an unknown component is provided as a command line argument."""
    mocker.patch.object(
        sys,
        "argv",
        [
            "snapcraft",
            "upload",
            snap_file_with_component,
            "--component",
            "unknown-name=unknown-filename.comp",
        ],
    )

    cli.run()

    _, err = capsys.readouterr()

    assert "Unknown component(s) provided 'unknown-name'." in err


def test_component_missing_file(capsys, mocker, snap_file_with_component):
    """Raise an error if an unknown component is provided as a command line argument."""
    mocker.patch.object(
        sys,
        "argv",
        [
            "snapcraft",
            "upload",
            snap_file_with_component,
            "--component",
            "test-component=missing-file.comp",
        ],
    )

    cli.run()

    _, err = capsys.readouterr()

    assert (
        "File 'missing-file.comp' does not exist for component 'test-component'."
    ) in err


@pytest.mark.usefixtures("memory_keyring")
def test_components(
    emitter,
    fake_store_notify_upload,
    fake_store_verify_upload,
    snap_file_with_component,
    component_file,
    mocker,
):
    """Upload a snap with components."""
    mocker.patch.object(
        sys,
        "argv",
        [
            "snapcraft",
            "upload",
            snap_file_with_component,
            "--component",
            f"test-component={component_file}",
        ],
    )

    cli.run()

    assert fake_store_verify_upload.mock_calls == [
        call(ANY, snap_name="test-snap-with-component")
    ]
    assert fake_store_notify_upload.mock_calls == [
        call(
            ANY,
            snap_name="test-snap-with-component",
            upload_id="2ecbfac1-3448-4e7d-85a4-7919b999f120",
            built_at=None,
            channels=None,
            snap_file_size=16384,
            components={"test-component": "227a7e65-b29f-4e62-af1c-c1969169d396"},
        )
    ]
    emitter.assert_message("Revision 10 created for 'test-snap-with-component'")
