# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2020 Canonical Ltd
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

import contextlib
import hashlib
import json
import logging
import operator
import os
import re
import subprocess
import tempfile
from datetime import datetime
from pathlib import Path
from subprocess import Popen
from typing import TYPE_CHECKING, Any, Dict, Iterable, List, Optional, TextIO, Tuple
from urllib.parse import urljoin

from tabulate import tabulate

from snapcraft import storeapi, yaml_utils

# Ideally we would move stuff into more logical components
from snapcraft.cli import echo
from snapcraft.file_utils import (
    calculate_sha3_384,
    get_host_tool_path,
    get_snap_tool_path,
)
from snapcraft.internal import cache, deltas
from snapcraft.internal.deltas.errors import (
    DeltaGenerationError,
    DeltaGenerationTooBigError,
)
from snapcraft.internal.errors import SnapDataExtractionError, ToolMissingError
from snapcraft.storeapi.constants import DEFAULT_SERIES

if TYPE_CHECKING:
    from snapcraft.storeapi._status_tracker import StatusTracker
    from snapcraft.storeapi.v2.channel_map import ChannelMap
    from snapcraft.storeapi.v2.releases import Releases


logger = logging.getLogger(__name__)


def _get_data_from_snap_file(snap_path):
    with tempfile.TemporaryDirectory() as temp_dir:
        unsquashfs_path = get_snap_tool_path("unsquashfs")
        try:
            output = subprocess.check_output(
                [
                    unsquashfs_path,
                    "-d",
                    os.path.join(temp_dir, "squashfs-root"),
                    snap_path,
                    "-e",
                    # cygwin unsquashfs on windows uses unix paths.
                    Path("meta", "snap.yaml").as_posix(),
                ]
            )
        except subprocess.CalledProcessError:
            raise SnapDataExtractionError(os.path.basename(snap_path))
        logger.debug(output)
        with open(
            os.path.join(temp_dir, "squashfs-root", "meta", "snap.yaml")
        ) as yaml_file:
            snap_yaml = yaml_utils.load(yaml_file)
    return snap_yaml


@contextlib.contextmanager
def _get_icon_from_snap_file(snap_path):
    icon_file = None
    with tempfile.TemporaryDirectory() as temp_dir:
        unsquashfs_path = get_snap_tool_path("unsquashfs")
        try:
            output = subprocess.check_output(
                [
                    unsquashfs_path,
                    "-d",
                    os.path.join(temp_dir, "squashfs-root"),
                    snap_path,
                    "-e",
                    "meta/gui",
                ]
            )
        except subprocess.CalledProcessError:
            raise SnapDataExtractionError(os.path.basename(snap_path))
        logger.debug("Output extracting icon from snap: %s", output)
        for extension in ("png", "svg"):
            icon_name = "icon.{}".format(extension)
            icon_path = os.path.join(temp_dir, "squashfs-root", "meta/gui", icon_name)
            if os.path.exists(icon_path):
                icon_file = open(icon_path, "rb")
                break
        try:
            yield icon_file
        finally:
            if icon_file is not None:
                icon_file.close()


def _get_url_from_error(error: storeapi.errors.StoreAccountInformationError) -> str:
    if error.extra:  # type: ignore
        return error.extra[0].get("url")  # type: ignore
    return ""


def _check_dev_agreement_and_namespace_statuses(store) -> None:
    """ Check the agreement and namespace statuses of the dev.
    Fail if either of those conditions is not met.
    Re-raise `StoreAccountInformationError` if we get an error and
    the error is not either of these.
    """
    # Check account information for the `developer agreement` status.
    try:
        store.get_account_information()
    except storeapi.errors.StoreAccountInformationError as e:
        if storeapi.constants.MISSING_AGREEMENT == e.error:  # type: ignore
            # A precaution if store does not return new style error.
            url = _get_url_from_error(e) or urljoin(
                storeapi.constants.STORE_DASHBOARD_URL, "/dev/tos"
            )
            choice = echo.prompt(storeapi.constants.AGREEMENT_INPUT_MSG.format(url))
            if choice in {"y", "Y"}:
                try:
                    store.sign_developer_agreement(latest_tos_accepted=True)
                except:  # noqa LP: #1733003
                    raise storeapi.errors.NeedTermsSignedError(
                        storeapi.constants.AGREEMENT_SIGN_ERROR.format(url)
                    )
            else:
                raise storeapi.errors.NeedTermsSignedError(
                    storeapi.constants.AGREEMENT_ERROR
                )

    # Now check account information for the `namespace` status.
    try:
        store.get_account_information()
    except storeapi.errors.StoreAccountInformationError as e:
        if storeapi.constants.MISSING_NAMESPACE in e.error:  # type: ignore
            # A precaution if store does not return new style error.
            url = _get_url_from_error(e) or urljoin(
                storeapi.constants.STORE_DASHBOARD_URL, "/dev/account"
            )
            raise storeapi.errors.NeedTermsSignedError(
                storeapi.constants.NAMESPACE_ERROR.format(url)
            )
        else:
            raise


def _try_login(
    email: str,
    password: str,
    *,
    store: storeapi.StoreClient,
    save: bool = True,
    packages: Iterable[Dict[str, str]] = None,
    acls: Iterable[str] = None,
    channels: Iterable[str] = None,
    expires: str = None,
    config_fd: TextIO = None,
) -> None:
    try:
        store.login(
            email=email,
            password=password,
            packages=packages,
            acls=acls,
            channels=channels,
            expires=expires,
            config_fd=config_fd,
            save=save,
        )
        if not config_fd:
            print()
            echo.wrapped(storeapi.constants.TWO_FACTOR_WARNING)
    except storeapi.http_clients.errors.StoreTwoFactorAuthenticationRequired:
        one_time_password = echo.prompt("Second-factor auth")
        store.login(
            email=email,
            password=password,
            otp=one_time_password,
            acls=acls,
            packages=packages,
            channels=channels,
            expires=expires,
            config_fd=config_fd,
            save=save,
        )

    # Continue if agreement and namespace conditions are met.
    _check_dev_agreement_and_namespace_statuses(store)


def login(
    *,
    store: storeapi.StoreClient,
    packages: Iterable[Dict[str, str]] = None,
    save: bool = True,
    acls: Iterable[str] = None,
    channels: Iterable[str] = None,
    expires: str = None,
    config_fd: TextIO = None,
) -> bool:
    if not store:
        store = storeapi.StoreClient()

    email = ""
    password = ""

    if not config_fd:
        echo.wrapped("Enter your Ubuntu One e-mail address and password.")
        echo.wrapped(
            "If you do not have an Ubuntu One account, you can create one "
            "at https://snapcraft.io/account"
        )
        email = echo.prompt("Email")
        if os.getenv("SNAPCRAFT_TEST_INPUT"):
            # Integration tests do not work well with hidden input.
            echo.warning("Password will be visible.")
            hide_input = False
        else:
            hide_input = True
        password = echo.prompt("Password", hide_input=hide_input)

    _try_login(
        email,
        password,
        store=store,
        packages=packages,
        acls=acls,
        channels=channels,
        expires=expires,
        config_fd=config_fd,
        save=save,
    )

    return True


def _login_wrapper(method):
    def login_decorator(self, *args, **kwargs):
        try:
            return method(self, *args, **kwargs)
        except storeapi.http_clients.errors.InvalidCredentialsError:
            print("You are required to login before continuing.")
            login(store=self)
            return method(self, *args, **kwargs)

    return login_decorator


def _register_wrapper(method):
    def register_decorator(self, *args, snap_name: str, **kwargs):
        try:
            return method(self, *args, snap_name=snap_name, **kwargs)
        except storeapi.errors.StoreUploadError as upload_error:
            if "resource-not-found" not in upload_error.error_list:
                raise
            echo.wrapped(
                "You are required to register this snap before continuing. "
                "Refer to 'snapcraft help register' for more options."
            )
            if echo.confirm(
                "Would you like to register {!r} with the Snap Store?".format(snap_name)
            ):
                self.register(snap_name=snap_name)
                return method(self, *args, snap_name=snap_name, **kwargs)
            else:
                raise

    return register_decorator


class StoreClientCLI(storeapi.StoreClient):
    """A CLI friendly StoreClient implementation."""

    # While not complete, the goal of this class is to orchestrate calls
    # to the storeapi and provide a friendly CLI interface around those
    # calls with regards to authentication and authorization, spinners or
    # progressbars and credential configuration.
    #
    # Methods defined here shall also be folded into this class as new
    # features are developed for them, but still provide a simple wrapper
    # method around those methods for backwards compatibility.
    #
    # This class can be thought of and extension to snapcraft.cli.store.
    # It just lives in snapcraft._store due to the convenience of the
    # methods it is trying to replace. Considering this is a private module
    # and this class is not exported, moving it to snapcraft.cli can take
    # place.
    #
    # This is the list of items that needs to be tackled to get to there:
    #
    # TODO create an internal copy of snapcraft.storeapi
    # TODO move configuration loading to this class and out of
    #      snapcraft.storeapi.StoreClient
    # TODO Move progressbar implementation out of snapcraft.storeapi used
    #      during upload into this class using click.
    # TODO use an instance of this class directly from snapcraft.cli.store

    @_login_wrapper
    def close_channels(
        self, *, snap_id: str, channel_names: List[str]
    ) -> Dict[str, Any]:
        return super().close_channels(snap_id=snap_id, channel_names=channel_names)

    @_login_wrapper
    def get_snap_releases(self, *, snap_name: str) -> "Releases":
        return super().get_snap_releases(snap_name=snap_name)

    @_login_wrapper
    def get_snap_channel_map(self, *, snap_name: str) -> "ChannelMap":
        return super().get_snap_channel_map(snap_name=snap_name)

    @_login_wrapper
    def get_account_information(self) -> Dict[str, Any]:
        return super().get_account_information()

    @_login_wrapper
    @_register_wrapper
    def upload_precheck(self, *, snap_name: str) -> None:
        return super().upload_precheck(snap_name=snap_name)

    @_login_wrapper
    @_register_wrapper
    def upload_metadata(
        self, *, snap_name: str, metadata: Dict[str, str], force: bool
    ) -> Dict[str, Any]:
        return super().upload_metadata(
            snap_name=snap_name, metadata=metadata, force=force
        )

    @_login_wrapper
    def register(
        self,
        *,
        snap_name: str,
        is_private: bool = False,
        store_id: Optional[str] = None,
    ) -> None:
        super().register(snap_name=snap_name, is_private=is_private, store_id=store_id)

    @_login_wrapper
    def release(
        self,
        *,
        snap_name: str,
        revision: str,
        channels: List[str],
        progressive_percentage: Optional[int] = None,
    ) -> Dict[str, Any]:
        return super().release(
            snap_name=snap_name,
            revision=revision,
            channels=channels,
            progressive_percentage=progressive_percentage,
        )

    @_login_wrapper
    @_register_wrapper
    def upload(
        self,
        *,
        snap_name: str,
        snap_filename: str,
        built_at: Optional[str] = None,
        channels: Optional[List[str]] = None,
        delta_format: Optional[str] = None,
        source_hash: Optional[str] = None,
        target_hash: Optional[str] = None,
        delta_hash: Optional[str] = None,
    ) -> "StatusTracker":
        return super().upload(
            snap_name=snap_name,
            snap_filename=snap_filename,
            built_at=built_at,
            channels=channels,
            delta_format=delta_format,
            source_hash=source_hash,
            target_hash=target_hash,
            delta_hash=delta_hash,
        )


def list_registered():
    account_info = StoreClientCLI().get_account_information()
    snaps = [
        (
            name,
            info["since"],
            "private" if info["private"] else "public",
            info["price"] or "-",
            "-",
        )
        for name, info in account_info["snaps"].get(DEFAULT_SERIES, {}).items()
        # Presenting only approved snap registrations, which means name
        # disputes will be displayed/sorted some other way.
        if info["status"] == "Approved"
    ]

    if not snaps:
        echo.warning("There are no registered snaps.")
        return

    tabulated_snaps = tabulate(
        sorted(snaps, key=operator.itemgetter(0)),
        headers=["Name", "Since", "Visibility", "Price", "Notes"],
        tablefmt="plain",
    )
    print(tabulated_snaps)


def _get_usable_keys(name=None):
    snap_path = get_host_tool_path(command_name="snap", package_name="snapd")
    keys = json.loads(
        subprocess.check_output([snap_path, "keys", "--json"], universal_newlines=True)
    )
    if keys is not None:
        for key in keys:
            if name is None or name == key["name"]:
                yield key


def _select_key(keys):
    if len(keys) > 1:
        print("Select a key:")
        print()
        tabulated_keys = tabulate(
            [(i + 1, key["name"], key["sha3-384"]) for i, key in enumerate(keys)],
            headers=["Number", "Name", "SHA3-384 fingerprint"],
            tablefmt="plain",
        )
        print(tabulated_keys)
        print()
        while True:
            try:
                keynum = int(echo.prompt("Key number: ")) - 1
            except ValueError:
                continue
            if keynum >= 0 and keynum < len(keys):
                return keys[keynum]
    else:
        return keys[0]


def _export_key(name, account_id):
    return subprocess.check_output(
        ["snap", "export-key", "--account={}".format(account_id), name],
        universal_newlines=True,
    )


def list_keys():
    keys = list(_get_usable_keys())
    account_info = StoreClientCLI().get_account_information()
    enabled_keys = {
        account_key["public-key-sha3-384"]
        for account_key in account_info["account_keys"]
    }
    if keys and enabled_keys:
        tabulated_keys = tabulate(
            [
                (
                    "*" if key["sha3-384"] in enabled_keys else "-",
                    key["name"],
                    key["sha3-384"],
                    "" if key["sha3-384"] in enabled_keys else "(not registered)",
                )
                for key in keys
            ],
            headers=["", "Name", "SHA3-384 fingerprint", ""],
            tablefmt="plain",
        )
        print(tabulated_keys)
    elif not keys and enabled_keys:
        registered_keys = "\n".join([f"- {key}" for key in enabled_keys])
        print(
            "No keys have been created on this system. "
            " See 'snapcraft create-key --help' to create a key.\n"
            "The following SHA3-384 key fingerprints have been registered "
            f"but are not available on this system:\n{registered_keys}"
        )
    else:
        print(
            "No keys have been registered."
            " See 'snapcraft register-key --help' to register a key."
        )


def create_key(name):
    if not name:
        name = "default"
    keys = list(_get_usable_keys(name=name))
    if keys:
        # `snap create-key` would eventually fail, but we can save the user
        # some time in this obvious error case by not bothering to talk to
        # the store first.
        raise storeapi.errors.KeyAlreadyRegisteredError(name)
    try:
        account_info = StoreClientCLI().get_account_information()
        enabled_names = {
            account_key["name"] for account_key in account_info["account_keys"]
        }
    except storeapi.http_clients.errors.InvalidCredentialsError:
        # Don't require a login here; if they don't have valid credentials,
        # then they probably also don't have a key registered with the store
        # yet.
        enabled_names = set()
    if name in enabled_names:
        raise storeapi.errors.KeyAlreadyRegisteredError(name)
    subprocess.check_call(["snap", "create-key", name])


def _maybe_prompt_for_key(name):
    keys = list(_get_usable_keys(name=name))
    if not keys:
        if name is not None:
            raise storeapi.errors.NoSuchKeyError(name)
        else:
            raise storeapi.errors.NoKeysError
    return _select_key(keys)


def register_key(name, use_candid: bool = False) -> None:
    key = _maybe_prompt_for_key(name)
    store_client = StoreClientCLI(use_candid=use_candid)

    # TODO: remove coupling.
    if isinstance(store_client.auth_client, storeapi.http_clients.CandidClient):
        store_client.login(acls=["modify_account_key"], save=False)
    else:
        login(store=store_client, acls=["modify_account_key"], save=False)

    logger.info("Registering key ...")
    account_info = store_client.get_account_information()
    account_key_request = _export_key(key["name"], account_info["account_id"])
    store_client.register_key(account_key_request)
    logger.info(
        'Done. The key "{}" ({}) may be used to sign your assertions.'.format(
            key["name"], key["sha3-384"]
        )
    )


def register(snap_name: str, is_private: bool = False, store_id: str = None) -> None:
    logger.info("Registering {}.".format(snap_name))
    StoreClientCLI().register(
        snap_name=snap_name, is_private=is_private, store_id=store_id
    )


def _generate_snap_build(authority_id, snap_id, grade, key_name, snap_filename):
    """Return the signed snap-build declaration for a snap on disk."""
    snap_path = get_host_tool_path(command_name="snap", package_name="snapd")
    cmd = [
        snap_path,
        "sign-build",
        "--developer-id=" + authority_id,
        "--snap-id=" + snap_id,
        "--grade=" + grade,
    ]
    if key_name:
        cmd.extend(["-k", key_name])
    cmd.append(snap_filename)
    try:
        return subprocess.check_output(cmd)
    except subprocess.CalledProcessError as e:
        raise storeapi.errors.SignBuildAssertionError(snap_filename) from e


def sign_build(snap_filename, key_name=None, local=False):
    if not os.path.exists(snap_filename):
        raise FileNotFoundError("The file {!r} does not exist.".format(snap_filename))

    snap_yaml = _get_data_from_snap_file(snap_filename)
    snap_name = snap_yaml["name"]
    grade = snap_yaml.get("grade", "stable")

    store_client = StoreClientCLI()
    account_info = store_client.get_account_information()

    try:
        authority_id = account_info["account_id"]
        snap_id = account_info["snaps"][DEFAULT_SERIES][snap_name]["snap-id"]
    except KeyError as e:
        raise storeapi.errors.StoreBuildAssertionPermissionError(
            snap_name, DEFAULT_SERIES
        ) from e

    snap_build_path = snap_filename + "-build"
    if os.path.isfile(snap_build_path):
        logger.info("A signed build assertion for this snap already exists.")
        with open(snap_build_path, "rb") as fd:
            snap_build_content = fd.read()
    else:
        key = _maybe_prompt_for_key(key_name)
        if not local:
            is_registered = [
                a
                for a in account_info["account_keys"]
                if a["public-key-sha3-384"] == key["sha3-384"]
            ]
            if not is_registered:
                raise storeapi.errors.KeyNotRegisteredError(key["name"])
        snap_build_content = _generate_snap_build(
            authority_id, snap_id, grade, key["name"], snap_filename
        )
        with open(snap_build_path, "w+") as fd:
            fd.write(snap_build_content.decode())
        logger.info("Build assertion {} saved to disk.".format(snap_build_path))

    if not local:
        store_client.push_snap_build(snap_id, snap_build_content.decode())
        logger.info("Build assertion {} uploaded to the Store.".format(snap_build_path))


def upload_metadata(snap_filename, force):
    """Upload only the metadata to the server.

    If force=True it will force the local metadata into the Store,
    ignoring any possible conflict.
    """
    logger.debug("Uploading metadata to the Store (force=%s)", force)

    # get the metadata from the snap
    snap_yaml = _get_data_from_snap_file(snap_filename)
    metadata = {
        "summary": snap_yaml["summary"],
        "description": snap_yaml["description"],
    }

    # followed by the non mandatory keys
    if "license" in snap_yaml:
        metadata["license"] = snap_yaml["license"]

    if "title" in snap_yaml:
        metadata["title"] = snap_yaml["title"]

    # other snap info
    snap_name = snap_yaml["name"]

    # hit the server
    store_client = StoreClientCLI()
    store_client.upload_precheck(snap_name=snap_name)
    store_client.upload_metadata(snap_name=snap_name, metadata=metadata, force=force)
    with _get_icon_from_snap_file(snap_filename) as icon:
        metadata = {"icon": icon}
        store_client.upload_binary_metadata(snap_name, metadata, force)

    logger.info("The metadata has been uploaded")


def upload(snap_filename, release_channels=None) -> Tuple[str, int]:
    """Upload a snap_filename to the store.

    If a cached snap is available, a delta will be generated from
    the cached snap to the new target snap and uploaded instead. In the
    case of a delta processing or upload failure, upload will fall back to
    uploading the full snap.

    If release_channels is defined it also releases it to those channels if the
    store deems the uploaded snap as ready to release.
    """
    snap_yaml = _get_data_from_snap_file(snap_filename)
    snap_name = snap_yaml["name"]
    built_at = snap_yaml.get("snapcraft-started-at")

    logger.debug(
        "Run upload precheck and verify cached data for {!r}.".format(snap_filename)
    )
    store_client = StoreClientCLI()
    store_client.upload_precheck(snap_name=snap_name)

    snap_cache = cache.SnapCache(project_name=snap_name)

    try:
        deb_arch = snap_yaml["architectures"][0]
    except KeyError:
        deb_arch = "all"

    source_snap = snap_cache.get(deb_arch=deb_arch)
    sha3_384_available = hasattr(hashlib, "sha3_384")

    result: Optional[Dict[str, Any]] = None
    if sha3_384_available and source_snap:
        try:
            result = _upload_delta(
                store_client,
                snap_name=snap_name,
                snap_filename=snap_filename,
                source_snap=source_snap,
                built_at=built_at,
                channels=release_channels,
            )
        except storeapi.errors.StoreDeltaApplicationError as e:
            logger.warning(
                "Error generating delta: {}\n"
                "Falling back to uploading full snap...".format(str(e))
            )
        except storeapi.errors.StoreUploadError as upload_error:
            logger.warning(
                "Unable to upload delta to store: {}\n"
                "Falling back to uploading full snap...".format(upload_error.error_list)
            )

    if result is None:
        result = _upload_snap(
            store_client,
            snap_name=snap_name,
            snap_filename=snap_filename,
            built_at=built_at,
            channels=release_channels,
        )

    snap_cache.cache(snap_filename=snap_filename)
    snap_cache.prune(deb_arch=deb_arch, keep_hash=calculate_sha3_384(snap_filename))

    return snap_name, result["revision"]


def _upload_snap(
    store_client,
    *,
    snap_name: str,
    snap_filename: str,
    built_at: str,
    channels: Optional[List[str]],
) -> Dict[str, Any]:
    tracker = store_client.upload(
        snap_name=snap_name,
        snap_filename=snap_filename,
        built_at=built_at,
        channels=channels,
    )
    result = tracker.track()
    tracker.raise_for_code()
    return result


def _upload_delta(
    store_client,
    *,
    snap_name: str,
    snap_filename: str,
    source_snap: str,
    built_at: str,
    channels: Optional[List[str]] = None,
) -> Dict[str, Any]:
    delta_format = "xdelta3"
    logger.debug("Found cached source snap {}.".format(source_snap))
    target_snap = os.path.join(os.getcwd(), snap_filename)

    try:
        xdelta_generator = deltas.XDelta3Generator(
            source_path=source_snap, target_path=target_snap
        )
        delta_filename = xdelta_generator.make_delta()
    except (DeltaGenerationError, DeltaGenerationTooBigError, ToolMissingError) as e:
        raise storeapi.errors.StoreDeltaApplicationError(str(e))

    snap_hashes = {
        "source_hash": calculate_sha3_384(source_snap),
        "target_hash": calculate_sha3_384(target_snap),
        "delta_hash": calculate_sha3_384(delta_filename),
    }

    try:
        logger.debug("Uploading delta {!r}.".format(delta_filename))
        delta_tracker = store_client.upload(
            snap_name=snap_name,
            snap_filename=delta_filename,
            built_at=built_at,
            channels=channels,
            delta_format=delta_format,
            source_hash=snap_hashes["source_hash"],
            target_hash=snap_hashes["target_hash"],
            delta_hash=snap_hashes["delta_hash"],
        )
        result = delta_tracker.track()
        delta_tracker.raise_for_code()
    except storeapi.errors.StoreReviewError as e:
        if e.code == "processing_upload_delta_error":
            raise storeapi.errors.StoreDeltaApplicationError(str(e))
        else:
            raise
    except storeapi.http_clients.errors.StoreServerError as e:
        raise storeapi.errors.StoreUploadError(snap_name, e.response)
    finally:
        if os.path.isfile(delta_filename):
            try:
                os.remove(delta_filename)
            except OSError:
                logger.warning("Unable to remove delta {}.".format(delta_filename))
    return result


def _get_text_for_opened_channels(opened_channels):
    if len(opened_channels) == 1:
        return "The {!r} channel is now open.".format(opened_channels[0])
    else:
        channels = ("{!r}".format(channel) for channel in opened_channels[:-1])
        return "The {} and {!r} channels are now open.".format(
            ", ".join(channels), opened_channels[-1]
        )


def _get_text_for_channel(channel):
    if "progressive" in channel:
        notes = "progressive ({}%)".format(channel["progressive"]["percentage"])
    else:
        notes = "-"

    if channel["info"] == "none":
        channel_text = (channel["channel"], "-", "-", notes, "")
    elif channel["info"] == "tracking":
        channel_text = (channel["channel"], "^", "^", notes, "")
    elif channel["info"] == "specific":
        channel_text = (
            channel["channel"],
            channel["version"],
            channel["revision"],
            notes,
            "",
        )
    elif channel["info"] == "branch":
        channel_text = (
            channel["channel"],
            channel["version"],
            channel["revision"],
            notes,
            channel["expires_at"],
        )
    else:
        logger.error(
            "Unexpected channel info: %r in channel %s",
            channel["info"],
            channel["channel"],
        )
        channel_text = (channel["channel"], "", "", "", "")

    return channel_text


def _tabulated_channel_map_tree(channel_map_tree):
    """Tabulate channel map (LTS Channel channel-maps)"""

    def _format_tree(channel_maps, track):
        arches = []

        for arch, channel_map in sorted(channel_maps.items()):
            arches += [
                (printable_arch,) + _get_text_for_channel(channel)
                for (printable_arch, channel) in zip(
                    [arch] + [""] * len(channel_map), channel_map
                )
            ]

        return [
            (printable_arch,) + printable_track
            for (printable_arch, printable_track) in zip(
                [track] + [""] * len(arches), arches
            )
        ]

    data = []
    for track, track_data in sorted(channel_map_tree.items()):
        channel_maps = {}
        for series, series_data in track_data.items():
            for arch, channel_map in series_data.items():
                channel_maps[arch] = channel_map
        parsed_channels = [channel for channel in _format_tree(channel_maps, track)]
        data += parsed_channels

    have_expiration = any(x[6] for x in data)
    expires_at_header = "Expires at" if have_expiration else ""
    headers = [
        "Track",
        "Arch",
        "Channel",
        "Version",
        "Revision",
        "Notes",
        expires_at_header,
    ]
    return tabulate(data, numalign="left", headers=headers, tablefmt="plain")


def download(
    snap_name,
    *,
    arch: str,
    download_path: str,
    risk: str,
    track: Optional[str] = None,
    except_hash="",
) -> str:
    """Download snap from the store to download_path.
    :param str snap_name: The snap name to download.
    :param str risk: the channel risk get the snap from.
    :param str track: the specific channel track get the snap from.
    :param str download_path: the path to write the downloaded snap to.
    :param str arch: the architecture of the download as a deb arch.
    :param str except_hash: do not download if set to a sha3_384 hash that
                            matches the snap_name to be downloaded.
    :raises storeapi.errors.SHAMismatchErrorRuntimeError:
         If the checksum for the downloaded file does not match the expected
         hash.
    :returns: A sha3_384 of the file that was or would have been downloaded.
    """
    return StoreClientCLI().download(
        snap_name,
        risk=risk,
        track=track,
        download_path=download_path,
        arch=arch,
        except_hash=except_hash,
    )


def status(snap_name, arch):
    status = StoreClientCLI().get_snap_status(snap_name, arch)

    channel_map_tree = status.get("channel_map_tree", {})
    # This does not look good in green so we print instead
    tabulated_status = _tabulated_channel_map_tree(channel_map_tree)
    print(tabulated_status)


def gated(snap_name):
    """Print list of snaps gated by snap_name."""
    store_client = StoreClientCLI()
    account_info = store_client.get_account_information()
    # Get data for the gating snap
    snaps = account_info.get("snaps", {})

    # Resolve name to snap-id
    try:
        snap_id = snaps[DEFAULT_SERIES][snap_name]["snap-id"]
    except KeyError:
        raise storeapi.errors.SnapNotFoundError(snap_name=snap_name)

    validations = store_client.get_assertion(snap_id, endpoint="validations")

    if validations:
        table_data = []
        for v in validations:
            name = v["approved-snap-name"]
            revision = v["approved-snap-revision"]
            if revision == "-":
                revision = None
            required = str(v.get("required", True))
            # Currently timestamps have microseconds, which look bad
            timestamp = v["timestamp"]
            if "." in timestamp:
                timestamp = timestamp.split(".")[0] + "Z"
            table_data.append([name, revision, required, timestamp])
        tabulated = tabulate(
            table_data,
            headers=["Name", "Revision", "Required", "Approved"],
            tablefmt="plain",
            missingval="-",
        )
        print(tabulated)
    else:
        print("There are no validations for snap {!r}".format(snap_name))


def validate(
    snap_name: str,
    validations: List[str],
    revoke: bool = False,
    key: Optional[str] = None,
):
    """Generate, sign and upload validation assertions."""
    # Check validations format
    _check_validations(validations)

    # Need the ID of the logged in user.
    store_client = StoreClientCLI()
    account_info = store_client.get_account_information()
    authority_id = account_info["account_id"]

    # Get data for the gating snap
    try:
        snap_id = account_info["snaps"][DEFAULT_SERIES][snap_name]["snap-id"]
    except KeyError:
        raise storeapi.errors.SnapNotFoundError(snap_name=snap_name)

    existing_validations = {
        (i["approved-snap-id"], i["approved-snap-revision"]): i
        for i in store_client.get_assertion(
            snap_id, endpoint="validations", params={"include_revoked": "1"}
        )
    }

    # Then, for each requested validation, generate assertion
    for validation in validations:
        gated_snap, rev = validation.split("=", 1)
        echo.info(f"Getting details for {gated_snap}")
        # The Info API is not authed, so it cannot see private snaps.
        try:
            approved_data = store_client.snap.get_info(gated_snap)
            approved_snap_id = approved_data.snap_id
        except storeapi.errors.SnapNotFoundError:
            approved_snap_id = gated_snap

        assertion_payload = {
            "type": "validation",
            "authority-id": authority_id,
            "series": DEFAULT_SERIES,
            "snap-id": snap_id,
            "approved-snap-id": approved_snap_id,
            "approved-snap-revision": rev,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "revoked": "true" if revoke else "false",
        }

        # check for existing validation assertions
        existing = existing_validations.get((approved_snap_id, rev))
        if existing:
            previous_revision = int(existing.get("revision", "0"))
            assertion_payload["revision"] = str(previous_revision + 1)

        assertion = _sign_assertion(validation, assertion_payload, key, "validations")

        # Save assertion to a properly named file
        fname = f"{snap_name}-{gated_snap}-r{rev}.assertion"
        with open(fname, "wb") as f:
            f.write(assertion)

        store_client.push_assertion(snap_id, assertion, endpoint="validations")


validation_re = re.compile("^[^=]+=[0-9]+$")


def _check_validations(validations):
    invalids = [v for v in validations if not validation_re.match(v)]
    if invalids:
        raise storeapi.errors.InvalidValidationRequestsError(invalids)


def _sign_assertion(snap_name, assertion, key, endpoint):
    cmdline = ["snap", "sign"]
    if key:
        cmdline += ["-k", key]
    snap_sign = Popen(
        cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    data = json.dumps(assertion).encode("utf8")
    echo.info("Signing {} assertion for {}".format(endpoint, snap_name))
    assertion, err = snap_sign.communicate(input=data)
    if snap_sign.returncode != 0:
        err = err.decode()
        raise storeapi.errors.StoreAssertionError(
            endpoint=endpoint, snap_name=snap_name, error=err
        )

    return assertion
