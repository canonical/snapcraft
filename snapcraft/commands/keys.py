# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Commands for managing keys."""

from __future__ import annotations

import datetime
import functools
import json
import os
import subprocess
import textwrap
from typing import TYPE_CHECKING, Any

import craft_store
from craft_application.commands import AppCommand
from craft_cli import emit
from tabulate import tabulate
from typing_extensions import override

from snapcraft import errors, store

if TYPE_CHECKING:
    import argparse
    from collections.abc import Callable, Iterator


class StoreKeysCommand(AppCommand):
    """List local keys and registered keys."""

    name = "keys"
    help_msg = "List the keys available to sign assertions"
    overview = textwrap.dedent(
        """
        List the available keys to sign assertions together with their
        local availability."""
    )

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Lists all snap keys."""
        keys = list(_get_usable_keys())
        client = store.StoreClientCLI()
        account_info = client.get_account_info()
        enabled_keys = [
            account_key["public-key-sha3-384"]
            for account_key in account_info["account_keys"]
        ]

        if keys:
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
            emit.message("The following keys are available on this system:")
            emit.message(tabulated_keys)
        else:
            emit.message(
                "No keys have been created on this system. "
                "See 'snapcraft create-key --help' to create a key."
            )

        if enabled_keys:
            local_hashes = {key["sha3-384"] for key in keys}
            registered_keys = "\n".join(
                f"- {key}" for key in enabled_keys if key not in local_hashes
            )
            if registered_keys:
                emit.message(
                    "The following SHA3-384 key fingerprints have been registered "
                    f"but are not available on this system:\n{registered_keys}"
                )
        else:
            emit.message(
                "No keys have been registered with this account. "
                "See 'snapcraft register-key --help' to register a key."
            )


class StoreListKeysCommand(StoreKeysCommand):
    """Removed command alias for the keys command."""

    name = "list-keys"
    hidden = True

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)


class StoreCreateKeyCommand(AppCommand):
    """Create a new key."""

    name = "create-key"
    help_msg = "Create a key to sign assertions."
    overview = textwrap.dedent(
        """
        Create a key and store it locally. Use the register-key command to register
        it in the store.
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "key_name",
            metavar="key-name",
            nargs="?",
            default="default",
            help="Key used to sign the assertion",
        )

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        keys = list(_get_usable_keys(name=parsed_args.key_name))
        if keys:
            # `snap create-key` would eventually fail, but we can save the user
            # some time in this obvious error case by not bothering to talk to
            # the store first.
            raise store.errors.KeyAlreadyExistsError(parsed_args.key_name)
        try:
            client = store.StoreClientCLI()
            account_info = client.get_account_info()
            enabled_names = {
                account_key["name"] for account_key in account_info["account_keys"]
            }
        except craft_store.errors.StoreServerError as store_error:
            if store_error.response.status_code == 401:
                # Don't require a login here; if they don't have valid credentials,
                # then they probably also don't have a key registered with the store yet.
                enabled_names = set()
            else:
                raise

        if parsed_args.key_name in enabled_names:
            raise store.errors.KeyAlreadyRegisteredError(parsed_args.key_name)
        subprocess.check_call(["snap", "create-key", parsed_args.key_name])


class StoreRegisterKeyCommand(AppCommand):
    """Register a key with the Snap store."""

    name = "register-key"
    help_msg = "Register a key to sign assertions with the Snap Store."
    overview = textwrap.dedent(
        """
        Register a locally-created key with the Snap Store.
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "key_name",
            metavar="key-name",
            help="Key used to sign the assertion",
            nargs="?",
        )

    @staticmethod
    def _save_key(func: Callable) -> Callable:
        """Temporarily unset store credentials.

        Registering a key requires a login with specific ACLs, so store credentials
        are temporarily removed from the environment.
        """

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            credentials = os.getenv(store.constants.ENVIRONMENT_STORE_CREDENTIALS)
            if credentials:
                del os.environ[store.constants.ENVIRONMENT_STORE_CREDENTIALS]
            try:
                return func(*args, **kwargs)
            finally:
                if credentials:
                    os.environ[store.constants.ENVIRONMENT_STORE_CREDENTIALS] = (
                        credentials
                    )

        return wrapper

    @_save_key
    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Register a locally-created key with the Snap Store.

        Prompts for the key if multiple keys exist, then logs in with a
        short-lived token scoped to ``modify_account_key``, exports the key assertion,
        and registers it with the store.
        """
        key = self._maybe_prompt_for_key(parsed_args.key_name)

        client = store.StoreClientCLI(ephemeral=True)
        client.login(
            acls=["modify_account_key"],
            ttl=int(datetime.timedelta(days=1).total_seconds()),
        )

        emit.progress("Registering key ...")
        account_info = client.get_account_info()
        account_key_request = self._export_key(key["name"], account_info["account_id"])
        client.register_key(account_key_request)
        emit.message(
            f"Done. The key {key['name']!r} ({key['sha3-384']!r}) may be used to sign your assertions."
        )

    def _maybe_prompt_for_key(self, name: str | None) -> dict[str, Any]:
        """Return a key, prompting if more than one key exists.

        :param name: The key name supplied by the user.

        :returns: The selected key dict from snapd.

        :raises NoSuchKeyError: If no key with 'name' exists locally.
        :raises NoKeysError: If no keys are present and name is None.
        """
        keys = list(_get_usable_keys(name=name))
        if not keys:
            if name is not None:
                raise store.errors.NoSuchKeyError(name)
            else:
                raise store.errors.NoKeysError()
        return self._select_key(keys)

    @staticmethod
    def _export_key(name: str, account_id: str) -> str:
        """Export a snapd key as an assertion.

        :param name: The local key name to export.
        :param account_id: The account ID for the key.

        :returns: The serialized key assertion string.

        :raises subprocess.CalledProcessError: If the snap command fails.
        """
        return subprocess.check_output(
            ["snap", "export-key", f"--account={account_id}", name],
            universal_newlines=True,
        )

    @staticmethod
    def _select_key(keys: list[dict[str, Any]]) -> dict[str, Any]:
        """Interactively select a key when multiple are available.

        :param keys: A list of key dicts.

        :returns: The selected key.
        """
        if len(keys) > 1:
            emit.progress("Select a key:\n", permanent=True)
            tabulated_keys = tabulate(
                [(i + 1, key["name"], key["sha3-384"]) for i, key in enumerate(keys)],
                headers=["Number", "Name", "SHA3-384 fingerprint"],
                tablefmt="plain",
            )
            emit.progress(f"{tabulated_keys}\n", permanent=True)
            while True:
                try:
                    keynum = int(emit.prompt("Key number: ")) - 1
                except ValueError:
                    continue
                if keynum >= 0 and keynum < len(keys):
                    return keys[keynum]
        else:
            return keys[0]


def _get_usable_keys(name: str | None = None) -> Iterator[dict[str, Any]]:
    """Get keys registered by snapd.

    :param name: Optional key name to filter results.

    :returns: An iterator of dicts representing snap keys.

    :raises subprocess.CalledProcessError: If the snap command fails.
    :raises FileNotFoundError: If the snap executable isn't found.
    :raises json.JSONDecodeError: If snapd returns invalid JSON.
    :raises KeyError: On invalid snapd output.
    :raises TypeError: On invalid snapd output.
    """
    keys = json.loads(
        subprocess.check_output(["snap", "keys", "--json"], universal_newlines=True)
    )
    if keys is not None:
        for key in keys:
            if name is None or name == key["name"]:
                yield key
