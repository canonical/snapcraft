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

import json
import subprocess
import textwrap
from typing import TYPE_CHECKING, Any

import craft_store
from craft_application.commands import AppCommand
from typing_extensions import override

from snapcraft import store

if TYPE_CHECKING:
    import argparse
    from collections.abc import Iterator


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
