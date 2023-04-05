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

"""Snapcraft Store Account management commands."""

import contextlib
import functools
import os
import pathlib
import stat
import textwrap
from datetime import datetime
from typing import TYPE_CHECKING, Dict, Union

from craft_cli import BaseCommand, emit
from craft_cli.errors import ArgumentParsingError
from overrides import overrides

from snapcraft import store, utils

if TYPE_CHECKING:
    import argparse


_VALID_DATE_FORMATS = [
    "%Y-%m-%d",
    "%Y-%m-%dT%H:%M:%SZ",
]


def _read_config(config_path) -> str:
    if config_path == "-":
        config_path = "/dev/stdin"

    config_file = pathlib.Path(config_path)

    if not config_file.exists():
        raise ArgumentParsingError(f"<login-file> {config_path!r} does not exist")

    return config_file.read_text(encoding="utf-8")


class StoreLoginCommand(BaseCommand):
    """Command to log in to the Snap Store."""

    name = "login"
    help_msg = "Log in to the Snap Store"
    overview = textwrap.dedent(
        f"""
        Log in to the Snap Store with your Ubuntu One SSO credentials.
        If you do not have any, you can create them at https://login.ubuntu.com

        To use the alternative authentication mechanism (Candid), set the
        environment variable {store.constants.ENVIRONMENT_STORE_AUTH!r} to 'candid'.

        The login command requires a working keyring on the system it is used on.
        As an alternative, export {store.constants.ENVIRONMENT_STORE_CREDENTIALS!r}
        with the exported credentials.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        """Add arguments specific to the export-login command."""
        parser.add_argument(
            "--with",
            metavar="<login-file>",
            dest="login_with",
            type=str,
            default=None,
            help="File to use for imported credentials",
        )
        parser.add_argument(
            "--experimental-login",
            action="store_true",
            default=False,
            help=(
                "Deprecated option to enable candid login. "
                f"Set {store.constants.ENVIRONMENT_STORE_AUTH}=candid instead"
            ),
        )

    @overrides
    def run(self, parsed_args):
        if parsed_args.experimental_login:
            raise ArgumentParsingError(
                "--experimental-login no longer supported. "
                f"Set {store.constants.ENVIRONMENT_STORE_AUTH}=candid instead",
            )

        if parsed_args.login_with:
            config_content = _read_config(parsed_args.login_with)
            emit.progress(
                "--with is no longer supported, export the auth to the environment "
                f"variable {store.constants.ENVIRONMENT_STORE_CREDENTIALS!r} instead",
                permanent=True,
            )
            store.LegacyUbuntuOne.store_credentials(config_content)
        else:
            store.StoreClientCLI().login()

        emit.message("Login successful")


class StoreExportLoginCommand(BaseCommand):
    """Command to export login to use with the Snap Store."""

    name = "export-login"
    help_msg = "Log in to the Snap Store exporting the credentials"
    overview = textwrap.dedent(
        f"""
        Log in to the Snap Store with your Ubuntu One SSO credentials.
        If you do not have any, you can create them at https://login.ubuntu.com

        To use the alternative authentication mechanism (Candid), set the
        environment variable {store.constants.ENVIRONMENT_STORE_AUTH!r} to 'candid'.

        This command exports credentials to use on systems where login is not
        possible or desired.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        """Add arguments specific to the export-login command."""
        parser.add_argument(
            "login_file",
            metavar="<login-file>",
            type=str,
            help="Where to write the exported credentials, - for stdout",
        )
        parser.add_argument(
            "--snaps",
            metavar="<snaps>",
            type=str,
            nargs="?",
            default=None,
            help="Comma-separated list of snaps to limit access",
        )
        parser.add_argument(
            "--channels",
            metavar="<channels>",
            type=str,
            nargs="?",
            default=None,
            help="Comma-separated list of channels to limit access",
        )
        parser.add_argument(
            "--acls",
            metavar="<acls>",
            type=str,
            nargs="?",
            default=None,
            help="Comma-separated list of ACLs to limit access",
        )
        parser.add_argument(
            "--expires",
            metavar="<expires>",
            type=str,
            nargs="?",
            default=None,
            help="Date/time (in ISO 8601) when this exported login expires",
        )
        parser.add_argument(
            "--experimental-login",
            action="store_true",
            default=False,
            help=(
                "Deprecated option to enable candid login. "
                f"Set {store.constants.ENVIRONMENT_STORE_AUTH}=candid instead"
            ),
        )

    @overrides
    def run(self, parsed_args):
        if parsed_args.experimental_login:
            raise ArgumentParsingError(
                "--experimental-login no longer supported. "
                f"Set {store.constants.ENVIRONMENT_STORE_AUTH}=candid instead",
            )

        kwargs: Dict[str, Union[str, int]] = {}
        if parsed_args.snaps:
            kwargs["packages"] = parsed_args.snaps.split(",")
        if parsed_args.channels:
            kwargs["channels"] = parsed_args.channels.split(",")
        if parsed_args.acls:
            kwargs["acls"] = parsed_args.acls.split(",")
        if parsed_args.expires is not None:
            for date_format in _VALID_DATE_FORMATS:
                with contextlib.suppress(ValueError):
                    expiry_date = datetime.strptime(parsed_args.expires, date_format)
                    break
            else:  # noqa: PLW0120 Else clause on loop without a break statement
                valid_formats = utils.humanize_list(_VALID_DATE_FORMATS, "or")
                raise ArgumentParsingError(
                    f"The expiry follow an ISO 8601 format ({valid_formats})"
                )

            kwargs["ttl"] = int((expiry_date - datetime.now()).total_seconds())

        credentials = store.StoreClientCLI(ephemeral=True).login(**kwargs)

        # Support a login_file of '-', which indicates a desire to print to stdout
        if parsed_args.login_file.strip() == "-":
            message = f"Exported login credentials:\n{credentials}"
        else:
            # This is sensitive-- it should only be accessible by the owner
            private_open = functools.partial(os.open, mode=0o600)

            with open(
                parsed_args.login_file, "w", opener=private_open, encoding="utf-8"
            ) as login_fd:
                print(credentials, file=login_fd, end="")

            # Now that the file has been written, we can just make it
            # owner-readable
            os.chmod(parsed_args.login_file, stat.S_IRUSR)

            message = f"Exported login credentials to {parsed_args.login_file!r}"

        message += "\n\nThese credentials must be used on Snapcraft 7.2 or greater."
        if os.getenv(store.constants.ENVIRONMENT_STORE_AUTH) == "candid":
            message += (
                f"\nSet '{store.constants.ENVIRONMENT_STORE_AUTH}=candid' for "
                "these credentials to work."
            )
        emit.message(message)


class StoreWhoAmICommand(BaseCommand):
    """Command to show login information from Snap Store."""

    name = "whoami"
    help_msg = "Get information about the current login"
    overview = textwrap.dedent(
        """
        Return useful information about the current login.
        """
    )

    @overrides
    def run(self, parsed_args):
        whoami = store.StoreClientCLI().store_client.whoami()

        if whoami.get("permissions"):
            permissions = ", ".join(whoami["permissions"])
        else:
            permissions = "no restrictions"

        if whoami.get("channels"):
            channels = ", ".join(whoami["channels"])
        else:
            channels = "no restrictions"

        account = whoami["account"]

        # onprem store does not have expires
        try:
            expires = f"{whoami['expires']}Z"
        except KeyError:
            expires = "N/A"

        message = textwrap.dedent(
            f"""\
            email: {account["email"]}
            username: {account["username"]}
            id: {account["id"]}
            permissions: {permissions}
            channels: {channels}
            expires: {expires}"""
        )

        emit.message(message)


class StoreLogoutCommand(BaseCommand):
    """Command to log out from the Snap Store."""

    name = "logout"
    help_msg = "Clear Snap Store credentials."
    overview = textwrap.dedent(
        """
        Remove stored Snap Store credentials from the system.
        """
    )

    @overrides
    def run(self, parsed_args):
        store.StoreClientCLI().store_client.logout()
        emit.message("Credentials cleared")
