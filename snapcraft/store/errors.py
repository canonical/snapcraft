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

"""Snapcraft store error definitions."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any

import craft_cli
from craft_application.util import humanize_list

from snapcraft.errors import SnapcraftError

if TYPE_CHECKING:
    import httpx
    import requests

    from . import channels, status

_FORUM_URL = "https://forum.snapcraft.io/c/store"


class StoreMetadataError(SnapcraftError):
    def __init__(
        self,
        snap_name: str,
        response: requests.Response | httpx.Response,
        metadata: dict[str, Any],
    ):
        try:
            response_json = response.json()
        except (AttributeError, json.decoder.JSONDecodeError) as error:
            craft_cli.emit.debug(f"Failed to decode store response: {error}")
            response_json = {}

        if response.status_code == 404:
            message = (
                "Sorry, updating the information in the store has failed, first run "
                f"`snapcraft register {snap_name}` and then "
                "`snapcraft upload <snap-file>`."
            )
            super().__init__(message)
        elif response.status_code == 409:
            conflicts = [
                (error["extra"]["name"], error)
                for error in response_json.get("error_list", [])
                if error["code"] == "conflict"
            ]
            details: list[str] = []
            for field, err in sorted(conflicts):
                details.extend(
                    (
                        f"Conflict in {field!r} field:",
                        f"    In snapcraft.yaml: {metadata.get(field)!r}",
                        f"    In the Store:      {err['extra']['current']!r}",
                    )
                )
            super().__init__(
                "Metadata not uploaded!",
                details="\n".join(details) if details else None,
                resolution=(
                    "You can repeat the upload-metadata command with "
                    "--force to force the local values into the Store"
                ),
            )
        elif "error_list" in response_json:
            craft_cli.emit.debug(str(response_json))
            message = response_json["error_list"][0]["message"]
            super().__init__(message)
        else:
            craft_cli.emit.debug(str(response_json))
            text = response_json.get("text", "")
            super().__init__(f"Store error {response.status_code!r}: {text!r}")


class SnapNotFoundError(SnapcraftError):
    def __init__(
        self,
        *,
        snap_name: str = "",
        snap_id: str | None = None,
        channel: str | None = None,
        arch: str | None = None,
    ):
        # Defaulting snap_name to "" to support the one case we have that
        # makes use of snap_id.
        if snap_name == "" and snap_id is None:
            raise RuntimeError("Both 'snap_name' and 'snap_id' cannot be None.")

        if snap_id:
            # This is legacy.
            message = f"Cannot find snap with snap_id {snap_id!r}."
        elif channel and arch:
            message = f"Snap {snap_name!r} for architecture {arch!r} was not found on channel {channel!r}."
        elif channel:
            message = f"Snap {snap_name!r} was not found on channel {channel!r}."
        elif arch:
            message = f"Snap {snap_name!r} for architecture {arch!r} was not found."
        else:
            message = f"Snap {snap_name!r} was not found."

        resolution = (
            f"Ensure you have proper access rights for {snap_id or snap_name!r}."
        )
        if channel and arch:
            resolution += "\nAlso ensure the correct channel and architecture was used."
        elif channel:
            resolution += "\nAlso ensure the correct channel was used."
        elif arch:
            resolution += "\nAlso ensure the correct architecture was used."

        super().__init__(message, resolution=resolution)


class NoSnapIdError(SnapcraftError):
    def __init__(self, snap_name: str):
        super().__init__(
            f"Failed to get snap ID for snap {snap_name!r}. This is an error in the store.",
            resolution=(
                f"Please open a new topic in the 'store' category in the forum: {_FORUM_URL}"
            ),
        )


class KeyAlreadyExistsError(SnapcraftError):
    def __init__(self, key_name: str):
        super().__init__(f"The key {key_name!r} already exists.")


class KeyAlreadyRegisteredError(SnapcraftError):
    def __init__(self, key_name: str):
        super().__init__(f"You have already registered a key named {key_name!r}.")


class NoKeysError(SnapcraftError):
    def __init__(self):
        super().__init__(
            "You have no usable keys.\nPlease create at least one key with "
            "`snapcraft create-key` for use with snap."
        )


class NoSuchKeyError(SnapcraftError):
    def __init__(self, key_name: str):
        super().__init__(
            f"You have no usable key named {key_name!r}.\nSee the keys available "
            "in your system with `snapcraft keys`."
        )


class InvalidValidationRequestsError(SnapcraftError):
    def __init__(self, requests: list[str]):
        super().__init__(
            "Invalid validation requests (format must be name=revision): "
            f"{' '.join(requests)}"
        )


class ChannelNotAvailableOnArchError(SnapcraftError):
    def __init__(self, snap_name: str, channel: channels.Channel, arch: str):
        super().__init__(
            f"No releases available for {snap_name!r} on channel {channel!r} "
            f"for architecture {arch!r}.\n"
            "Ensure the selected channel contains released revisions for this architecture."
        )


class InvalidChannelSet(SnapcraftError):
    def __init__(
        self,
        snap_name: str,
        channel: channels.Channel,
        channel_outliers: list[status.SnapStatusChannelDetails],
    ):
        arches = humanize_list([c.arch for c in channel_outliers], "and", "{}")
        super().__init__(
            f"The {channel!r} channel for {snap_name!r} does not form a complete set.\n"
            f"There is no revision released for the following architectures: {arches}.\n"
            "Ensure the selected channel contains released revisions for all architectures."
        )
