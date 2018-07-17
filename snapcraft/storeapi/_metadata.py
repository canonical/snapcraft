# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import hashlib
import json
import os

from snapcraft.storeapi.errors import StoreMetadataError


def _media_hash(media_file):
    sha = hashlib.sha256(media_file.read())
    # rewind file before returning
    media_file.seek(0)
    return sha.hexdigest()


class StoreMetadataHandler:
    def __init__(self, store_client, store_auth, snap_id, snap_name):
        super().__init__()
        self.client = store_client
        self.auth = store_auth
        self.snap_id = snap_id
        self.snap_name = snap_name

    def push(self, metadata, force):
        """Push the metadata to SCA."""
        url = "snaps/" + self.snap_id + "/metadata"
        headers = {
            "Authorization": self.auth,
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        method = "PUT" if force else "POST"
        response = self.client.request(
            method, url, data=json.dumps(metadata), headers=headers
        )

        if not response.ok:
            raise StoreMetadataError(self.snap_name, response, metadata)

    def _current_binary_metadata(self):
        """Get current icons and screenshots as set in the store."""
        url = "snaps/" + self.snap_id + "/binary-metadata"
        headers = {"Authorization": self.auth, "Accept": "application/json"}
        # get current binary metadata information
        response = self.client.request("GET", url, headers=headers)
        if not response.ok:
            raise StoreMetadataError(self.snap_name, response, {})

        binary_metadata = response.json()
        # current icons and screenshots
        icons = [media for media in binary_metadata if media.get("type") == "icon"]
        screenshots = [
            media for media in binary_metadata if media.get("type") == "screenshot"
        ]
        return icons, screenshots

    def _build_binary_request_data(self, metadata):
        """Return calculated data and files for the binary request."""
        data = files = None

        icons, screenshots = self._current_binary_metadata()
        current_icon = icons[0] if icons else None

        # only icon support atm
        icon = metadata.get("icon")
        # keep original screenshots
        updated_info = screenshots

        if current_icon is None and icon is None:
            # icon unchanged, nothing to push
            return data, files

        if icon:
            icon_hash = _media_hash(icon)
            if current_icon is None or current_icon.get("hash") != icon_hash:
                upload_icon = {
                    "type": "icon",
                    "hash": icon_hash,
                    "key": "icon",
                    "filename": icon.name,
                }
                updated_info.append(upload_icon)
                files = {"icon": icon}
            else:
                # icon unchanged, nothing to push
                return data, files

        if not files:
            # API requires a multipart request, but we have no files to push
            # https://github.com/requests/requests/issues/1081
            files = {"info": ("", json.dumps(updated_info))}
        else:
            data = {"info": json.dumps(updated_info)}

        return data, files

    def push_binary(self, metadata, force):
        """Push the binary metadata to SCA."""
        data, files = self._build_binary_request_data(metadata)
        if data is None and files is None:
            # nothing to update
            return

        url = "snaps/" + self.snap_id + "/binary-metadata"
        headers = {"Authorization": self.auth, "Accept": "application/json"}
        method = "PUT" if force else "POST"
        response = self.client.request(
            method, url, data=data, files=files, headers=headers
        )
        if not response.ok:
            icon = metadata.get("icon")
            icon_name = os.path.basename(icon.name) if icon else None
            raise StoreMetadataError(self.snap_name, response, {"icon": icon_name})
