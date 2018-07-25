# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import difflib
import hashlib
import logging
import os
import sys

import requests
import yaml
from xdg import BaseDirectory

from snapcraft.internal.indicators import download_requests_stream
from snapcraft.internal.common import get_terminal_width
from snapcraft.internal import errors


PARTS_URI = "https://parts.snapcraft.io/v1/parts.yaml"
_MATCH_RATIO = 0.6
_HEADER_PART_NAME = "PART NAME"
_HEADER_DESCRIPTION = "DESCRIPTION"

logging.getLogger("urllib3").setLevel(logging.CRITICAL)
logger = logging.getLogger(__name__)


def update():
    _Update().execute()


def define(part_name):
    try:
        remote_part = _RemoteParts().get_part(part_name, full=True)
    except errors.SnapcraftPartMissingError as e:
        raise errors.PartNotInCacheError(part_name=part_name) from e
    print("Maintainer: {!r}".format(remote_part.pop("maintainer")))
    print("Description: {}".format(remote_part.pop("description")))
    print("")
    yaml.dump({part_name: remote_part}, default_flow_style=False, stream=sys.stdout)


def search(part_match):
    header_len = len(_HEADER_PART_NAME)
    matches, part_length = _RemoteParts().matches_for(part_match, header_len)

    terminal_width = get_terminal_width(max_width=None)
    part_length = max(part_length, header_len)
    # <space> + <space> + <description> + ... = 5
    description_space = terminal_width - part_length - 5

    if not matches:
        # apt search does not return error, we probably shouldn't either.
        logger.info(
            "No matches found, try to run `snapcraft update` to "
            "refresh the remote parts cache."
        )
        return

    print(
        "{}  {}".format(_HEADER_PART_NAME.ljust(part_length, " "), _HEADER_DESCRIPTION)
    )
    for part_key in sorted(matches.keys()):
        description = matches[part_key]["description"].split("\n")[0]
        if len(description) > description_space:
            description = "{}...".format(description[0:description_space])
        print("{}  {}".format(part_key.ljust(part_length, " "), description))


def get_remote_parts():
    return _RemoteParts()


class _Base:
    def __init__(self):
        self._parts_uri = os.environ.get("SNAPCRAFT_PARTS_URI", PARTS_URI)
        self.parts_dir = os.path.join(
            BaseDirectory.xdg_data_home,
            "snapcraft",
            hashlib.sha384(
                self._parts_uri.encode(sys.getfilesystemencoding())
            ).hexdigest(),
        )
        os.makedirs(self.parts_dir, exist_ok=True)
        self.parts_yaml = os.path.join(self.parts_dir, "parts.yaml")


class _Update(_Base):
    def __init__(self):
        super().__init__()
        self._headers_yaml = os.path.join(self.parts_dir, "headers.yaml")

    def execute(self):
        headers = self._load_headers()

        try:
            self._request = requests.get(self._parts_uri, stream=True, headers=headers)
        except requests.exceptions.RequestException as e:
            raise errors.RemotePartsUpdateConnectionError(e) from e

        if self._request.status_code == 304:
            logger.info("The parts cache is already up to date.")
            return
        self._request.raise_for_status()

        download_requests_stream(
            self._request, self.parts_yaml, "Downloading parts list"
        )
        self._save_headers()

    def _load_headers(self):
        if not os.path.exists(self._headers_yaml):
            return None

        with open(self._headers_yaml) as headers_file:
            return yaml.safe_load(headers_file)

    def _save_headers(self):
        headers = {"If-Modified-Since": self._request.headers.get("Last-Modified")}

        with open(self._headers_yaml, "w") as headers_file:
            headers_file.write(yaml.dump(headers))


class _RemoteParts(_Base):
    def __init__(self):
        super().__init__()

        if not os.path.exists(self.parts_yaml):
            update()

        with open(self.parts_yaml) as parts_file:
            self._parts = yaml.safe_load(parts_file)

    def get_part(self, part_name, full=False):
        try:
            remote_part = self._parts[part_name].copy()
        except KeyError:
            raise errors.SnapcraftPartMissingError(part_name=part_name)
        if not full:
            for key in ["description", "maintainer"]:
                remote_part.pop(key)
        return remote_part

    def matches_for(self, part_match, max_len=0):
        matcher = difflib.SequenceMatcher(isjunk=None, autojunk=False)
        matcher.set_seq2(part_match)

        matching_parts = {}
        for part_name in self._parts.keys():
            matcher.set_seq1(part_name)
            add_part_name = matcher.ratio() >= _MATCH_RATIO

            if add_part_name or (part_match in part_name):
                matching_parts[part_name] = self._parts[part_name]
                if len(part_name) > max_len:
                    max_len = len(part_name)

        return matching_parts, max_len

    def compose(self, part_name, properties):
        """Return properties composed with the ones from part name in the wiki.
        :param str part_name: The name of the part to query from the wiki
        :param dict properties: The current set of properties
        :return: Part properties from the wiki composed with the properties
                 passed as a parameter.
        :rtype: dict
        :raises KeyError: if part_name is not found in the wiki.
        """
        remote_part = self.get_part(part_name)
        remote_part.update(properties)

        return remote_part
