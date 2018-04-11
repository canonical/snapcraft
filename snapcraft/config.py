# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import base64
import configparser
import io
import logging
import os
import sys
import urllib.parse
from typing import TextIO

from xdg import BaseDirectory

from snapcraft.storeapi import (
    constants,
    errors,
)

LOCAL_CONFIG_FILENAME = '.snapcraft/snapcraft.cfg'

logger = logging.getLogger(__name__)


class Config(object):
    """Hold configuration options in sections.

    There can be two sections for the sso related credentials: production and
    staging. This is governed by the UBUNTU_SSO_API_ROOT_URL environment
    variable. Other sections are ignored but preserved.

    """

    def __init__(self) -> None:
        self.parser = configparser.ConfigParser()
        self.load()

    def _section_name(self) -> str:
        # The only section we care about is the host from the SSO url
        url = os.environ.get('UBUNTU_SSO_API_ROOT_URL',
                             constants.UBUNTU_SSO_API_ROOT_URL)
        return urllib.parse.urlparse(url).netloc

    def get(self, option_name: str) -> str:
        try:
            return self.parser.get(self._section_name(), option_name)
        except (configparser.NoSectionError,
                configparser.NoOptionError,
                KeyError):
            return None

    def set(self, option_name: str, value: str) -> None:
        section_name = self._section_name()
        if not self.parser.has_section(section_name):
            self.parser.add_section(section_name)
        self.parser.set(section_name, option_name, value)

    def is_empty(self) -> bool:
        # Only check the current section
        section_name = self._section_name()
        if self.parser.has_section(section_name):
            if self.parser.options(section_name):
                return False
        return True

    def load(self, *, config_fd: TextIO = None) -> None:
        config = ''
        if config_fd:
            config = config_fd.read()
        else:
            # Local configurations (per project) are supposed to be static.
            # That's why it's only checked for 'loading' and never written to.
            # Essentially, all authentication-related changes, like
            # login/logout or macaroon-refresh, will not be persisted for the
            # next runs.
            file_path = ''
            if os.path.exists(LOCAL_CONFIG_FILENAME):
                file_path = LOCAL_CONFIG_FILENAME

                # FIXME: We don't know this for sure when loading the config.
                # Need a better separation of concerns.
                logger.warn(
                    'Using local configuration ({!r}), changes will not be '
                    'persisted.'.format(file_path))
            else:
                file_path = BaseDirectory.load_first_config(
                    'snapcraft', 'snapcraft.cfg')
            if file_path and os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    config = f.read()

        if config:
            _load_potentially_base64_config(self.parser, config)

    @staticmethod
    def save_path() -> str:
        return os.path.join(BaseDirectory.save_config_path('snapcraft'),
                            'snapcraft.cfg')

    def save(self, *, config_fd: TextIO=None, encode: bool=False) -> None:
        with io.StringIO() as config_buffer:
            self.parser.write(config_buffer)
            config = config_buffer.getvalue()
            if encode:
                # Encode config using base64
                config = base64.b64encode(
                    config.encode(sys.getfilesystemencoding())).decode(
                        sys.getfilesystemencoding())

            if config_fd:
                config_fd.write(config)
            else:
                with open(self.save_path(), 'w') as f:
                    f.write(config)

    def clear(self) -> None:
        self.parser.remove_section(self._section_name())


def _load_potentially_base64_config(parser, config):
    try:
        parser.read_string(config)
    except configparser.Error as e:
        # The config may be base64-encoded, try decoding it
        try:
            config = base64.b64decode(config).decode(
                sys.getfilesystemencoding())
        except base64.binascii.Error:  # type: ignore
            # It wasn't base64, so use the original error
            raise errors.InvalidLoginConfig(e) from e

        try:
            parser.read_string(config)
        except configparser.Error as e:
            raise errors.InvalidLoginConfig(e) from e
