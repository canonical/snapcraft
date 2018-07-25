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
import enum
import io
import logging
import os
import sys
import urllib.parse
from typing import TextIO

from xdg import BaseDirectory

from snapcraft.internal.errors import SnapcraftInvalidCLIConfigError
from snapcraft.storeapi import constants, errors

LOCAL_CONFIG_FILENAME = ".snapcraft/snapcraft.cfg"

logger = logging.getLogger(__name__)


@enum.unique
class OutdatedStepAction(enum.Enum):
    # Would like to use enum.auto(), but it's only available in >= 3.6
    ERROR = 1
    CLEAN = 2


class CLIConfig:
    """Hold general options affecting the CLI.

    Setting data takes the convention of set_<section>_<option> and
    getting data takes a similar form get_<section>_<option>.
    """

    def __init__(self, *, read_only: bool = False) -> None:
        """Initialize an instance of CLIConfig.

        :param bool read_only: if set, attempts to set data will fail.
        """
        self.parser = configparser.ConfigParser()
        self.config_path = os.path.join(
            BaseDirectory.save_config_path("snapcraft"), "cli.cfg"
        )
        self._read_only = read_only

    def __enter__(self):
        self.load()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self._read_only:
            self.save()

    def load(self) -> None:
        """Load config parser data.

        :raises SnapcraftInvalidCLIConfigError:
            if the configuration data cannot be loaded from disk.
        """
        try:
            with open(self.config_path) as f:
                config = f.read()
        except FileNotFoundError:
            config = None

        if config:
            try:
                self.parser.read_string(config)
            except configparser.Error as parse_error:
                raise SnapcraftInvalidCLIConfigError(
                    config_file=self.config_path, error=str(parse_error)
                ) from parse_error

    def save(self) -> None:
        """Save config parser data.

        :raises RuntimeError: if trying to save when operating with read_only.
        """
        if self._read_only:
            raise RuntimeError("CLIConfig was instantiated with read_only.")
        # Make sure the directory exists before saving
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)

        with open(self.config_path, "w") as f:
            self.parser.write(f)

    def _get_option(self, section_name: str, option_name: str) -> str:
        try:
            return self.parser.get(section_name, option_name)
        except (configparser.NoSectionError, configparser.NoOptionError, KeyError):
            return None

    def _set_option(self, section_name, option_name: str, value: str) -> None:
        if self._read_only:
            raise RuntimeError("CLIConfig was instantiated with read_only.")

        section_name = section_name
        if not self.parser.has_section(section_name):
            self.parser.add_section(section_name)
        self.parser.set(section_name, option_name, value)

    def set_sentry_send_always(self, value: bool) -> None:
        """Setter to define if errors should automatically be sent to sentry.

        :param bool value: if true, errors should automatically be sent.
        """
        if value:
            string_value = "true"
        else:
            string_value = "false"
        self._set_option("Sentry", "always_send", string_value)

    def get_sentry_send_always(self) -> bool:
        """Getter to define if errors should automatically be sent to sentry.

        :returns: True if errors should automatically be sent.
        :rtype: bool.
        """
        string_value = self._get_option("Sentry", "always_send")
        # Anything but "true" for string_value is considered False.
        return string_value == "true"

    def set_outdated_step_action(self, action: OutdatedStepAction) -> None:
        """Setter to define action to take if outdated step is encountered.

        :param OutdatedStepAction value: The action to take
        """
        self._set_option("Lifecycle", "outdated_step_action", action.name.lower())

    def get_outdated_step_action(self) -> OutdatedStepAction:
        """Getter to define action to take if outdated step is encountered.

        :returns: The action to take
        :rtype: OutdatedStepAction.
        """
        action = self._get_option("Lifecycle", "outdated_step_action")
        if action:
            return OutdatedStepAction[action.upper()]
        else:
            # Error by default
            return OutdatedStepAction.ERROR


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
        url = os.environ.get(
            "UBUNTU_SSO_API_ROOT_URL", constants.UBUNTU_SSO_API_ROOT_URL
        )
        return urllib.parse.urlparse(url).netloc

    def get(self, option_name: str) -> str:
        try:
            return self.parser.get(self._section_name(), option_name)
        except (configparser.NoSectionError, configparser.NoOptionError, KeyError):
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
        config = ""
        if config_fd:
            config = config_fd.read()
        else:
            # Local configurations (per project) are supposed to be static.
            # That's why it's only checked for 'loading' and never written to.
            # Essentially, all authentication-related changes, like
            # login/logout or macaroon-refresh, will not be persisted for the
            # next runs.
            file_path = ""
            if os.path.exists(LOCAL_CONFIG_FILENAME):
                file_path = LOCAL_CONFIG_FILENAME

                # FIXME: We don't know this for sure when loading the config.
                # Need a better separation of concerns.
                logger.warn(
                    "Using local configuration ({!r}), changes will not be "
                    "persisted.".format(file_path)
                )
            else:
                file_path = BaseDirectory.load_first_config(
                    "snapcraft", "snapcraft.cfg"
                )
            if file_path and os.path.exists(file_path):
                with open(file_path, "r") as f:
                    config = f.read()

        if config:
            _load_potentially_base64_config(self.parser, config)

    @staticmethod
    def save_path() -> str:
        return os.path.join(
            BaseDirectory.save_config_path("snapcraft"), "snapcraft.cfg"
        )

    def save(self, *, config_fd: TextIO = None, encode: bool = False) -> None:
        with io.StringIO() as config_buffer:
            self.parser.write(config_buffer)
            config = config_buffer.getvalue()
            if encode:
                # Encode config using base64
                config = base64.b64encode(
                    config.encode(sys.getfilesystemencoding())
                ).decode(sys.getfilesystemencoding())

            if config_fd:
                config_fd.write(config)
            else:
                with open(self.save_path(), "w") as f:
                    f.write(config)

    def clear(self) -> None:
        self.parser.remove_section(self._section_name())


def _load_potentially_base64_config(parser, config):
    try:
        parser.read_string(config)
    except configparser.Error as e:
        # The config may be base64-encoded, try decoding it
        try:
            config = base64.b64decode(config).decode(sys.getfilesystemencoding())
        except base64.binascii.Error:  # type: ignore
            # It wasn't base64, so use the original error
            raise errors.InvalidLoginConfig(e) from e

        try:
            parser.read_string(config)
        except configparser.Error as e:
            raise errors.InvalidLoginConfig(e) from e
