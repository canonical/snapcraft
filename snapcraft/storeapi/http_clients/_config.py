# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd
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

import abc
import base64
import io
import os
import pathlib
from typing import Optional, TextIO

import configparser

from . import errors


class Config(abc.ABC):
    def __init__(self) -> None:
        self.parser = configparser.ConfigParser()
        self.load()

    @abc.abstractmethod
    def _get_section_name(self) -> str:
        """Return section name."""

    @abc.abstractmethod
    def _get_config_path(self) -> pathlib.Path:
        """Return Path to configuration file."""

    def get(
        self, option_name: str, section_name: Optional[str] = None
    ) -> Optional[str]:
        """Return content of section_name/option_name or None if not found."""
        if section_name is None:
            section_name = self._get_section_name()
        try:
            return self.parser.get(section_name, option_name)
        except (configparser.NoSectionError, configparser.NoOptionError, KeyError):
            return None

    def set(
        self, option_name: str, value: str, section_name: Optional[str] = None
    ) -> None:
        """Set value to section_name/option_name."""
        if not section_name:
            section_name = self._get_section_name()
        if not self.parser.has_section(section_name):
            self.parser.add_section(section_name)
        self.parser.set(section_name, option_name, value)

    def is_section_empty(self, section_name: Optional[str] = None) -> bool:
        """Check if section_name is empty."""
        if section_name is None:
            section_name = self._get_section_name()

        if self.parser.has_section(section_name):
            if self.parser.options(section_name):
                return False
        return True

    def _load_potentially_base64_config(self, config_content: str) -> None:
        try:
            self.parser.read_string(config_content)
        except configparser.Error as parser_error:
            # The config may be base64-encoded, try decoding it
            try:
                decoded_config_content = base64.b64decode(config_content).decode()
            except base64.binascii.Error:  # type: ignore
                # It wasn't base64, so use the original error
                raise errors.InvalidLoginConfig(parser_error)

            try:
                self.parser.read_string(decoded_config_content)
            except configparser.Error as parser_error:
                raise errors.InvalidLoginConfig(parser_error)

    def load(self, *, config_fd: TextIO = None) -> None:
        if config_fd is not None:
            config_content = config_fd.read()
        elif self._get_config_path().exists():
            with self._get_config_path().open() as config_file:
                config_content = config_file.read()
        else:
            return

        self._load_potentially_base64_config(config_content)

    def save(self, *, config_fd: Optional[TextIO] = None, encode: bool = False) -> None:
        with io.StringIO() as config_buffer:
            self.parser.write(config_buffer)
            config_content = config_buffer.getvalue()
            if encode:
                config_content = base64.b64encode(config_content.encode()).decode()

            if config_fd:
                print(config_content, file=config_fd)
            else:
                with self._get_config_path().open("w") as config_file:
                    print(config_content, file=config_file)
                    config_file.flush()
                    os.fsync(config_file.fileno())

    def clear(self, section_name: Optional[str] = None) -> None:
        if section_name is None:
            section_name = self._get_section_name()

        self.parser.remove_section(self._get_section_name())
