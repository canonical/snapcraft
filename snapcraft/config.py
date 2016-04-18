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
from __future__ import absolute_import, unicode_literals
import configparser
import os

from xdg import BaseDirectory

from urllib.parse import urlparse

from configparser import ConfigParser
from xdg.BaseDirectory import load_first_config, save_config_path

from snapcraft.storeapi.constants import UBUNTU_SSO_API_ROOT_URL


class Config(object):
    """Hold configuration options in sections.

    There can be two sections for the sso related credentials: production and
    staging. This is gouverned by the UBUNTU_SSO_API_ROOT_URL environment
    variable. Other sections are ignored but preserved.
    """

    def __init__(self):
        self.parser = configparser.ConfigParser()
        self.filename = None

    def _section_name(self):
        url = os.environ.get('UBUNTU_SSO_API_ROOT_URL',
                             UBUNTU_SSO_API_ROOT_URL)
        return urlparse(url).netloc

    def get(self, option_name):
        try:
            return self.parser.get(self._section_name(), option_name)
        except (configparser. NoSectionError, KeyError):
            return None

    def set(self, option_name, value):
        section_name = self._section_name()
        if not self.parser.has_section(section_name):
            self.parser.add_section(section_name)
        return self.parser.set(section_name, option_name, value)

    def load(self):
        self.filename = BaseDirectory.load_first_config(
            'snapcraft', 'snapcraft.cfg')
        if self.filename and os.path.exists(self.filename):
            self.parser.read(self.filename)

    @staticmethod
    def save_path():
        return os.path.join(BaseDirectory.save_config_path('snapcraft'),
                            'snapcraft.cfg')

    def save(self):
        self.filename = self.save_path()
        with open(self.filename, 'w') as f:
            self.parser.write(f)

    def clear(self):
        self.parser.remove_section(self._section_name())


def load_config():

    """Read and return configuration from disk."""
    filename = load_first_config('snapcraft', 'snapcraft.cfg') or ''

    parser = ConfigParser()
    if os.path.exists(filename):
        parser.read(filename)

    api_endpoint = os.environ.get(
        'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
    location = urlparse(api_endpoint).netloc

    config = {}
    if parser.has_section(location):
        config.update(dict(parser.items(location)))
    return config


def save_config(data):
    """Store current configuration to disk."""
    config_dir = save_config_path('snapcraft')
    filename = os.path.join(config_dir, 'snapcraft.cfg')

    parser = ConfigParser()
    if os.path.exists(filename):
        parser.read(filename)

    api_endpoint = os.environ.get(
        'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
    location = urlparse(api_endpoint).netloc
    if not parser.has_section(location):
        parser.add_section(location)

    for key, value in data.items():
        parser.set(location, key, str(value))

    with open(filename, 'w') as fd:
        parser.write(fd)


def clear_config():
    """Remove configuration section from files on disk."""
    config_dir = save_config_path('snapcraft')
    filename = os.path.join(config_dir, 'snapcraft.cfg')

    parser = ConfigParser()
    if os.path.exists(filename):
        parser.read(filename)

    api_endpoint = os.environ.get(
        'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
    location = urlparse(api_endpoint).netloc
    parser.remove_section(location)

    with open(filename, 'w') as fd:
        parser.write(fd)
