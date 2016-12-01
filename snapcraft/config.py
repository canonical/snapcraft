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

import configparser
import logging
import os
import urllib.parse

from xdg import BaseDirectory

from snapcraft.storeapi import constants

LOCAL_CONFIG_FILENAME = '.snapcraft/snapcraft.cfg'

logger = logging.getLogger(__name__)


class Config(object):
    """Hold configuration options in sections.

    There can be two sections for the sso related credentials: production and
    staging. This is governed by the UBUNTU_SSO_API_ROOT_URL environment
    variable. Other sections are ignored but preserved.

    """

    def __init__(self):
        self.parser = configparser.ConfigParser()
        self.filename = None
        self.load()

    def _section_name(self):
        # The only section we care about is the host from the SSO url
        url = os.environ.get('UBUNTU_SSO_API_ROOT_URL',
                             constants.UBUNTU_SSO_API_ROOT_URL)
        return urllib.parse.urlparse(url).netloc

    def get(self, option_name):
        try:
            return self.parser.get(self._section_name(), option_name)
        except (configparser.NoSectionError,
                configparser.NoOptionError,
                KeyError):
            return None

    def set(self, option_name, value):
        section_name = self._section_name()
        if not self.parser.has_section(section_name):
            self.parser.add_section(section_name)
        return self.parser.set(section_name, option_name, value)

    def is_empty(self):
        # Only check the current section
        section_name = self._section_name()
        if self.parser.has_section(section_name):
            if self.parser.options(section_name):
                return False
        return True

    def load(self):
        # Local configurations (per project) are supposed to be static.
        # That's why it's only checked for 'loading' and never written to.
        # Essentially, all authentication-related changes, like login/logout
        # or macaroon-refresh, will not be persisted for the next runs.
        if os.path.exists(LOCAL_CONFIG_FILENAME):
            self.parser.read(LOCAL_CONFIG_FILENAME)
            logger.warn(
                'Using local configuration (`{}`), changes will '
                'not be persisted.'.format(LOCAL_CONFIG_FILENAME))
            return

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
