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

"""
snapcraft login

Authenticates session against Ubuntu One SSO.

Usage:
  login [options]

Options:
  -h --help             show this help message and exit.

"""

import getpass
import logging

from docopt import docopt

from snapcraft.config import save_config
from snapcraft.storeapi import login


logger = logging.getLogger(__name__)


def main(argv=None, project_options=None):
    """Authenticates session against Ubuntu One SSO."""
    argv = argv if argv else []
    docopt(__doc__, argv=argv)

    print('Enter your Ubuntu One SSO credentials.')
    email = input('Email: ')
    password = getpass.getpass('Password: ')
    otp = input('One-time password (just press enter if you don\'t use '
                'two-factor authentication): ')

    logger.info('Authenticating against Ubuntu One SSO.')
    response = login(email, password, token_name='snapcraft', otp=otp)
    success = response.get('success', False)

    if success:
        save_config(response['body'])
        logger.info('Login successful.')
    else:
        logger.info('Login failed.')
