# -*- coding: utf-8 -*-
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

from .common import store_api_call


def get_channels(session, package_name):
    """Get current channels config for package through API."""
    channels_endpoint = 'package-channels/%s/' % package_name
    return store_api_call(channels_endpoint, session=session)


def update_channels(session, package_name, data):
    """Update current channels config for package through API."""
    channels_endpoint = 'package-channels/%s/' % package_name
    result = store_api_call(channels_endpoint, method='POST',
                            data=data, session=session)
    if result['success']:
        result['errors'] = result['data']['errors']
        result['data'] = result['data']['channels']
    return result
