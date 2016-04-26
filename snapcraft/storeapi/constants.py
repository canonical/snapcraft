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


# FIXME: snapcraft targets the '16' series, hardcode it until more choices
# become available server side -- vila 2016-04-22
DEFAULT_SERIES = '16'
SCAN_STATUS_POLL_DELAY = 5
SCAN_STATUS_POLL_RETRIES = 5
UBUNTU_SSO_API_ROOT_URL = 'https://login.ubuntu.com/api/v2/'
UBUNTU_STORE_API_ROOT_URL = 'https://myapps.developer.ubuntu.com/dev/api/'
UBUNTU_STORE_SEARCH_ROOT_URL = 'https://search.apps.ubuntu.com/'
UBUNTU_STORE_UPLOAD_ROOT_URL = 'https://upload.apps.ubuntu.com/'
