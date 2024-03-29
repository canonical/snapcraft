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
DEFAULT_SERIES = "16"
SCAN_STATUS_POLL_DELAY = 5
SCAN_STATUS_POLL_RETRIES = 5

STORE_DASHBOARD_URL = "https://dashboard.snapcraft.io"
STORE_API_URL = "https://api.snapcraft.io"
STORE_UPLOAD_URL = "storage.snapcraftcontent.com"

UBUNTU_ONE_SSO_URL = "https://login.ubuntu.com"

# Messages and warnings.
MISSING_AGREEMENT = "Developer has not signed agreement."
MISSING_NAMESPACE = "Developer profile is missing short namespace."
AGREEMENT_ERROR = (
    "You must agree to the developer terms and conditions to upload snaps."
)
NAMESPACE_ERROR = (
    "You need to set a username. It will appear in the developer field "
    "alongside the other details for your snap. Please visit {} and login "
    "again."
)
AGREEMENT_INPUT_MSG = "Do you agree to the developer terms and conditions. ({})?"
AGREEMENT_SIGN_ERROR = (
    "Unexpected error encountered during signing the developer terms and "
    "conditions. Please visit {} and agree to the terms and conditions before "
    "continuing."
)
TWO_FACTOR_WARNING = (
    "We strongly recommend enabling multi-factor authentication: "
    "https://help.ubuntu.com/community/SSO/FAQs/2FA"
)

ENVIRONMENT_STORE_CREDENTIALS = "SNAPCRAFT_STORE_CREDENTIALS"
"""Environment variable where credentials can be picked up from."""

ENVIRONMENT_STORE_AUTH = "SNAPCRAFT_STORE_AUTH"
"""Environment variable used to set an alterntive login method.

The only setting that changes the behavior is `candid`, every
other value uses Ubuntu SSO.
"""
