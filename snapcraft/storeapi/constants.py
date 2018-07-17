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
UBUNTU_SSO_API_ROOT_URL = "https://login.ubuntu.com/api/v2/"
UBUNTU_STORE_API_ROOT_URL = "https://dashboard.snapcraft.io/dev/api/"
UBUNTU_STORE_SEARCH_ROOT_URL = "https://api.snapcraft.io/"
UBUNTU_STORE_UPLOAD_ROOT_URL = "https://upload.apps.ubuntu.com/"
UBUNTU_STORE_TOS_URL = "https://dashboard.snapcraft.io/dev/tos/"
UBUNTU_STORE_ACCOUNT_URL = "https://dashboard.snapcraft.io/dev/account/"

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
AGREEMENT_INPUT_MSG = "Do you agree to the developer terms and conditions. ({})? [y/N] "
AGREEMENT_SIGN_ERROR = (
    "Unexpected error encountered during signing the developer terms and "
    "conditions. Please visit {} and agree to the terms and conditions before "
    "continuing."
)
TWO_FACTOR_WARNING = (
    "We strongly recommend enabling multi-factor authentication: "
    "https://help.ubuntu.com/community/SSO/FAQs/2FA"
)
INVALID_CREDENTIALS = "Invalid credentials supplied."
AUTHENTICATION_ERROR = "Problems encountered when authenticating your credentials."
ACCOUNT_INFORMATION_ERROR = "Unexpected error when obtaining your account information."
