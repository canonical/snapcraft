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

from typing import Union

from . import errors  # noqa: F401
from ._candid_client import CandidClient  # noqa: F401
from ._ubuntu_sso_client import UbuntuOneAuthClient  # noqa: F401
from ._http_client import Client  # noqa: F401


AuthClient = Union[CandidClient, UbuntuOneAuthClient]
