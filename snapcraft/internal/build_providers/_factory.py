# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from typing import Union, Type

from . import errors
from ._multipass import Multipass


def get_provider_for(provider_name: str) -> Union[Type[Multipass]]:
    """Returns a Type that can build with provider_name."""
    if provider_name == 'multipass':
        return Multipass
    else:
        raise errors.ProviderNotSupportedError(provider=provider_name)
