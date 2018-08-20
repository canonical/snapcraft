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

import yaml
from typing import Any, Dict, TextIO

try:
    # The C-based loaders/dumpers aren't available everywhere, but they're much faster,
    # so use them if possible.
    from yaml import (
        CLoader as Loader,
        CSafeLoader as SafeLoader,
        CDumper as Dumper,
        CSafeDumper as SafeDumper,
    )
except ImportError:
    from yaml import Loader, SafeLoader, Dumper, SafeDumper


def load(file_object: TextIO) -> Dict[str, Any]:
    return yaml.load(file_object, Loader=Loader)


def safe_load(file_object: TextIO) -> Dict[str, Any]:
    return yaml.load(file_object, Loader=SafeLoader)


def dump(data: Dict[str, Any], *, stream: TextIO = None) -> str:
    return yaml.dump(data, stream=stream, Dumper=SafeDumper, default_flow_style=False)


def safe_dump(data: Dict[str, Any], *, stream: TextIO = None) -> str:
    return yaml.dump(data, stream=stream, Dumper=Dumper, default_flow_style=False)
