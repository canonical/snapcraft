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

from collections import OrderedDict
import yaml
from typing import Any, Dict, TextIO, Union

try:
    # The C-based loaders/dumpers aren't available everywhere, but they're much faster.
    # Use them if possible.
    from yaml import (  # type: ignore
        CLoader as Loader,
        CSafeLoader as SafeLoader,
        CDumper as Dumper,
        CSafeDumper as SafeDumper,
    )
except ImportError:
    from yaml import Loader, SafeLoader, Dumper, SafeDumper

# Setup yaml module globally
# yaml OrderedDict loading and dumping
# from http://stackoverflow.com/a/21048064 Wed Jun 22 16:05:34 UTC 2016
_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG


def _dict_representer(dumper, data):
    return dumper.represent_dict(data.items())


def _dict_constructor(loader, node):
    # Necessary in order to make yaml merge tags work
    loader.flatten_mapping(node)
    return OrderedDict(loader.construct_pairs(node))


def _str_presenter(dumper, data):
    if len(data.splitlines()) > 1:  # check for multiline string
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


yaml.add_representer(str, _str_presenter)
Dumper.add_representer(str, _str_presenter)
SafeDumper.add_representer(str, _str_presenter)
yaml.add_representer(OrderedDict, _dict_representer)
Dumper.add_representer(OrderedDict, _dict_representer)
SafeDumper.add_representer(OrderedDict, _dict_representer)
yaml.add_constructor(_mapping_tag, _dict_constructor)
Loader.add_constructor(_mapping_tag, _dict_constructor)
SafeLoader.add_constructor(_mapping_tag, _dict_constructor)


def load(file_object: TextIO) -> OrderedDict:
    return yaml.load(file_object, Loader=Loader)


def safe_load(file_object: TextIO) -> OrderedDict:
    return yaml.load(file_object, Loader=SafeLoader)


def dump(data: Union[Dict[str, Any], yaml.YAMLObject], *, stream: TextIO = None) -> str:
    return yaml.dump(data, stream=stream, Dumper=Dumper, default_flow_style=False)


def safe_dump(
    data: Union[Dict[str, Any], yaml.YAMLObject], *, stream: TextIO = None
) -> str:
    return yaml.dump(data, stream=stream, Dumper=SafeDumper, default_flow_style=False)
