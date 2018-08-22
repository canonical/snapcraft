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
        CSafeLoader as SafeLoader,
        CSafeDumper as SafeDumper,
    )
except ImportError:
    from yaml import SafeLoader, SafeDumper


def load(stream: TextIO) -> Any:
    """Safely load YAML in ordered manner."""
    return yaml.load(stream, Loader=_SafeOrderedLoader)


def dump(data: Union[Dict[str, Any], yaml.YAMLObject], *, stream: TextIO = None) -> str:
    """Safely dump YAML in ordered manner."""
    return yaml.dump(data, stream, _SafeOrderedDumper, default_flow_style=False)


class _SafeOrderedLoader(SafeLoader):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dict_constructor
        )


class _SafeOrderedDumper(SafeDumper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_representer(str, _str_presenter)
        self.add_representer(OrderedDict, _dict_representer)


class SnapcraftYAMLObject(yaml.YAMLObject):
    yaml_loader = _SafeOrderedLoader
    yaml_dumper = _SafeOrderedDumper

    # We could implement a from_yaml class method here which would force loading to
    # go through constructors, but our previous method of doing that has been broken
    # long enough that it would require a larger change.


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
