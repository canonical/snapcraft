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

import collections
import yaml
from typing import Any, Dict, TextIO, Union

try:
    # The C-based loaders/dumpers aren't available everywhere, but they're much faster.
    # Use them if possible. If not, we could fallback to the normal loader/dumper, but
    # they actually behave differently, so raise an error instead.
    from yaml import CSafeLoader, CSafeDumper  # type: ignore
except ImportError:
    raise RuntimeError("Snapcraft requires PyYAML to be built with libyaml bindings")


def load(stream: TextIO) -> Any:
    """Safely load YAML in ordered manner."""
    return yaml.load(stream, Loader=_SafeOrderedLoader)


def dump(data: Union[Dict[str, Any], yaml.YAMLObject], *, stream: TextIO = None) -> str:
    """Safely dump YAML in ordered manner."""
    return yaml.dump(
        data, stream, _SafeOrderedDumper, default_flow_style=False, allow_unicode=True
    )


class _SafeOrderedLoader(CSafeLoader):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dict_constructor
        )


class _SafeOrderedDumper(CSafeDumper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_representer(str, _str_presenter)
        self.add_representer(collections.OrderedDict, _dict_representer)


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
    value = loader.construct_pairs(node)

    try:
        return collections.OrderedDict(value)
    except TypeError:
        raise yaml.constructor.ConstructorError(
            "while constructing a mapping",
            node.start_mark,
            "found unhashable key",
            node.start_mark,
        )


def _str_presenter(dumper, data):
    if len(data.splitlines()) > 1:  # check for multiline string
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)
