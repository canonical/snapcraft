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
    from yaml import CLoader as Loader, CSafeLoader as SafeLoader  # type: ignore
    from yaml import Dumper, SafeDumper  # CDumper garbles custom class tags
except ImportError:
    from yaml import Loader, SafeLoader, Dumper, SafeDumper


def load(stream: TextIO) -> OrderedDict:
    return yaml.load(stream, Loader=_ordered_loader(Loader))


def safe_load(stream: TextIO) -> OrderedDict:
    return yaml.load(stream, Loader=_ordered_loader(SafeLoader))


def dump(data: Union[Dict[str, Any], yaml.YAMLObject], *, stream: TextIO = None) -> str:
    return yaml.dump(data, stream, _ordered_dumper(Dumper), default_flow_style=False)


def safe_dump(
    data: Union[Dict[str, Any], yaml.YAMLObject], *, stream: TextIO = None
) -> str:
    return yaml.dump(
        data, stream, _ordered_dumper(SafeDumper), default_flow_style=False
    )


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


def _ordered_loader(base_loader_class):
    class OrderedLoader(base_loader_class):
        pass

    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dict_constructor
    )

    return OrderedLoader


def _ordered_dumper(base_loader_class):
    class OrderedDumper(base_loader_class):
        pass

    OrderedDumper.add_representer(str, _str_presenter)
    OrderedDumper.add_representer(OrderedDict, _dict_representer)

    return OrderedDumper


class OrderedDumper(Dumper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_representer(str, _str_presenter)
        self.add_representer(OrderedDict, _dict_representer)


class SnapcraftYAMLObject(yaml.YAMLObject):
    yaml_loader = _ordered_loader(Loader)
    yaml_dumper = OrderedDumper
    yaml_tag = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, yaml_tag=self.yaml_tag, **kwargs)

    @classmethod
    def from_yaml(cls, loader, node):
        """
        Convert a representation node to a Python object.
        """
        parameters = loader.construct_mapping(node)
        return cls(**parameters)
