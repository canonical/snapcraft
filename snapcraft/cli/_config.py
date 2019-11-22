# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import logging
import os

import click
import click_config_file
from typing import Dict

from snapcraft import yaml_utils
from . import echo


logger = logging.getLogger(__name__)


def _get_global_config_path() -> str:
    file_name = "config.yaml"
    return os.path.join(click.get_app_dir("snapcraft"), file_name)


def _convert_key_dashes_to_underscores(config: Dict[str, str]):
    # Convert keys from dashes to underscores to match click behavior.
    converted_config = dict()
    for k, v in config.items():
        k_mod = k.replace("-", "_")
        converted_config[k_mod] = v
    return converted_config


def _load_config(file_path: str) -> Dict[str, str]:
    try:
        config = yaml_utils.load_yaml_file(file_path)
    except FileNotFoundError:
        return dict()
    else:
        echo.info(f"Using config file: {file_path}")
        echo.warning("Configuration file is experimental and is subject to change.")
        return _convert_key_dashes_to_underscores(config)


def _click_load_config(file_path: str, cmd_name: str) -> Dict[str, str]:
    """Wrapper for click-config-file."""
    return _load_config(file_path)


def enable_snapcraft_config_file():
    def _add_config_file_support(func):
        config_file_path = _get_global_config_path()
        option = click_config_file.configuration_option(
            implicit=True,
            provider=_click_load_config,
            config_file_name=config_file_path,
        )
        return option(func)

    return _add_config_file_support
