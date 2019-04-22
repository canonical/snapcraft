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

import copy
import logging
import os
import re
from typing import Any, Dict, List  # noqa: F401

from snapcraft.project import Project
from snapcraft.internal import project_loader
from snapcraft.formatting_utils import remove_suffix
from . import errors

logger = logging.getLogger(__name__)


_EXPECTED_SNAP_DIR_PATTERNS = {
    re.compile(r"^snapcraft.yaml$"),
    re.compile(r"^.snapcraft(/state)?$"),
    re.compile(r"^hooks(/.*)?$"),
    re.compile(r"^local(/.*)?$"),
    re.compile(r"^plugins(/.*)?$"),
    re.compile(r"^gui(/.*\.(png|svg|desktop))?$"),
}


def conduct_project_sanity_check(project: Project) -> None:
    """Sanity check the project itself before continuing.

    The checks done here are meant to be light, and not rely on the build environment.
    """
    # The snapcraft.yaml should be valid even without extensions applied
    # This here check is mostly for backwards compatibility with the
    # rest of the code base.
    if project.info is not None:
        project.info.validate_raw_snapcraft()

    snap_dir_path = os.path.join(project._project_dir, "snap")
    if os.path.isdir(snap_dir_path):
        _check_snap_dir(snap_dir_path)


def conduct_environment_sanity_check(
    project: Project, yaml_data: Dict[str, Any], schema: Dict[str, Any]
) -> None:
    """Sanity check the build environment and expanded YAML.

    :param snapcraft.Project project: Project settings
    :param dict yaml_data: Validated YAML, with extensions applied
    """
    # Never modify the YAML data
    yaml_data = copy.deepcopy(yaml_data)

    _verify_command_chain_only_full_adapter(yaml_data, schema)


def conduct_config_sanity_check(project_config) -> None:
    """Sanity check the project configuration.
    """
    _validate_common_id(project_config.data)


def _check_snap_dir(snap_dir_path: str) -> None:
    unexpected_paths = set()
    for root, directories, files in os.walk(snap_dir_path):
        for entry in directories + files:
            path = os.path.relpath(os.path.join(root, entry), snap_dir_path)
            if not _snap_dir_path_expected(path):
                unexpected_paths.add(path)

    if unexpected_paths:
        logger.warn(
            "The snap/ directory is meant specifically for snapcraft, but it contains "
            "the following non-snapcraft-related paths, which is unsupported and will "
            "cause unexpected behavior:"
            "\n- {}\n\n"
            "If you must store these files within the snap/ directory, move them to "
            "snap/local/, which is ignored by snapcraft.".format(
                "\n- ".join(sorted(unexpected_paths))
            )
        )


def _snap_dir_path_expected(path: str) -> bool:
    for pattern in _EXPECTED_SNAP_DIR_PATTERNS:
        if pattern.match(path):
            return True
    return False


def _verify_command_chain_only_full_adapter(
    yaml_data: Dict[str, Any], schema: Dict[str, Any]
) -> None:
    # Determine the default adapter
    app_schema = schema["apps"]["patternProperties"]["^[a-zA-Z0-9](?:-?[a-zA-Z0-9])*$"][
        "properties"
    ]
    default_adapter = app_schema["adapter"]["default"]

    # Loop through all apps
    for app_name, app_definition in yaml_data.get("apps", dict()).items():
        # Verify that, if command-chain is used, the "full" adapter is also used
        if "command-chain" in app_definition:
            adapter_string = app_definition.get("adapter", default_adapter)
            adapter = project_loader.Adapter[adapter_string.upper()]
            if adapter == project_loader.Adapter.LEGACY:
                raise errors.CommandChainWithLegacyAdapterError(app_name)


def _validate_common_id(config_data: Dict[str, Any]) -> None:
    """ Check if common-id in part matches metadata filename

    In appstream the common ID is encoded in the metadata file name, so check for
    consistency and warn the user if names don't match. Example data:
    common-id: org.kde.kbruch.desktop
    parse-info: [usr/share/metainfo/org.kde.kbruch.appdata.xml]
    """
    if "apps" not in config_data or "parts" not in config_data:
        return

    common_ids = _get_common_ids(config_data)
    filenames = _get_metadata_filenames(config_data)

    for common_id in common_ids:
        if remove_suffix(common_id, ".desktop") not in filenames:
            raise errors.CommonIdMismatchError(common_id, common_ids[common_id])


def _get_common_ids(config_data: Dict[str, Any]) -> Dict[str, str]:
    common_ids = {}  # type: Dict[str, str]
    for app in config_data["apps"]:
        common_id = config_data["apps"][app].get("common-id")
        if common_id:
            common_ids[common_id] = app
    return common_ids


def _get_metadata_filenames(config_data: Dict[str, Any]) -> List[str]:
    filenames = []  # type: List[str]
    for part in config_data["parts"]:
        parse_info = config_data["parts"][part].get("parse-info")
        if parse_info:
            for filename in parse_info:
                for suffix in [".metainfo.xml", ".appdata.xml"]:
                    if filename.endswith(suffix):
                        filename = remove_suffix(os.path.basename(filename), suffix)
                        filenames.append(filename)
    return filenames
