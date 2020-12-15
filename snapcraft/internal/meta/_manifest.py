# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import contextlib
import json
import os
from collections import OrderedDict
from typing import TYPE_CHECKING, Any, Dict, Set

import snapcraft
from snapcraft.internal import errors, os_release, steps
from snapcraft.internal.states import GlobalState, get_state

if TYPE_CHECKING:
    from snapcraft.project import Project


def annotate_snapcraft(project: "Project", data: Dict[str, Any]) -> Dict[str, Any]:
    manifest = OrderedDict()  # type: Dict[str, Any]
    manifest["snapcraft-version"] = snapcraft._get_version()
    manifest["snapcraft-started-at"] = project._get_start_time().isoformat() + "Z"

    release = os_release.OsRelease()
    with contextlib.suppress(errors.OsReleaseIdError):
        manifest["snapcraft-os-release-id"] = release.id()
    with contextlib.suppress(errors.OsReleaseVersionIdError):
        manifest["snapcraft-os-release-version-id"] = release.version_id()

    for k, v in data.items():
        manifest[k] = v
    image_info = os.environ.get("SNAPCRAFT_IMAGE_INFO")
    if image_info:
        try:
            image_info_dict = json.loads(image_info)
        except json.decoder.JSONDecodeError as exception:
            raise errors.InvalidContainerImageInfoError(image_info) from exception
        manifest["image-info"] = image_info_dict

    global_state = GlobalState.load(filepath=project._get_global_state_file_path())
    manifest["build-packages"] = sorted(global_state.get_build_packages())
    manifest["build-snaps"] = sorted(global_state.get_build_snaps())

    for part in data["parts"]:
        state_dir = os.path.join(project.parts_dir, part, "state")
        pull_state = get_state(state_dir, steps.PULL)
        manifest["parts"][part]["build-packages"] = sorted(
            pull_state.assets.get("build-packages", [])
        )
        manifest["parts"][part]["stage-packages"] = sorted(
            pull_state.assets.get("stage-packages", [])
        )
        source_details = pull_state.assets.get("source-details", {})
        if source_details:
            manifest["parts"][part].update(source_details)
        build_state = get_state(state_dir, steps.BUILD)
        manifest["parts"][part].update(build_state.assets)

    # Assemble all primed stage packages into a single list...
    primed_stage_packages: Set[str] = set()
    for part in data["parts"]:
        state_dir = os.path.join(project.parts_dir, part, "state")
        prime_state = get_state(state_dir, steps.PRIME)
        primed_stage_packages |= prime_state.primed_stage_packages
    manifest["primed-stage-packages"] = sorted(primed_stage_packages)

    return manifest
