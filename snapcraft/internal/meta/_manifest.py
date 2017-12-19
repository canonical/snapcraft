# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import os
import json

from snapcraft.internal import errors
from snapcraft.internal.states import (
    get_global_state,
    get_state
)


def annotate_snapcraft(data, parts_dir: str):
    image_info = os.environ.get('SNAPCRAFT_IMAGE_INFO')
    if image_info:
        try:
            image_info_dict = json.loads(image_info)
        except json.decoder.JSONDecodeError as exception:
            raise errors.InvalidContainerImageInfoError(
                image_info) from exception
        data['image-info'] = image_info_dict
    for field in ('build-packages', 'build-snaps'):
        data[field] = get_global_state().assets.get(field, [])
    for part in data['parts']:
        state_dir = os.path.join(parts_dir, part, 'state')
        pull_state = get_state(state_dir, 'pull')
        data['parts'][part]['build-packages'] = (
           pull_state.assets.get('build-packages', []))
        data['parts'][part]['stage-packages'] = (
            pull_state.assets.get('stage-packages', []))
        source_details = pull_state.assets.get('source-details', {})
        if source_details:
            data['parts'][part].update(source_details)
        build_state = get_state(state_dir, 'build')
        data['parts'][part].update(build_state.assets)
    return data
