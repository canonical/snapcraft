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

import contextlib
from typing import List, Tuple

from snapcraft.internal import pluginhandler, steps
import snapcraft.internal.errors
from . import errors


def latest_step(
    parts: List[pluginhandler.PluginHandler]
) -> Tuple[pluginhandler.PluginHandler, steps.Step, int]:
    """Determine and return the latest step that was run.

    :param list parts: List of all parts.
    :returns: Tuple of (part, step, timestamp).
    """
    latest_part = None
    latest_step = None
    latest_timestamp = 0
    for part in parts:
        with contextlib.suppress(snapcraft.internal.errors.NoLatestStepError):
            step = part.latest_step()
            timestamp = part.step_timestamp(step)
            if latest_timestamp < timestamp:
                latest_part = part
                latest_step = step
                latest_timestamp = timestamp

    if not latest_part:
        raise errors.NoStepsRunError()

    return (latest_part, latest_step, latest_timestamp)
