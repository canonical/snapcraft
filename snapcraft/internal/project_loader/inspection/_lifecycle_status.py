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

from typing import Dict, List

from snapcraft.internal import lifecycle, steps
from snapcraft.internal.project_loader import _config


def lifecycle_status(config: _config.Config) -> List[Dict[str, str]]:
    """Return a list of dicts summarizing the current lifecycle status.

    :param _config.Config config: Config object representing loaded project status.
    :returns: List of dicts summarizing the current lifecycle status (perfect for
              printing via tabulate).
    """
    cache = lifecycle.StatusCache(config)

    summary = []
    for part in config.all_parts:
        part_summary = {"part": part.name}
        for step in steps.STEPS:
            part_summary[step.name] = None
            if cache.has_step_run(part, step):
                part_summary[step.name] = "complete"

            dirty_report = cache.get_dirty_report(part, step)
            if dirty_report:
                part_summary[step.name] = "dirty ({})".format(
                    dirty_report.get_summary()
                )

            outdated_report = cache.get_outdated_report(part, step)
            if outdated_report:
                part_summary[step.name] = "outdated ({})".format(
                    outdated_report.get_summary()
                )
        summary.append(part_summary)

    return summary
