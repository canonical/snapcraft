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

from snapcraft import formatting_utils
from snapcraft.internal import steps


class OutdatedReport:
    """The OutdatedReport class explains why a given step is outdated.

    An outdated step is defined to be a step that has run, but since doing so
    one of the following things have happened:

    - A step earlier in the lifecycle has run again.

    - The source on disk has been updated.
    """

    def __init__(
        self, *, previous_step_modified: steps.Step = None, source_updated: bool = False
    ) -> None:
        """Create a new OutdatedReport.

        :param steps.step previous_step_modified: Step earlier in the lifecycle
                                                  that has changed.
        :param bool source_updated: Whether or not the source changed on disk.
        """
        self.previous_step_modified = previous_step_modified
        self.source_updated = source_updated

    def get_report(self) -> str:
        """Get verbose report.

        :return: Report why the step is outdated.
        :rtype: str
        """
        messages = []

        if self.previous_step_modified:
            messages.append(
                "The {!r} step has run more recently.\n".format(
                    self.previous_step_modified.name
                )
            )

        if self.source_updated:
            messages.append("The source has changed on disk.\n")

        return "".join(messages)

    def get_summary(self) -> str:
        """Get summarized report.

        :return: Short summary of why the step is dirty.
        :rtype: str
        """
        reasons = []

        if self.previous_step_modified:
            reasons.append("{!r} step".format(self.previous_step_modified.name))

        if self.source_updated:
            reasons.append("source")

        return "{} changed".format(formatting_utils.humanize_list(reasons, "and", "{}"))
