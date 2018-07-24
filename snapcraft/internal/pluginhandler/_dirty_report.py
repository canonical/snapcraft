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

from typing import List, Set, Union
from snapcraft import formatting_utils

# Ideally we'd just use Collection from typing, but that wasn't introduced
# until 3.6
StrCollection = Union[List[str], Set[str]]
DependencyCollection = Union[List["Dependency"], Set["Dependency"]]


class Dependency:
    def __init__(self, *, part_name, step):
        self.part_name = part_name
        self.step = step


class DirtyReport:
    """The DirtyReport class explains why a given step is dirty.

    A dirty step is defined to be a step that has run, but since doing so one
    of the following things have happened:

    - One or more YAML properties used by the step (e.g. `stage-packages`) have
      changed.

    - One of more project options (e.g. the `--target-arch` CLI option) have
      changed.

    - One of more of its dependencies have been re-staged.
    """

    def __init__(
        self,
        *,
        dirty_properties: StrCollection = None,
        dirty_project_options: StrCollection = None,
        changed_dependencies: DependencyCollection = None
    ) -> None:
        """Create a new DirtyReport.

        :param list dirty_properties: YAML properties that have changed.
        :param list dirty_project_options: Project options that have changed.
        :param list changed_dependencies: Dependencies that have changed.
        """
        self.dirty_properties = dirty_properties
        self.dirty_project_options = dirty_project_options
        self.changed_dependencies = changed_dependencies

    def get_report(self) -> str:
        """Get verbose report.

        :return: Report why the part is dirty.
        :rtype: str
        """
        messages = []  # type: List[str]

        if self.dirty_properties:
            humanized_properties = formatting_utils.humanize_list(
                self.dirty_properties, "and"
            )
            pluralized_connection = formatting_utils.pluralize(
                self.dirty_properties, "property appears", "properties appear"
            )
            messages.append(
                "The {} part {} to have changed.\n".format(
                    humanized_properties, pluralized_connection
                )
            )

        if self.dirty_project_options:
            humanized_options = formatting_utils.humanize_list(
                self.dirty_project_options, "and"
            )
            pluralized_connection = formatting_utils.pluralize(
                self.dirty_project_options, "option appears", "options appear"
            )
            messages.append(
                "The {} project {} to have changed.\n".format(
                    humanized_options, pluralized_connection
                )
            )

        if self.changed_dependencies:
            dependencies = [d.part_name for d in self.changed_dependencies]
            messages.append(
                "{} changed: {}\n".format(
                    formatting_utils.pluralize(
                        dependencies, "A dependency has", "Some dependencies have"
                    ),
                    formatting_utils.humanize_list(dependencies, "and"),
                )
            )

        return "".join(messages)

    def get_summary(self) -> str:
        """Get summarized report.

        :return: Short summary of why the part is dirty.
        :rtype: str
        """
        reasons = []

        reasons_count = 0
        if self.dirty_properties:
            reasons_count += 1
        if self.dirty_project_options:
            reasons_count += 1
        if self.changed_dependencies:
            reasons_count += 1

        if self.dirty_properties:
            # Be specific only if this is the only reason
            if reasons_count > 1 or len(self.dirty_properties) > 1:
                reasons.append("properties")
            else:
                reasons.append(
                    "{!r} property".format(next(iter(self.dirty_properties)))
                )

        if self.dirty_project_options:
            # Be specific only if this is the only reason
            if reasons_count > 1 or len(self.dirty_project_options) > 1:
                reasons.append("options")
            else:
                reasons.append(
                    "{!r} option".format(next(iter(self.dirty_project_options)))
                )

        if self.changed_dependencies:
            # Be specific only if this is the only reason
            if reasons_count > 1 or len(self.changed_dependencies) > 1:
                reasons.append("dependencies")
            else:
                reasons.append(
                    "{!r}".format(next(iter(self.changed_dependencies)).part_name)
                )

        return "{} changed".format(formatting_utils.humanize_list(reasons, "and", "{}"))
