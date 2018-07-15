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

from typing import List

from snapcraft.internal import errors


class Step:
    def __init__(self, name: str, clean_if_dirty: bool) -> None:
        self.name = name
        self.clean_if_dirty = clean_if_dirty
        self.__order = None  # type: int

    @property
    def _order(self) -> int:
        if self.__order is None:
            try:
                self.__order = STEPS.index(self)
            except ValueError:
                raise errors.InvalidStepError(self.name)
        return self.__order

    def previous_step(self) -> "Step":
        if self._order > 0:
            return STEPS[self._order - 1]
        else:
            return None

    def next_step(self) -> "Step":
        try:
            return STEPS[self._order + 1]
        except IndexError:
            return None

    def previous_steps(self) -> List["Step"]:
        return STEPS[: self._order]

    def next_steps(self) -> List["Step"]:
        return STEPS[self._order + 1 :]

    def __lt__(self, other) -> bool:
        if type(other) is type(self):
            return self._order < other._order

        return NotImplemented

    def __le__(self, other) -> bool:
        if type(other) is type(self):
            return self._order <= other._order

        return NotImplemented

    def __eq__(self, other) -> bool:
        if type(other) is type(self):
            return self.name == other.name

        return NotImplemented

    def __gt__(self, other) -> bool:
        if type(other) is type(self):
            return self._order > other._order

        return NotImplemented

    def __ge__(self, other) -> bool:
        if type(other) is type(self):
            return self._order >= other._order

        return NotImplemented

    def __hash__(self):
        return hash(self.name)

    def __repr__(self):
        return "Step({!r})".format(self.name)


# Step names and order are now maintained in a single place: right here. If
# another step is needed, add it here, and insert it into STEPS in the proper
# order.
PULL = Step("pull", False)
BUILD = Step("build", False)
STAGE = Step("stage", True)
PRIME = Step("prime", True)

STEPS = [PULL, BUILD, STAGE, PRIME]


def next_step(step):
    """Get the next step of the lifecycle

    :param Step step: The current step. If None, the next step is the first
                      step in the lifecycle.
    :return: The next step in the lifecycle
    :rtype: Step
    """
    if step:
        return step.next_step()
    else:
        return STEPS[0]


def get_step_by_name(step_name):
    """Get the lifecycle step that has the given name.

    :param str step_name: Name of the step in question.
    :return: The Step in the lifecycle that has the given name.
    :rtype: Step
    :raises: errors.InvalidStepError if there is no step with the given name.
    """
    if step_name:
        for step in STEPS:
            if step.name == step_name:
                return step
        raise errors.InvalidStepError(step_name)
    else:
        return STEPS[0]


def get_dependency_prerequisite_step(step):
    if step <= STAGE:
        return STAGE
    else:
        return step


def dirty_step_if_dependency_changes(changed_step):
    if changed_step <= STAGE:
        return STEPS[0]
    else:
        return changed_step
