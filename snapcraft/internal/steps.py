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


class Step:
    def __init__(self, name: str) -> None:
        self.name = name
        self.previous_step = None  # type: Step
        self.next_step = None  # type: Step

    def previous_steps(self):
        steps = []
        previous_step = self.previous_step
        while previous_step:
            steps.insert(0, previous_step)
            previous_step = previous_step.previous_step
        return steps

    def next_steps(self):
        steps = []
        next_step = self.next_step
        while next_step:
            steps.append(next_step)
            next_step = next_step.next_step
        return steps

    def __lt__(self, other) -> bool:
        if type(other) is type(self):
            return other in self.next_steps()

        return NotImplemented

    def __le__(self, other) -> bool:
        if type(other) is type(self):
            return other == self or other in self.next_steps()

        return NotImplemented

    def __eq__(self, other) -> bool:
        if type(other) is type(self):
            return self.name == other.name

        return NotImplemented

    def __gt__(self, other) -> bool:
        if type(other) is type(self):
            return other in self.previous_steps()

        return NotImplemented

    def __ge__(self, other) -> bool:
        if type(other) is type(self):
            return other == self or other in self.previous_steps()

        return NotImplemented

    def __hash__(self):
        return hash(self.name)

    def __repr__(self):
        return 'snapcraft.internal.steps.Step({!r})'.format(self.name)


def _setup_step_order(steps: List[Step]):
    for index, step in enumerate(steps):
        if 0 <= index < len(steps)-1:
            step.next_step = steps[index+1]
        if index > 0:
            step.previous_step = steps[index-1]


# Step names and order are now maintained in a single place: right here. If
# another step is needed, add it here and to the _setup_step_order call in the
# proper order.
PULL = Step('pull')
BUILD = Step('build')
STAGE = Step('stage')
PRIME = Step('prime')

FIRST_STEP = PULL

_setup_step_order([PULL, BUILD, STAGE, PRIME])


def next_step(step):
    if step:
        return step.next_step
    else:
        return FIRST_STEP


def ordered_steps():
    return [FIRST_STEP] + FIRST_STEP.next_steps()
