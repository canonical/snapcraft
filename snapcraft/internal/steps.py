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

from snapcraft.internal import errors


class Step:
    def __init__(self, name: str) -> None:
        self.name = name
        self.__order = None  # type: int

    @property
    def _order(self) -> int:
        if self.__order is None:
            self.__order = STEPS.index(self)
        return self.__order

    def next_step(self) -> 'Step':
        try:
            return STEPS[self._order+1]
        except IndexError as e:
            raise errors.NoNextStepError(self) from e

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
        return 'snapcraft.internal.steps.Step({!r})'.format(self.name)


# Step names and order are now maintained in a single place: right here. If
# another step is needed, add it here, and insert it into STEPS in the proper
# order.
PULL = Step('pull')
BUILD = Step('build')
STAGE = Step('stage')
PRIME = Step('prime')

STEPS = [PULL, BUILD, STAGE, PRIME]


def next_step(step):
    if step:
        return step.next_step()
    else:
        return STEPS[0]


def steps_required_for(step):
    return [s for s in STEPS if s <= step]


def steps_following(step):
    return [s for s in STEPS if s > step]
