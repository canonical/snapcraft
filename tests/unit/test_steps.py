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

import pytest

from snapcraft.internal import steps


def test_step_order():
    step = steps.PULL

    step = step.next_step()
    assert step == steps.BUILD

    step = step.next_step()
    assert step == steps.STAGE

    step = step.next_step()
    assert step == steps.PRIME

    assert step.next_step() is None

    step = step.previous_step()
    assert step == steps.STAGE

    step = step.previous_step()
    assert step == steps.BUILD

    step = step.previous_step()
    assert step == steps.PULL

    assert step.previous_step() is None


def test_next_step_handles_none():
    assert steps.next_step(None) == steps.PULL


@pytest.mark.parametrize(
    "step,next_steps",
    [
        (steps.PULL, [steps.BUILD, steps.STAGE, steps.PRIME]),
        (steps.BUILD, [steps.STAGE, steps.PRIME]),
        (steps.STAGE, [steps.PRIME]),
        (steps.PRIME, []),
    ],
)
def test_next_steps(step, next_steps):
    assert step.next_steps() == next_steps


@pytest.mark.parametrize(
    "step,previous_steps",
    [
        (steps.PULL, []),
        (steps.BUILD, [steps.PULL]),
        (steps.STAGE, [steps.PULL, steps.BUILD]),
        (steps.PRIME, [steps.PULL, steps.BUILD, steps.STAGE]),
    ],
)
def test_previous_steps(step, previous_steps):
    assert step.previous_steps() == previous_steps
