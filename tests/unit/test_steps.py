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

from testtools.matchers import Equals

from snapcraft.internal import steps
from tests import unit


class StepsTestCase(unit.TestCase):
    def test_step_order(self):
        step = steps.PULL
        step = step.next_step()
        self.expectThat(step, Equals(steps.BUILD))
        step = step.next_step()
        self.expectThat(step, Equals(steps.STAGE))
        step = step.next_step()
        self.expectThat(step, Equals(steps.PRIME))
        self.assertIsNone(step.next_step())

        step = step.previous_step()
        self.expectThat(step, Equals(steps.STAGE))
        step = step.previous_step()
        self.expectThat(step, Equals(steps.BUILD))
        step = step.previous_step()
        self.expectThat(step, Equals(steps.PULL))
        self.assertIsNone(step.previous_step())

    def test_next_step_handles_none(self):
        self.assertThat(steps.next_step(None), Equals(steps.PULL))


class NextStepsTestCase(unit.TestCase):

    scenarios = [
        (
            "pull",
            {
                "step": steps.PULL,
                "expected_steps": [steps.BUILD, steps.STAGE, steps.PRIME],
            },
        ),
        ("build", {"step": steps.BUILD, "expected_steps": [steps.STAGE, steps.PRIME]}),
        ("stage", {"step": steps.STAGE, "expected_steps": [steps.PRIME]}),
        ("prime", {"step": steps.PRIME, "expected_steps": []}),
    ]

    def test_next_steps(self):
        self.assertThat(self.step.next_steps(), Equals(self.expected_steps))


class PreviousStepsTestCase(unit.TestCase):

    scenarios = [
        ("pull", {"step": steps.PULL, "expected_steps": []}),
        ("build", {"step": steps.BUILD, "expected_steps": [steps.PULL]}),
        ("stage", {"step": steps.STAGE, "expected_steps": [steps.PULL, steps.BUILD]}),
        (
            "prime",
            {
                "step": steps.PRIME,
                "expected_steps": [steps.PULL, steps.BUILD, steps.STAGE],
            },
        ),
    ]

    def test_previous_steps(self):
        self.assertThat(self.step.previous_steps(), Equals(self.expected_steps))
