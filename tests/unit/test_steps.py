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

from snapcraft.internal import (
    errors,
    steps,
)
from tests import unit


class StepsTestCase(unit.TestCase):

    def test_no_next_step_raises(self):
        raised = self.assertRaises(
            errors.NoNextStepError, steps.next_step, steps.PRIME)
        self.assertThat(raised.step, Equals(steps.PRIME))


class NextStepTestCase(unit.TestCase):

    scenarios = [
        ('pull', {
            'step': steps.PULL,
            'expected_step': steps.BUILD,
        }),
        ('build', {
            'step': steps.BUILD,
            'expected_step': steps.STAGE,
        }),
        ('stage', {
            'step': steps.STAGE,
            'expected_step': steps.PRIME,
        }),
    ]

    def test_steps_required_for(self):
        self.assertThat(steps.next_step(self.step), Equals(self.expected_step))


class StepsRequiredForTestCase(unit.TestCase):

    scenarios = [
        ('pull', {
            'step': steps.PULL,
            'expected_steps': [steps.PULL],
        }),
        ('build', {
            'step': steps.BUILD,
            'expected_steps': [steps.PULL, steps.BUILD],
        }),
        ('stage', {
            'step': steps.STAGE,
            'expected_steps': [steps.PULL, steps.BUILD, steps.STAGE],
        }),
        ('prime', {
            'step': steps.PRIME,
            'expected_steps': [
                steps.PULL, steps.BUILD, steps.STAGE, steps.PRIME],
        }),
    ]

    def test_steps_required_for(self):
        self.assertThat(steps.steps_required_for(
            self.step), Equals(self.expected_steps))


class StepsFollowingTestCase(unit.TestCase):

    scenarios = [
        ('pull', {
            'step': steps.PULL,
            'expected_steps': [steps.BUILD, steps.STAGE, steps.PRIME],
        }),
        ('build', {
            'step': steps.BUILD,
            'expected_steps': [steps.STAGE, steps.PRIME],
        }),
        ('stage', {
            'step': steps.STAGE,
            'expected_steps': [steps.PRIME],
        }),
        ('prime', {
            'step': steps.PRIME,
            'expected_steps': [],
        }),
    ]

    def test_steps_following(self):
        self.assertThat(steps.steps_following(
            self.step), Equals(self.expected_steps))
