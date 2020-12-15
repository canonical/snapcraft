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
import os
import textwrap
from unittest import mock

from testtools.matchers import Contains, Equals, HasLength

import snapcraft
from snapcraft.internal import lifecycle, pluginhandler, states, steps
from snapcraft.internal.lifecycle._status_cache import StatusCache

from . import LifecycleTestBase

# TODO refactor this entire module, the current test time is around 15 minutes.


def pull_order(part_name=None):
    events = []

    if not part_name or part_name in ("main", "dependent", "nested-dependent"):
        events.extend([dict(part="main", step=steps.PULL)])

    if not part_name or part_name in ("dependent", "nested-dependent"):
        events.extend(
            [
                dict(part="main", step=steps.BUILD),
                dict(part="main", step=steps.STAGE),
                dict(part="dependent", step=steps.PULL),
            ]
        )

    if not part_name or part_name == "nested-dependent":
        events.extend(
            [
                dict(part="dependent", step=steps.BUILD),
                dict(part="dependent", step=steps.STAGE),
                dict(part="nested-dependent", step=steps.PULL),
            ]
        )

    return events


def build_order(part_name=None):
    events = []

    if not part_name or part_name in ("main", "dependent", "nested-dependent"):
        events.extend(
            [dict(part="main", step=steps.PULL), dict(part="main", step=steps.BUILD)]
        )

    if not part_name or part_name in ("dependent", "nested-dependent"):
        events.extend(
            [
                dict(part="main", step=steps.STAGE),
                dict(part="dependent", step=steps.PULL),
                dict(part="dependent", step=steps.BUILD),
            ]
        )

    if not part_name or part_name == "nested-dependent":
        events.extend(
            [
                dict(part="dependent", step=steps.STAGE),
                dict(part="nested-dependent", step=steps.PULL),
                dict(part="nested-dependent", step=steps.BUILD),
            ]
        )

    return events


def stage_order(part_name=None):
    events = []

    if not part_name or part_name in ("main", "dependent", "nested-dependent"):
        events.extend(
            [
                dict(part="main", step=steps.PULL),
                dict(part="main", step=steps.BUILD),
                dict(part="main", step=steps.STAGE),
            ]
        )

    if not part_name or part_name in ("dependent", "nested-dependent"):
        events.extend(
            [
                dict(part="dependent", step=steps.PULL),
                dict(part="dependent", step=steps.BUILD),
                dict(part="dependent", step=steps.STAGE),
            ]
        )

    if not part_name or part_name == "nested-dependent":
        events.extend(
            [
                dict(part="nested-dependent", step=steps.PULL),
                dict(part="nested-dependent", step=steps.BUILD),
                dict(part="nested-dependent", step=steps.STAGE),
            ]
        )

    return events


def prime_order(part_name=None):
    events = stage_order(part_name)

    if not part_name or part_name in ("main", "dependent", "nested-dependent"):
        events.append(dict(part="main", step=steps.PRIME))

    if not part_name or part_name in ("dependent", "nested-dependent"):
        events.append(dict(part="dependent", step=steps.PRIME))

    if not part_name or part_name == "nested-dependent":
        events.append(dict(part="nested-dependent", step=steps.PRIME))

    return events


class OrderTestBase(LifecycleTestBase):
    def setUp(self):
        super().setUp()

        self.project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  main:
                    plugin: nil
                  dependent:
                    plugin: nil
                    after: [main]
                  nested-dependent:
                    plugin: nil
                    after: [dependent]
                """
            )
        )

        # Set the option to automatically clean dirty/outdated steps
        with snapcraft.config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(
                snapcraft.config.OutdatedStepAction.CLEAN
            )

    def set_attributes(self, kwargs):
        self.__dict__.update(kwargs)

    def get_run_order(self):
        # Let's determine run order by using the timestamp of state files
        actual_order = []
        for part_name in ("main", "dependent", "nested-dependent"):
            state_dir = os.path.join(self.parts_dir, part_name, "state")
            with contextlib.suppress(FileNotFoundError):
                for step_name in os.listdir(state_dir):
                    path = os.path.join(state_dir, step_name)
                    actual_order.append(
                        {
                            "part": part_name,
                            "step": steps.get_step_by_name(step_name),
                            "timestamp": os.stat(path).st_mtime,
                        }
                    )

        # Now sort by timestamp to get real order
        return sorted(actual_order, key=lambda k: k["timestamp"])

    def assert_parts_dirty(self, expected_parts, hint):
        cache = StatusCache(self.project_config)
        dirty_parts = []
        for part in self.project_config.parts.all_parts:
            for step in steps.STEPS:
                if cache.get_dirty_report(part, step):
                    dirty_parts.append(dict(part=part.name, step=step))
        self.assertThat(dirty_parts, HasLength(len(expected_parts)), hint)
        for expected in expected_parts:
            self.assertThat(dirty_parts, Contains(expected), hint)

    def assert_parts_cleaned(self, earlier_parts, current_parts, expected_parts, hint):
        cleaned_parts = []
        for earlier in earlier_parts:
            earlier_part = earlier["part"]
            earlier_step = earlier["step"]
            found = False
            for current in current_parts:
                if earlier_part == current["part"] and earlier_step == current["step"]:
                    found = True
                    break
            if not found:
                cleaned_parts.append(dict(part=earlier_part, step=earlier_step))

        self.assertThat(cleaned_parts, HasLength(len(expected_parts)), hint)
        for expected in expected_parts:
            self.assertThat(cleaned_parts, Contains(expected), hint)

    def assert_run_order_equal(self, actual_order, expected_order, hint):
        self.assertThat(actual_order, HasLength(len(expected_order)), hint)
        for actual, expected in zip(actual_order, expected_order):
            actual_tuple = (actual["part"], actual["step"])
            expected_tuple = (expected["part"], expected["step"])
            self.assertThat(
                actual_tuple,
                Equals(expected_tuple),
                "expected {} {}".format(expected_tuple, hint),
            )


class ReStepOrderTestBase(OrderTestBase):
    def run_test(self):
        lifecycle.execute(
            self.initial_step, self.project_config, part_names=self.initial_parts
        )

        initial_order = self.get_run_order()
        self.assert_run_order_equal(initial_order, self.expected_initial, "(initial)")

        lifecycle.execute(
            self.test_step, self.project_config, part_names=self.test_parts
        )

        current_order = self.get_run_order()
        test_order = [o for o in current_order if o not in initial_order]
        self.assert_run_order_equal(test_order, self.expected_test, "(test)")

        self.assert_parts_cleaned(
            initial_order, current_order, self.expected_test_cleaned, "(cleaned)"
        )

        self.assert_run_order_equal(
            self.get_run_order(), self.expected_final, "(final)"
        )

        self.assert_parts_dirty(self.expected_dirty, "(dirty)")


class MainTest(ReStepOrderTestBase):
    def test_pull_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PULL,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": pull_order("main"),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [],
                "expected_final": pull_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_build_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": build_order("main"),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [dict(part="main", step=steps.BUILD)],
                "expected_final": pull_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": stage_order("main"),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                ],
                "expected_final": pull_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": prime_order("main"),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                ],
                "expected_final": pull_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_pull_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PULL,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": pull_order(),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                ],
                "expected_final": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                ]
                + pull_order("main"),
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                ],
            }
        )

        self.run_test()

    def test_all_build_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": build_order(),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                ],
                "expected_final": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                ]
                + pull_order("main"),
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                ],
            }
        )

        self.run_test()

    def test_all_stage_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": stage_order(),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                ],
                "expected_final": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ]
                + pull_order("main"),
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
            }
        )

        self.run_test()

    def test_all_prime_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.PULL,
                "expected_initial": prime_order(),
                "expected_test": pull_order("main"),
                "expected_test_cleaned": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                ],
                "expected_final": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ]
                + pull_order("main"),
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_build_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.BUILD,
                "expected_initial": build_order("main"),
                "expected_test": [dict(part="main", step=steps.BUILD)],
                "expected_test_cleaned": [],
                "expected_final": build_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.BUILD,
                "expected_initial": stage_order("main"),
                "expected_test": [dict(part="main", step=steps.BUILD)],
                "expected_test_cleaned": [dict(part="main", step=steps.STAGE)],
                "expected_final": build_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.BUILD,
                "expected_initial": prime_order("main"),
                "expected_test": [dict(part="main", step=steps.BUILD)],
                "expected_test_cleaned": [
                    dict(part="main", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                ],
                "expected_final": build_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_build_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.BUILD,
                "expected_initial": build_order(),
                "expected_test": [dict(part="main", step=steps.BUILD)],
                "expected_test_cleaned": [dict(part="main", step=steps.STAGE)],
                "expected_final": [
                    dict(part="main", step=steps.PULL),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="main", step=steps.BUILD),
                ],
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                ],
            }
        )

        self.run_test()

    def test_all_stage_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.BUILD,
                "expected_initial": stage_order(),
                "expected_test": [dict(part="main", step=steps.BUILD)],
                "expected_test_cleaned": [dict(part="main", step=steps.STAGE)],
                "expected_final": [
                    dict(part="main", step=steps.PULL),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.BUILD),
                ],
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
            }
        )

        self.run_test()

    def test_all_prime_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.BUILD,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="main", step=steps.BUILD)],
                "expected_test_cleaned": [
                    dict(part="main", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                ],
                "expected_final": [
                    dict(part="main", step=steps.PULL),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="main", step=steps.BUILD),
                ],
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_stage_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.STAGE,
                "expected_initial": stage_order("main"),
                "expected_test": [dict(part="main", step=steps.STAGE)],
                "expected_test_cleaned": [],
                "expected_final": stage_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.STAGE,
                "expected_initial": prime_order("main"),
                "expected_test": [dict(part="main", step=steps.STAGE)],
                "expected_test_cleaned": [dict(part="main", step=steps.PRIME)],
                "expected_final": stage_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_stage_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.STAGE,
                "expected_initial": stage_order(),
                "expected_test": [dict(part="main", step=steps.STAGE)],
                "expected_test_cleaned": [],
                "expected_final": build_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.STAGE),
                ],
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
            }
        )

        self.run_test()

    def test_all_prime_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.STAGE,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="main", step=steps.STAGE)],
                "expected_test_cleaned": [dict(part="main", step=steps.PRIME)],
                "expected_final": build_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="main", step=steps.STAGE),
                ],
                "expected_dirty": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_prime_reprime(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["main"],
                "test_parts": ["main"],
                "test_step": steps.PRIME,
                "expected_initial": prime_order("main"),
                "expected_test": [dict(part="main", step=steps.PRIME)],
                "expected_test_cleaned": [],
                "expected_final": prime_order("main"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_prime_reprime(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["main"],
                "test_step": steps.PRIME,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="main", step=steps.PRIME)],
                "expected_test_cleaned": [],
                "expected_final": stage_order()
                + [
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="main", step=steps.PRIME),
                ],
                "expected_dirty": [
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()


class DependentTest(ReStepOrderTestBase):
    def test_pull_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PULL,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": pull_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [],
                "expected_final": pull_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_build_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": build_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [dict(part="dependent", step=steps.BUILD)],
                "expected_final": pull_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": stage_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_final": pull_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": prime_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order("main")
                + [dict(part="dependent", step=steps.PULL)],
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_pull_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PULL,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": pull_order(),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.PULL),
                ],
                "expected_dirty": [dict(part="nested-dependent", step=steps.PULL)],
            }
        )

        self.run_test()

    def test_all_build_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": build_order(),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.PULL),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                ],
            }
        )

        self.run_test()

    def test_all_stage_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": stage_order(),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PULL),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
            }
        )

        self.run_test()

    def test_all_prime_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.PULL,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="dependent", step=steps.PULL),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_build_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.BUILD,
                "expected_initial": build_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.BUILD)],
                "expected_test_cleaned": [],
                "expected_final": build_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.BUILD,
                "expected_initial": stage_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.BUILD)],
                "expected_test_cleaned": [dict(part="dependent", step=steps.STAGE)],
                "expected_final": build_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.BUILD,
                "expected_initial": prime_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.BUILD)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                ],
                "expected_final": pull_order("dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.BUILD),
                ],
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_build_rebuild(self):

        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.BUILD,
                "expected_initial": build_order(),
                "expected_test": [dict(part="dependent", step=steps.BUILD)],
                "expected_test_cleaned": [dict(part="dependent", step=steps.STAGE)],
                "expected_final": stage_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.BUILD),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                ],
            }
        )

        self.run_test()

    def test_all_stage_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.BUILD,
                "expected_initial": stage_order(),
                "expected_test": [dict(part="dependent", step=steps.BUILD)],
                "expected_test_cleaned": [dict(part="dependent", step=steps.STAGE)],
                "expected_final": stage_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.BUILD),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
            }
        )

        self.run_test()

    def test_all_prime_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.BUILD,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="dependent", step=steps.BUILD)],
                "expected_test_cleaned": [
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                ],
                "expected_final": pull_order("dependent")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="dependent", step=steps.BUILD),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_stage_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.STAGE,
                "expected_initial": stage_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.STAGE)],
                "expected_test_cleaned": [],
                "expected_final": stage_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.STAGE,
                "expected_initial": prime_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.STAGE)],
                "expected_test_cleaned": [dict(part="dependent", step=steps.PRIME)],
                "expected_final": build_order("dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_stage_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.STAGE,
                "expected_initial": stage_order(),
                "expected_test": [dict(part="dependent", step=steps.STAGE)],
                "expected_test_cleaned": [],
                "expected_final": build_order("dependent")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
            }
        )

        self.run_test()

    def test_all_prime_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.STAGE,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="dependent", step=steps.STAGE)],
                "expected_test_cleaned": [dict(part="dependent", step=steps.PRIME)],
                "expected_final": build_order("dependent")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="dependent", step=steps.STAGE),
                ],
                "expected_dirty": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_prime_reprime(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["dependent"],
                "test_parts": ["dependent"],
                "test_step": steps.PRIME,
                "expected_initial": prime_order("dependent"),
                "expected_test": [dict(part="dependent", step=steps.PRIME)],
                "expected_test_cleaned": [],
                "expected_final": prime_order("dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_all_prime_reprime(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": [],
                "test_parts": ["dependent"],
                "test_step": steps.PRIME,
                "expected_initial": prime_order(),
                "expected_test": [dict(part="dependent", step=steps.PRIME)],
                "expected_test_cleaned": [],
                "expected_final": stage_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                ],
                "expected_dirty": [dict(part="nested-dependent", step=steps.PRIME)],
            }
        ),

        self.run_test()


class NestedDependentTest(ReStepOrderTestBase):
    def test_pull_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PULL,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.PULL,
                "expected_initial": pull_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.PULL)],
                "expected_test_cleaned": [],
                "expected_final": pull_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_build_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.PULL,
                "expected_initial": build_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="nested-dependent", step=steps.BUILD)
                ],
                "expected_final": pull_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.PULL,
                "expected_initial": stage_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
                "expected_final": pull_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_repull(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.PULL,
                "expected_initial": prime_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.PULL)],
                "expected_test_cleaned": [
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order("dependent")
                + [dict(part="nested-dependent", step=steps.PULL)],
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_build_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.BUILD,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.BUILD,
                "expected_initial": build_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.BUILD)],
                "expected_test_cleaned": [],
                "expected_final": build_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.BUILD,
                "expected_initial": stage_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.BUILD)],
                "expected_test_cleaned": [
                    dict(part="nested-dependent", step=steps.STAGE)
                ],
                "expected_final": build_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_rebuild(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.BUILD,
                "expected_initial": prime_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.BUILD)],
                "expected_test_cleaned": [
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": pull_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.BUILD),
                ],
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_stage_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.STAGE,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.STAGE,
                "expected_initial": stage_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.STAGE)],
                "expected_test_cleaned": [],
                "expected_final": stage_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_restage(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.STAGE,
                "expected_initial": prime_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.STAGE)],
                "expected_test_cleaned": [
                    dict(part="nested-dependent", step=steps.PRIME)
                ],
                "expected_final": build_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.STAGE),
                ],
                "expected_dirty": [],
            }
        )

        self.run_test()

    def test_prime_reprime(self):
        self.set_attributes(
            {
                "initial_step": steps.PRIME,
                "initial_parts": ["nested-dependent"],
                "test_parts": ["nested-dependent"],
                "test_step": steps.PRIME,
                "expected_initial": prime_order("nested-dependent"),
                "expected_test": [dict(part="nested-dependent", step=steps.PRIME)],
                "expected_test_cleaned": [],
                "expected_final": prime_order("nested-dependent"),
                "expected_dirty": [],
            }
        )

        self.run_test()


class DirtyOutdatedTestBase(OrderTestBase):
    def setUp(self):
        super().setUp()

        for directory in ("main", "dependent", "nested-dependent"):
            os.mkdir(directory)
        self.project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  main:
                    plugin: dump
                    source: main/
                  dependent:
                    plugin: dump
                    source: dependent/
                    after: [main]
                  nested-dependent:
                    plugin: dump
                    source: nested-dependent/
                    after: [dependent]
                """
            )
        )

        self.dirty_parts = []
        dirty_parts = self.dirty_parts

        original_dirty_report = pluginhandler.PluginHandler.get_dirty_report

        def _fake_dirty_report(self, step):
            nonlocal dirty_parts
            if dict(part=self.name, step=step) in dirty_parts:
                return pluginhandler.DirtyReport(dirty_properties={"forced"})
            return original_dirty_report(self, step)

        patcher = mock.patch.object(
            pluginhandler.PluginHandler, "get_dirty_report", _fake_dirty_report
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        original_clean = pluginhandler.PluginHandler.clean

        def _fake_clean(self, staged_state, primed_state, step):
            nonlocal dirty_parts
            original_clean(self, staged_state, primed_state, step)
            with contextlib.suppress(ValueError):
                dirty_parts.remove(dict(part=self.name, step=step))

        patcher = mock.patch.object(pluginhandler.PluginHandler, "clean", _fake_clean)
        patcher.start()
        self.addCleanup(patcher.stop)

    def make_outdated(self, part_name, step):
        if step == steps.PULL:
            open(os.path.join(part_name, "file"), "w").close()
        else:
            part = self.project_config.parts.get_part(part_name)
            state_file_path = states.get_step_state_file(part.plugin.statedir, step)
            open(state_file_path, "w").close()

    def run_test(self):
        # Prime all parts
        lifecycle.execute(steps.PRIME, self.project_config)

        initial_order = self.get_run_order()
        self.assert_run_order_equal(initial_order, self.expected_initial, "(initial)")

        if self.test_type == "dirty":
            # Make some parts dirty
            self.dirty_parts.extend(self.test_parts)
        else:
            # Make some parts outdated
            for part in self.test_parts:
                self.make_outdated(part["part"], part["step"])

        # Prime again
        lifecycle.execute(steps.PRIME, self.project_config)
        current_order = self.get_run_order()
        test_order = [o for o in current_order if o not in initial_order]
        self.assert_run_order_equal(test_order, self.expected_test, "(test)")

        self.assert_parts_cleaned(initial_order, current_order, [], "(cleaned)")

        self.assert_run_order_equal(
            self.get_run_order(), self.expected_final, "(final)"
        )

        self.assert_parts_dirty([], "(dirty)")


class DirtyTest(DirtyOutdatedTestBase):
    test_type = "dirty"

    def test_main_pull_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.PULL)],
                "expected_initial": prime_order(),
                "expected_test": prime_order(),
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_main_build_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.BUILD)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_main_stage_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.STAGE)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="main", step=steps.STAGE),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_test_cleaned": [],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_main_prime_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.PRIME)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_test_cleaned": [],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def dependent_pull_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.PULL)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_dependent_build_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.BUILD)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_dependent_stage_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.STAGE)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )
        self.run_test()

    def test_dependent_prime_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.PRIME)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_nested_dependent_pull_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.PULL)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order("dependent")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_nested_dependendent_build_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.BUILD)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": pull_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_nested_dependent_stage_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.STAGE)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": build_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_nested_dependent_prime_dirty(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.PRIME)],
                "expected_initial": prime_order(),
                "expected_test": [dict(part="nested-dependent", step=steps.PRIME)],
                "expected_final": prime_order("nested-dependent"),
            }
        )

        self.run_test()


class OutdatedTest(DirtyOutdatedTestBase):
    test_type = "outdated"

    def test_main_pull_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.PULL)],
                "expected_initial": prime_order(),
                "expected_test": prime_order(),
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_main_build_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.BUILD)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="main", step=steps.BUILD),
                    dict(part="main", step=steps.STAGE),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_main_stage_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.STAGE)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="main", step=steps.STAGE),
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_test_cleaned": [],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_main_prime_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="main", step=steps.PRIME)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_test_cleaned": [],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def dependent_pull_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.PULL)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_dependent_build_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.BUILD)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_dependent_stage_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.STAGE)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": stage_order("main")
                + [
                    dict(part="dependent", step=steps.PULL),
                    dict(part="dependent", step=steps.BUILD),
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )
        self.run_test()

    def test_dependent_prime_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="dependent", step=steps.PRIME)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order(),
            }
        )

        self.run_test()

    def test_nested_dependent_pull_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.PULL)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": prime_order("dependent")
                + [
                    dict(part="nested-dependent", step=steps.PULL),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_nested_dependendent_build_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.BUILD)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": pull_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.BUILD),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_nested_dependent_stage_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.STAGE)],
                "expected_initial": prime_order(),
                "expected_test": [
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
                "expected_final": build_order("nested-dependent")
                + [
                    dict(part="main", step=steps.PRIME),
                    dict(part="dependent", step=steps.PRIME),
                    dict(part="nested-dependent", step=steps.STAGE),
                    dict(part="nested-dependent", step=steps.PRIME),
                ],
            }
        )

        self.run_test()

    def test_nested_dependent_prime_outdated(self):
        self.set_attributes(
            {
                "test_parts": [dict(part="nested-dependent", step=steps.PRIME)],
                "expected_initial": prime_order(),
                "expected_test": [dict(part="nested-dependent", step=steps.PRIME)],
                "expected_final": prime_order("nested-dependent"),
            }
        )

        self.run_test()
