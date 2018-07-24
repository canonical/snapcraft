
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

import os
import textwrap

from snapcraft.internal import lifecycle, states, steps
from snapcraft.internal.lifecycle._status_cache import StatusCache

from . import LifecycleTestBase


class StatusCacheTestCase(LifecycleTestBase):
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
                """
            )
        )

        self.cache = StatusCache(self.project_config)

    def test_has_step_run(self):
        # No steps should have run, yet
        main_part = self.project_config.parts.get_part("main")
        self.assertFalse(self.cache.has_step_run(main_part, steps.PULL))

        # Now run the pull step
        lifecycle.execute(steps.PULL, self.project_config, part_names=["main"])

        # Should still have cached that no steps have run
        self.assertFalse(self.cache.has_step_run(main_part, steps.PULL))

        # Now clear that step from the cache, and it should be up-to-date
        self.cache.clear_step(main_part, steps.PULL)
        self.assertTrue(self.cache.has_step_run(main_part, steps.PULL))

    def test_add_step_run(self):
        # No steps should have run, yet
        main_part = self.project_config.parts.get_part("main")
        self.assertFalse(self.cache.has_step_run(main_part, steps.PULL))

        # Tell the cache that the pull step has run though
        self.cache.add_step_run(main_part, steps.PULL)

        # Now it should think that it has actually run
        self.assertTrue(self.cache.has_step_run(main_part, steps.PULL))

        # Now clear that step from the cache, and it should no longer have ran
        self.cache.clear_step(main_part, steps.PULL)
        self.assertFalse(self.cache.has_step_run(main_part, steps.PULL))

    def test_get_dirty_report(self):
        # No dirty reports should be available, yet
        dependent_part = self.project_config.parts.get_part("dependent")
        self.assertFalse(self.cache.get_dirty_report(dependent_part, steps.PULL))

        # Now run the pull step
        lifecycle.execute(steps.PULL, self.project_config)

        # Re-stage main, which will make dependent dirty
        lifecycle.execute(steps.PULL, self.project_config, part_names=["main"])

        # Should still have cached that it's not dirty, though
        self.assertFalse(self.cache.get_dirty_report(dependent_part, steps.PULL))

        # Now clear that step from the cache, and it should be up-to-date
        self.cache.clear_step(dependent_part, steps.PULL)
        self.assertTrue(self.cache.get_dirty_report(dependent_part, steps.PULL))

    def test_get_outdated_report(self):
        # No outdated reports should be available, yet
        main_part = self.project_config.parts.get_part("main")
        self.assertFalse(self.cache.get_outdated_report(main_part, steps.PULL))

        # Now run the pull step for main
        lifecycle.execute(steps.PULL, self.project_config, part_names=["main"])

        # Change the source on disk, which will make the pull step of main
        # outdated (to ensure this is the case, manually set the timestamp)
        open("new-file", "w").close()
        pull_state_file = states.get_step_state_file(
            main_part.plugin.statedir, steps.PULL
        )
        access_time = os.stat(pull_state_file).st_atime
        modified_time = os.stat(pull_state_file).st_atime
        os.utime("new-file", (access_time, modified_time + 1))

        # Should still have cached that it's not outdated, though
        self.assertFalse(self.cache.get_outdated_report(main_part, steps.PULL))

        # Now clear that step from the cache, and it should be up-to-date
        self.cache.clear_step(main_part, steps.PULL)
        self.assertTrue(self.cache.get_outdated_report(main_part, steps.PULL))
