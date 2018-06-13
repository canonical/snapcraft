
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

import textwrap

from snapcraft.internal import lifecycle, steps
import snapcraft.internal.project_loader._config as _config

from . import BaseLifecycleTestCase


class StatusCacheTestCase(BaseLifecycleTestCase):

    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            textwrap.dedent("""\
                parts:
                  main:
                    plugin: nil
                  dependent:
                    plugin: nil
                    after: [main]
                """))

        self.config = _config.Config()
        self.cache = lifecycle.StatusCache(self.config)

    def test_step_has_run(self):
        # No steps should have run, yet
        main_part = self.config.parts.get_part('main')
        self.assertFalse(self.cache.step_has_run(main_part, steps.PULL))

        # Now run the pull step
        lifecycle.execute(
            steps.PULL, self.project_options, part_names=['main'])

        # Should still have cached that no steps have run
        self.assertFalse(self.cache.step_has_run(main_part, steps.PULL))

        # Now clear that step from the cache, and it should be up-to-date
        self.cache.clear_step(main_part, steps.PULL)
        self.assertTrue(self.cache.step_has_run(main_part, steps.PULL))

    def test_add_step_run(self):
        # No steps should have run, yet
        main_part = self.config.parts.get_part('main')
        self.assertFalse(self.cache.step_has_run(main_part, steps.PULL))

        # Tell the cache that the pull step has run though
        self.cache.add_step_run(main_part, steps.PULL)

        # Now it should think that it has actually run
        self.assertTrue(self.cache.step_has_run(main_part, steps.PULL))

        # Now clear that step from the cache, and it should no longer have ran
        self.cache.clear_step(main_part, steps.PULL)
        self.assertFalse(self.cache.step_has_run(main_part, steps.PULL))

    def test_get_dirty_report(self):
        # No dirty reports should be available, yet
        dependent_part = self.config.parts.get_part('dependent')
        self.assertFalse(self.cache.get_dirty_report(
            dependent_part, steps.PULL))

        # Now run the pull step
        lifecycle.execute(steps.PULL, self.project_options)

        # Re-stage main, which will make dependent dirty
        lifecycle.execute(
            steps.PULL, self.project_options, part_names=['main'])

        # Should still have cached that it's not dirty, though
        self.assertFalse(self.cache.get_dirty_report(
            dependent_part, steps.PULL))

        # Now clear that step from the cache, and it should be up-to-date
        self.cache.clear_step(dependent_part, steps.PULL)
        self.assertTrue(self.cache.get_dirty_report(
            dependent_part, steps.PULL))
