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

from snapcraft_legacy.internal import lifecycle, steps
from snapcraft_legacy.internal.lifecycle._status_cache import StatusCache

from . import LifecycleTestBase

import pytest

@pytest.mark.slow
class StatusCacheTestCase(LifecycleTestBase):
    def setUp(self):
        super().setUp()

        self.project_config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  main:
                    source: .
                    plugin: dump
                  dependent:
                    plugin: nil
                    after: [main]
                """
            )
        )

        self.cache = StatusCache(self.project_config)

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