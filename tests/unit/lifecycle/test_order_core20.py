# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2022 Canonical Ltd
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

from unittest.mock import call, patch

import pytest

from snapcraft.internal import steps
from snapcraft.internal.lifecycle._runner import _Executor as Executor
from snapcraft.internal.meta.snap import Snap
from snapcraft.project import Project


class FakePart:
    def __init__(self, name: str = "part1") -> None:
        self.name = name

    def should_step_run(self, prerequisite_step):
        return True


@pytest.fixture
def project_config():
    class Parts:
        def get_dependencies(self, part_name: str):
            return [FakePart("dep")]

    class Config:
        def __init__(self):
            self.project = Project()
            self.project._snap_meta = Snap(
                name="project-name", base="core20", version="1.0", confinement="strict"
            )
            self.parts = Parts()

    return Config()


@pytest.fixture
def mock_executor_run():
    patcher = patch.object(Executor, "run")
    yield patcher.start()
    patcher.stop()


def test_pull(project_config, mock_executor_run):
    executor = Executor(project_config)

    executor._handle_part_dependencies(step=steps.PULL, part=FakePart())

    assert mock_executor_run.mock_calls == []


@pytest.mark.parametrize("step", [steps.BUILD, steps.STAGE])
def test_build_stage(step, project_config, mock_executor_run):
    executor = Executor(project_config)

    executor._handle_part_dependencies(step=step, part=FakePart())

    assert mock_executor_run.mock_calls == [call(steps.STAGE, {"dep"})]


def test_prime(project_config, mock_executor_run):
    executor = Executor(project_config)

    executor._handle_part_dependencies(step=steps.PRIME, part=FakePart())

    assert mock_executor_run.mock_calls == [call(steps.PRIME, {"dep"})]
