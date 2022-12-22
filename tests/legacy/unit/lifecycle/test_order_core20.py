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

from typing import Sequence
from unittest.mock import call, patch

import pytest

from snapcraft_legacy.internal import steps
from snapcraft_legacy.internal.lifecycle._runner import _Executor as Executor
from snapcraft_legacy.internal.meta.snap import Snap
from snapcraft_legacy.internal.pluginhandler._build_attributes import BuildAttributes
from snapcraft_legacy.project import Project


class FakePart:
    def __init__(self, name: str = "part1", build_attributes: Sequence = None) -> None:
        self.name = name

        if build_attributes is None:
            build_attributes = ["core22-step-dependencies"]
        self._build_attributes = BuildAttributes(build_attributes)

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


def test_pull_no_build_attribute(project_config, mock_executor_run):
    executor = Executor(project_config)

    executor._handle_part_dependencies(
        step=steps.PULL, part=FakePart(name="part1", build_attributes=[])
    )

    assert mock_executor_run.mock_calls == [call(steps.STAGE, {"dep"})]


@pytest.mark.parametrize("step", [steps.BUILD, steps.STAGE])
def test_build_stage(step, project_config, mock_executor_run):
    executor = Executor(project_config)

    executor._handle_part_dependencies(step=step, part=FakePart())

    assert mock_executor_run.mock_calls == [call(steps.STAGE, {"dep"})]


def test_prime(project_config, mock_executor_run):
    executor = Executor(project_config)

    executor._handle_part_dependencies(step=steps.PRIME, part=FakePart())

    assert mock_executor_run.mock_calls == [call(steps.PRIME, {"dep"})]
