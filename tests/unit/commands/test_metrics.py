# Copyright 2026 Canonical Ltd.
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

"""Tests for the metrics command."""

import argparse
import textwrap
from typing import Any
from unittest.mock import Mock

import pytest
from craft_cli.pytest_plugin import RecordingEmitter
from pytest_mock import MockerFixture

from snapcraft.commands import StoreMetricsCommand
from snapcraft.models import Metric, MetricName, MetricsResponse, Series


@pytest.fixture(autouse=True)
def fake_store_client(mocker: MockerFixture) -> Mock:
    return mocker.patch(
        "snapcraft.store.StoreClientCLI.get_account_info",
        autospec=True,
        return_value={
            "snaps": {"16": {"fakesnap": {"snap-id": "test-snap-id"}}},
        },
    )


@pytest.fixture(autouse=True)
def fake_metrics(mocker: MockerFixture) -> Mock:
    fake_response = MetricsResponse(
        metrics=[
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.DAILY_DEVICE_CHANGE,
                buckets=["2026-01-01", "2026-01-02", "2026-01-03"],
                series=[
                    Series(name="continued", values=[10, 11, 12]),
                    Series(name="lost", values=[1, 2, 3]),
                    Series(name="new", values=[2, 3, 4]),
                ],
            )
        ]
    )
    return mocker.patch(
        "snapcraft.store.StoreClientCLI.get_metrics",
        autospec=True,
        return_value=fake_response,
    )


@pytest.fixture
def metrics_command(fake_app_config: dict[str, Any]) -> StoreMetricsCommand:
    return StoreMetricsCommand(fake_app_config)


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        pytest.param(
            "json",
            textwrap.dedent(
                """\
                {
                  "buckets": [
                    "2026-01-01",
                    "2026-01-02",
                    "2026-01-03"
                  ],
                  "metric_name": "daily_device_change",
                  "series": [
                    {
                      "name": "continued",
                      "values": [
                        10,
                        11,
                        12
                      ]
                    },
                    {
                      "name": "lost",
                      "values": [
                        1,
                        2,
                        3
                      ]
                    },
                    {
                      "name": "new",
                      "values": [
                        2,
                        3,
                        4
                      ]
                    }
                  ],
                  "snap_id": "test-snap-id",
                  "status": "OK"
                }"""
            ),
            id="json",
        ),
        pytest.param(
            "table",
            textwrap.dedent(
                """\
                   Devices    2026-01-01  2026-01-02  2026-01-03
                   Continued  10          11          12
                   Lost       1           2           3
                   New        2           3           4"""
            ),
            id="table",
        ),
    ],
)
def test_metrics_json(
    metrics_command: StoreMetricsCommand,
    emitter: RecordingEmitter,
    mode: str,
    expected: str,
) -> None:
    namespace = argparse.Namespace(
        snap_name="fakesnap",
        metric="daily_device_change",
        format=mode,
        start="2026-01-01",
        end="2026-01-03",
    )

    metrics_command.run(namespace)

    emitter.assert_message(expected)
