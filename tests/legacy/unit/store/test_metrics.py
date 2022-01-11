# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd.
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

import re

import pytest

from snapcraft_legacy.storeapi.metrics import (
    MetricResults,
    MetricsFilter,
    MetricsNames,
    MetricsResults,
    MetricsStatus,
    Series,
)


def test_metrics_filter():
    metrics_filter = MetricsFilter(
        snap_id="test-snap-id",
        metric_name="test-metric-name",
        start="test-start",
        end="test-end",
    )

    assert metrics_filter.marshal() == {
        "snap_id": "test-snap-id",
        "metric_name": "test-metric-name",
        "start": "test-start",
        "end": "test-end",
    }


@pytest.mark.parametrize("values", [[""], ["x"], [1], ["x", 1]])
@pytest.mark.parametrize("currently_released", [False, True, None])
def test_series(values, currently_released):
    series = Series(
        name="test-name", values=values, currently_released=currently_released
    )

    expected_data = {
        "name": "test-name",
        "values": values,
        "currently_released": currently_released,
    }
    if currently_released is None:
        expected_data.pop("currently_released")

    assert series.marshal() == expected_data
    assert Series.unmarshal(expected_data) == series


@pytest.mark.parametrize("name", [False, {1}, 1])
def test_series_invalid_name(name):
    series = {"name": name, "values": []}

    with pytest.raises(ValueError, match=f"Invalid metric name: {name!r}"):
        Series.unmarshal(series)


@pytest.mark.parametrize("values", [False, 1, "x", {}, [{}]])
def test_series_invalid_values(values):
    series = {"name": "test-name", "values": values}

    with pytest.raises(
        ValueError, match=re.escape(f"Invalid metric values: {values!r}")
    ):
        Series.unmarshal(series)


@pytest.mark.parametrize("currently_released", [5, "x", {}, [], [{}]])
def test_series_invalid_currently_released(currently_released):
    series = {
        "name": "test-name",
        "values": [],
        "currently_released": currently_released,
    }

    with pytest.raises(
        ValueError,
        match=re.escape(f"Invalid metric currently_released: {currently_released!r}"),
    ):
        Series.unmarshal(series)


@pytest.mark.parametrize("buckets", [[], ["2021-01-01"], ["2021-01-01", "2021-01-02"]])
@pytest.mark.parametrize(
    "series", [[], [{"name": "s1", "values": []}], [{"name": "s2", "values": ["v1"]}]]
)
@pytest.mark.parametrize("status", ["OK", "FAIL", "NO DATA"])
def test_metric_results(buckets, series, status):
    results = MetricResults(
        status=MetricsStatus[status],
        snap_id="test-id",
        metric_name="test-metric",
        buckets=buckets,
        series=[Series.unmarshal(s) for s in series],
    )

    expected_data = {
        "status": status,
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": buckets,
        "series": series,
    }

    assert results.marshal() == expected_data
    assert MetricResults.unmarshal(expected_data) == results


@pytest.mark.parametrize("status", [False, {1}, 1, "INVALID"])
def test_metric_results_invalid_status(status):
    metric_results = {
        "status": status,
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": [],
        "series": [],
    }

    with pytest.raises(ValueError, match=f"Invalid metric status: {status!r}"):
        MetricResults.unmarshal(metric_results)


@pytest.mark.parametrize("snap_id", [False, {1}, 1])
def test_metric_results_invalid_snap_id(snap_id):
    metric_results = {
        "status": "OK",
        "snap_id": snap_id,
        "metric_name": "test-metric",
        "buckets": [],
        "series": [],
    }

    with pytest.raises(ValueError, match=f"Invalid metric snap id: {snap_id!r}"):
        MetricResults.unmarshal(metric_results)


@pytest.mark.parametrize("metric_name", [None, False, {1}, 1])
def test_metric_results_invalid_metric_name(metric_name):
    metric_results = {
        "status": "OK",
        "snap_id": "test-id",
        "metric_name": metric_name,
        "buckets": [],
        "series": [],
    }

    with pytest.raises(ValueError, match=f"Invalid metric name: {metric_name!r}"):
        MetricResults.unmarshal(metric_results)


@pytest.mark.parametrize("buckets", [None, False, {1}, 1])
def test_metric_results_invalid_buckets(buckets):
    metric_results = {
        "status": "OK",
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": buckets,
        "series": [],
    }

    with pytest.raises(ValueError, match=f"Invalid metric buckets: {buckets!r}"):
        MetricResults.unmarshal(metric_results)


@pytest.mark.parametrize("series", [None, False, {1}, 1])
def test_metric_results_invalid_series(series):
    metric_results = {
        "status": "OK",
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": [],
        "series": series,
    }

    with pytest.raises(ValueError, match=f"Invalid metric series: {series!r}"):
        MetricResults.unmarshal(metric_results)


def test_metrics_results_unmarshal_no_data():
    data = {
        "metrics": [
            {
                "buckets": [],
                "metric_name": "weekly_installed_base_by_channel",
                "series": [],
                "snap_id": "test-snap-id",
                "status": "NO DATA",
            }
        ]
    }

    metrics_results = MetricsResults.unmarshal(data)

    assert metrics_results == MetricsResults(
        metrics=[
            MetricResults(
                status=MetricsStatus["NO DATA"],
                snap_id="test-snap-id",
                metric_name="weekly_installed_base_by_channel",
                buckets=[],
                series=[],
            )
        ]
    )
    assert metrics_results.marshal() == data


@pytest.mark.parametrize("metric_name", [n.value for n in MetricsNames])
def test_metrics_results_unmarshal_one_bucket(metric_name):
    data = {
        "metrics": [
            {
                "buckets": ["2021-01-01"],
                "metric_name": metric_name,
                "series": [
                    {"name": "continued", "values": [11]},
                    {"name": "lost", "values": [3]},
                    {"name": "new", "values": [4]},
                ],
                "snap_id": "test-snap-id",
                "status": "OK",
            }
        ]
    }

    metrics_results = MetricsResults.unmarshal(data)

    assert metrics_results == MetricsResults(
        metrics=[
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name=metric_name,
                buckets=["2021-01-01"],
                series=[
                    Series(name="continued", values=[11], currently_released=None),
                    Series(name="lost", values=[3], currently_released=None),
                    Series(name="new", values=[4], currently_released=None),
                ],
            )
        ]
    )
    assert metrics_results.marshal() == data


def test_metrics_results_unmarshal_two_buckets():
    data = {
        "metrics": [
            {
                "buckets": ["2021-01-01", "2021-01-02"],
                "metric_name": "daily_device_change",
                "series": [
                    {"name": "continued", "values": [11, 15]},
                    {"name": "lost", "values": [3, 0]},
                    {"name": "new", "values": [4, 1]},
                ],
                "snap_id": "test-snap-id",
                "status": "OK",
            }
        ]
    }

    metrics_results = MetricsResults.unmarshal(data)

    assert metrics_results == MetricsResults(
        metrics=[
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="daily_device_change",
                buckets=["2021-01-01", "2021-01-02"],
                series=[
                    Series(name="continued", values=[11, 15], currently_released=None),
                    Series(name="lost", values=[3, 0], currently_released=None),
                    Series(name="new", values=[4, 1], currently_released=None),
                ],
            )
        ]
    )
    assert metrics_results.marshal() == data


def test_metrics_results_unmarshal_with_currently_released():
    data = {
        "metrics": [
            {
                "buckets": ["2021-01-01"],
                "metric_name": "weekly_installed_base_by_channel",
                "series": [
                    {"currently_released": False, "name": "beta", "values": [62]},
                    {"currently_released": True, "name": "edge", "values": [38]},
                ],
                "snap_id": "test-snap-id",
                "status": "OK",
            }
        ]
    }

    metrics_results = MetricsResults.unmarshal(data)

    assert metrics_results == MetricsResults(
        metrics=[
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="weekly_installed_base_by_channel",
                buckets=["2021-01-01"],
                series=[
                    Series(name="beta", values=[62], currently_released=False),
                    Series(name="edge", values=[38], currently_released=True),
                ],
            )
        ]
    )
    assert metrics_results.marshal() == data


def test_metrics_results_unmarshal_multiple_metrics():
    metrics = [
        {
            "buckets": ["2021-01-01"],
            "metric_name": n.value,
            "series": [
                {"name": "blah", "values": [1]},
                {"name": "blahh", "values": [1, 2]},
                {"name": "blahhh", "values": [1, 2, 3]},
            ],
            "snap_id": "test-snap-id",
            "status": "OK",
        }
        for n in MetricsNames
    ]
    data = {"metrics": metrics}

    metrics_results = MetricsResults.unmarshal(data)

    assert metrics_results == MetricsResults(
        metrics=[
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="daily_device_change",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="installed_base_by_channel",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="installed_base_by_country",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="installed_base_by_operating_system",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="installed_base_by_version",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="weekly_device_change",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="weekly_installed_base_by_channel",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="weekly_installed_base_by_country",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="weekly_installed_base_by_operating_system",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            MetricResults(
                status=MetricsStatus["OK"],
                snap_id="test-snap-id",
                metric_name="weekly_installed_base_by_version",
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
        ]
    )
    assert metrics_results.marshal() == data


@pytest.mark.parametrize("results", [False, {1}, 1, "INVALID", []])
def test_metrics_results_invalid_payload(results):
    with pytest.raises(
        ValueError, match=re.escape(f"Invalid metrics results: {results!r}")
    ):
        MetricsResults.unmarshal(results)


@pytest.mark.parametrize("metrics", [False, {1}, 1, "INVALID", {}])
def test_metrics_metrics_invalid_metrics(metrics):
    data = {"metrics": metrics}
    with pytest.raises(ValueError, match=re.escape(f"Invalid metrics: {metrics!r}")):
        MetricsResults.unmarshal(data)
