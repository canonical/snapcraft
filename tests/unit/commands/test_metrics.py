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

from textwrap import dedent

import pytest

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase


class MetricsCommandTestCase(FakeStoreCommandsBaseTestCase):
    def test_metrics_without_snap_raises_exception(self):
        result = self.run_command(
            ["metrics", "--name", "daily_device_change", "--format", "table"]
        )

        assert result.exit_code == 2
        assert "Usage:" in result.output

    def test_metrics_without_name_raises_exception(self):
        result = self.run_command(["metrics", "snap-test", "--format", "table"])

        assert result.exit_code == 2
        assert "Usage:" in result.output

    def test_metrics_without_format_raises_exception(self):
        result = self.run_command(
            ["metrics", "snap-test", "--name", "daily_device_change"]
        )

        assert result.exit_code == 2
        assert "Usage:" in result.output

    @pytest.mark.skip("needs more work")
    def test_status_without_login_must_ask(self):
        self.fake_store_account_info.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            self.fake_store_account_info_data,
        ]

        result = self.run_command(
            [
                "metrics",
                "snap-test",
                "--name",
                "daily_device_change",
                "--format",
                "table",
            ],
            input="user@example.com\nsecret\n",
        )
        assert "You are required to login before continuing." in result.output

    def test_status_table_format(self):
        result = self.run_command(
            [
                "metrics",
                "snap-test",
                "--name",
                "daily_device_change",
                "--format",
                "table",
            ]
        )

        assert result.output == dedent(
            """\
               Devices    2021-01-01  2021-01-02  2021-01-03
               Continued  10          11          12
               Lost       1           2           3
               New        2           3           4
            """
        )
        assert result.exit_code == 0

    def test_status_table_json(self):
        result = self.run_command(
            [
                "metrics",
                "snap-test",
                "--name",
                "daily_device_change",
                "--format",
                "json",
            ]
        )

        assert result.output == dedent(
            """\
                {
                  "buckets": [
                    "2021-01-01",
                    "2021-01-02",
                    "2021-01-03"
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
                }
            """
        )
        assert result.exit_code == 0
