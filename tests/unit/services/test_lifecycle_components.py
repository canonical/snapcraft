# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Tests for Components in Snapcraft's Lifecycle service."""
import pytest


@pytest.fixture
def extra_project_params(extra_project_params):
    from craft_application.models import (  # pylint: disable=import-outside-toplevel
        SummaryStr,
        VersionStr,
    )

    extra_project_params["components"] = {
        "firstcomponent": {
            "type": "test",
            "summary": SummaryStr("first component"),
            "description": "lorem ipsum",
            "version": VersionStr("1.0"),
        },
        "secondcomponent": {
            "type": "test",
            "summary": SummaryStr("second component"),
            "description": "lorem ipsum",
            "version": VersionStr("1.0"),
        },
    }

    return extra_project_params


@pytest.mark.usefixtures("enable_partitions_feature")
@pytest.mark.usefixtures("default_project")
@pytest.mark.parametrize(
    "component,expected_prime",
    [
        ("firstcomponent", "partitions/component/firstcomponent/prime"),
        ("secondcomponent", "partitions/component/secondcomponent/prime"),
        (None, "prime"),
    ],
)
def test_lifecycle_get_prime_dir(lifecycle_service, component, expected_prime):

    lifecycle_service.setup()

    assert (
        lifecycle_service.get_prime_dir(component=component)
        == lifecycle_service._work_dir / expected_prime
    )
