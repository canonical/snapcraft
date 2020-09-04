# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import datetime
from unittest.mock import Mock

import pytest
import tinydb

from snapcraft.internal.db import migration


@pytest.fixture
def test_db():
    db = tinydb.TinyDB(storage=tinydb.storages.MemoryStorage)
    return Mock(wraps=db)


@pytest.fixture(autouse=True)
def mock_datetime(monkeypatch):
    datetime_mock = Mock(wraps=datetime.datetime)
    datetime_mock.utcnow.return_value = datetime.datetime(2020, 1, 2, 3, 4, 5, 6)
    monkeypatch.setattr(migration, "datetime", datetime_mock)


def test_migration_on_empty_db(test_db):
    m = migration.MigrationV1(db=test_db, snapcraft_version="42")
    m.apply()

    assert test_db.table("control").all() == [
        {"created_with_snapcraft_version": "42", "schema_version": 1}
    ]
    assert test_db.table("migration").all() == [
        {
            "schema_version": 1,
            "snapcraft_version": "42",
            "timestamp": "2020-01-02T03:04:05.000006Z",
        }
    ]


def test_no_migration_on_v1_db(test_db):
    test_db.table("control").insert(
        {"created_with_snapcraft_version": "42", "schema_version": 1}
    )

    m = migration.MigrationV1(db=test_db, snapcraft_version="42")
    m.apply()

    assert test_db.table("control").all() == [
        {"created_with_snapcraft_version": "42", "schema_version": 1}
    ]
    assert test_db.table("migration").all() == []


def test_no_migration_on_v2_db(test_db):
    test_db.table("control").insert(
        {"created_with_snapcraft_version": "42", "schema_version": 2}
    )

    m = migration.MigrationV1(db=test_db, snapcraft_version="42")
    m.apply()

    assert test_db.table("control").all() == [
        {"created_with_snapcraft_version": "42", "schema_version": 2}
    ]
    assert test_db.table("migration").all() == []
