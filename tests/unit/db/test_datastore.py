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
import textwrap
from unittest.mock import Mock

import pytest
import tinydb

from snapcraft.internal.db import datastore, errors, migration


@pytest.fixture(autouse=True)
def mock_datetime(monkeypatch):
    datetime_mock = Mock(wraps=datetime.datetime)
    datetime_mock.utcnow.return_value = datetime.datetime(2020, 1, 2, 3, 4, 5, 6)
    monkeypatch.setattr(migration, "datetime", datetime_mock)


def test_open_invalid_data(tmp_path):
    db_path = tmp_path / "db.yaml"
    db_path.write_text("invalid")

    with pytest.raises(RuntimeError) as error:
        with datastore.Datastore(path=db_path, migrations=[]) as _:
            assert False

    assert (
        error.value.args[0]
        == f"Invalid datastore contents for {str(db_path)}: 'invalid'"
    )


def test_create_no_migration(tmp_path):
    db_path = tmp_path / "db.yaml"

    with datastore.Datastore(path=db_path, migrations=[], snapcraft_version="42") as db:
        assert isinstance(db, tinydb.TinyDB) is True
        db.table("test").insert({"foo": "bar"})

    assert db_path.read_text() == textwrap.dedent(
        """\
        test:
          '1':
            foo: bar
        """
    )


def test_create_default_migration(tmp_path):
    db_path = tmp_path / "db.yaml"

    with datastore.Datastore(
        path=db_path, migrations=[migration.MigrationV1], snapcraft_version="42"
    ) as db:
        assert isinstance(db, tinydb.TinyDB) is True
        db.table("test").insert({"foo": "bar"})

    assert db_path.read_text() == textwrap.dedent(
        """\
        control:
          '1':
            created_with_snapcraft_version: '42'
            schema_version: 1
        migration:
          '1':
            schema_version: 1
            snapcraft_version: '42'
            timestamp: '2020-01-02T03:04:05.000006Z'
        test:
          '1':
            foo: bar
        """
    )


def test_unknown_version(tmp_path):
    db_path = tmp_path / "db.yaml"
    control_record = {"created_with_snapcraft_version": "42", "schema_version": 3}

    # Populate test db with schema version greater than 1.
    with datastore.Datastore(path=db_path, migrations=[], snapcraft_version="42") as db:
        assert isinstance(db, tinydb.TinyDB) is True
        db.table("control").insert(control_record)

    # Re-open db, should raise version unsupported error.
    with pytest.raises(errors.SnapcraftDatastoreVersionUnsupported):
        with datastore.Datastore(
            path=db_path, migrations=[migration.MigrationV1]
        ) as db:
            assert False

    # Verify we are able to open unsupported version in read-only.
    with datastore.Datastore(
        path=db_path, migrations=[migration.MigrationV1], read_only=True
    ) as db:
        assert db.table("control").all() == [control_record]
