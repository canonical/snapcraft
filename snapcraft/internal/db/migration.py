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

import abc
import logging
from datetime import datetime
from typing import Any, ClassVar, Dict

import tinydb

logger = logging.getLogger(__name__)


class Migration(metaclass=abc.ABCMeta):
    """Migrates schema from <SCHEMA_VERSION-1> to <SCHEMA_VERSION>."""

    SCHEMA_VERSION: ClassVar[int] = 0

    def __init__(self, *, db: tinydb.TinyDB, snapcraft_version: str) -> None:
        self.db = db
        self._snapcraft_version = snapcraft_version

    def _query_control_record(self) -> Dict[str, Any]:
        """Query control record (single document in 'control' table)."""
        control_table = self.db.table("control")
        control_records = control_table.all()

        if len(control_records) == 0:
            return dict(schema_version=0)
        elif len(control_records) == 1:
            return control_records[0]

        raise RuntimeError(f"Invalid control records: {control_records!r}")

    def _update_control_schema_version(self) -> None:
        """Update 'control' table record to SCHEMA_VERSION."""
        control_record = self._query_control_record()
        control_record["schema_version"] = self.SCHEMA_VERSION

        control_table = self.db.table("control")
        control_table.truncate()
        control_table.insert(control_record)

    def _record_migration(self) -> None:
        """Record migration in 'migration' table."""
        migration_table = self.db.table("migration")
        migration_table.insert(
            {
                "schema_version": self.SCHEMA_VERSION,
                "snapcraft_version": self._snapcraft_version,
                "timestamp": datetime.utcnow().isoformat() + "Z",
            }
        )

    @abc.abstractmethod
    def _migrate(self) -> None:
        """Per-migration implementation."""
        ...

    def apply(self) -> int:
        """Apply migration, if determined to be necessary.

        Returns current schema version."""
        control_record = self._query_control_record()
        current_schema_version = control_record["schema_version"]

        if self.SCHEMA_VERSION <= current_schema_version:
            logger.debug(
                f"Migration apply: migration {self.SCHEMA_VERSION} already applied, ignoring..."
            )
            return current_schema_version

        logger.debug(
            f"Migration apply: applying migration for {self.SCHEMA_VERSION} for {control_record}"
        )
        self._migrate()
        self._record_migration()
        self._update_control_schema_version()
        return self.SCHEMA_VERSION


class MigrationV1(Migration):
    """Default (Initial) Migration to v1."""

    SCHEMA_VERSION: ClassVar[int] = 1

    def _migrate_control(self) -> None:
        control_table = self.db.table("control")
        control_table.insert(
            {
                "created_with_snapcraft_version": self._snapcraft_version,
                "schema_version": self.SCHEMA_VERSION,
            }
        )

    def _migrate(self) -> None:
        """Per-migration implementation."""
        self._migrate_control()
