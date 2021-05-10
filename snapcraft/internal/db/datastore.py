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

import logging
import pathlib
from typing import Any, Dict, List, Type

import tinydb
import yaml

import snapcraft

from . import errors, migration

logger = logging.getLogger(__name__)


class _YAMLStorage(tinydb.Storage):
    """Provide YAML-backed storage for TinyDB."""

    def __init__(self, path: str):
        self.path = pathlib.Path(path)
        logger.debug(f"_YAMLStorage init {self.path}")

    def read(self) -> Dict[str, Any]:
        """Read database from file."""
        logger.debug(f"_YAMLStorage read: {self.path}")
        try:
            with self.path.open() as fd:
                db_data = yaml.safe_load(fd)
        except FileNotFoundError:
            return dict()

        if not isinstance(db_data, dict) or any(
            [not isinstance(k, str) for k in db_data.keys()]
        ):
            raise RuntimeError(
                f"Invalid datastore contents for {str(self.path)}: {db_data!r}"
            )

        return db_data

    def write(self, data) -> None:
        """Write database (data) to file."""
        logger.debug(f"_YAMLStorage write: {self.path} data={data!r}")
        self.path.write_text(yaml.dump(data))

    def close(self):
        """Nothing to do since we do not keep <path> open."""
        logger.debug(f"_YAMLStorage close: {self.path}")


class _YAMLStorageReadOnly(_YAMLStorage):
    def write(self, data) -> None:
        """Ignore any writes in read-only mode."""


class Datastore:
    """Datastore class, providing context manager for TinyDB.

    Manages migrations and storage requirements.  If migrations
    do not indicate support for current datastore version,
    SnapcraftDatastoreVersionUnsupported will be raised. In that
    event, some basic fallback mode can be utilized by re-opening
    datastore in read-only mode."""

    def __init__(
        self,
        *,
        path: pathlib.Path,
        migrations: List[Type[migration.Migration]],
        read_only: bool = False,
        snapcraft_version: str = snapcraft.__version__,
    ) -> None:
        self.path = path
        self._snapcraft_version = snapcraft_version

        if read_only:
            storage_class: Type[_YAMLStorage] = _YAMLStorageReadOnly
        else:
            storage_class = _YAMLStorage

        self.db = tinydb.TinyDB(str(path), storage=storage_class)

        logger.debug(f"Datastore init: {self.path} read_only={read_only}")

        # Force the datastore to be read by making a query, otherwise it is
        # only read on the first access.
        _ = self.db.tables()

        # Nothing left to do if opening in read-only mode.
        if read_only:
            return

        current_version: int = 0
        supported_version: int = 0

        for migration_class in migrations:
            current_version = migration_class(
                db=self.db, snapcraft_version=self._snapcraft_version
            ).apply()
            supported_version = migration_class.SCHEMA_VERSION

        if current_version > supported_version:
            raise errors.SnapcraftDatastoreVersionUnsupported(
                path=self.path,
                current_version=current_version,
                supported_version=supported_version,
            )

    def __enter__(self) -> tinydb.TinyDB:
        return self.db

    def __exit__(self, exc_value, exc_type, exc_traceback) -> None:
        self.close()

    def close(self) -> None:
        """Close database."""
        self.db.close()
        logger.debug(f"Datastore close: {self.path}")
