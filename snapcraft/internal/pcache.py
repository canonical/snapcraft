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

import dbm
import functools
import logging
import pathlib
import shelve
import time
from typing import Callable


logger = logging.getLogger()


class PcacheInvalid(Exception):
    pass


def _pcache(*, path: pathlib.Path, expiry_secs: float, func, args, kwargs):
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with shelve.open(str(path), writeback=True) as db:
            logger.debug(f"Opened persistent cache {str(path)}: {db!r}")

            # Purge expired keys on load, as well as sanity check data.
            timestamp = time.time()
            for key in list(db.keys()):
                value = db.get(key)
                if (
                    not isinstance(value, dict)
                    or "expiry" not in value
                    or not isinstance(value["expiry"], float)
                    or "result" not in value
                ):
                    logger.debug(f"Invalid pcache: unexpected format")
                    raise PcacheInvalid()

                if value["expiry"] < timestamp:
                    logger.debug(f"Purging expired key: {key!r}")
                    del db[key]

            # Combine function name, args, and sorted kwargs to represent
            # unique key, relying on repr().
            key = f"{func.__name__}:{args!r}:{sorted(kwargs)!r}"

            if key in db:
                logger.debug(f"Returning persistent cache result for {key!r}")
                return db[key]["result"]

            result = func(*args, **kwargs)
            db[key] = dict()
            db[key]["result"] = result
            db[key]["expiry"] = timestamp + expiry_secs
            return result
    except dbm.error as error:
        logger.debug(f"Invalid pcache: {error!r}")
        raise PcacheInvalid()


def pcache(path_func: Callable[[], pathlib.Path], expiry_secs: float = 86400.0):
    """Persistent cache decorator.

    Cache results of wrapped function, returning the last known-good result,
    if available.  Configurable expiration times may be set, with a default
    of one-day.

    The recorded items have an identity key derived from:
    - function name
    - function arguments
    - function keyword arguments

    All arguments to the wrapped function must be representable via repr().
    The arguments must be represented in an uniquely identifiable and
    repeatable manner to ensure accuracy and prevent collisions.

    Note that this current implementation is not optimized for performance.
    The database is opened and closed on-demand which carries a cost.  It could
    be updated to keep the database open across the lifetime of the process,
    but that may complicate usage with multiple processes (e.g. snapcraftctl).
    For our current use cases, this is the safe and appropriate default.

    :param path_func Callable to get db path.  This allows for late-binding of
        a path. This is particularly helpful for instrumenting tests when using
        functions with this decorator.

    :param expiry_secs Record created, if any, is good for <expiry> seconds.
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            path = path_func()
            try:
                return _pcache(
                    path=path,
                    expiry_secs=float(expiry_secs),
                    func=func,
                    args=args,
                    kwargs=kwargs,
                )
            except PcacheInvalid:
                # It could fail for a number of reasons, including
                # corrupt db, incompatible changes, etc.   In those
                # cases we simply purge the cache and attempt again
                # without the exception handler.
                logger.debug(f"Unlinking invalid pcache: {path!r}")
                path.unlink()
                return _pcache(
                    path=path,
                    expiry_secs=expiry_secs,
                    func=func,
                    args=args,
                    kwargs=kwargs,
                )

        return wrapper

    return decorator
