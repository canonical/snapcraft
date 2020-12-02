# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import importlib.util
import logging
import os
from importlib.abc import Loader
from typing import Dict  # noqa: F401
from unittest.mock import patch

from snapcraft.extractors import _errors

from ._metadata import ExtractedMetadata

logger = logging.getLogger(__name__)


def extract(relpath: str, *, workdir: str) -> ExtractedMetadata:
    if os.path.basename(relpath) != "setup.py":
        raise _errors.UnhandledFileError(relpath, "setup.py")

    spec = importlib.util.spec_from_file_location(
        "setuppy", os.path.join(workdir, relpath)
    )
    setuppy = importlib.util.module_from_spec(spec)

    params = dict()  # type: Dict[str, str]

    def _fake_setup(*args, **kwargs):
        nonlocal params
        params = kwargs

    with patch("setuptools.setup") as setuptools_mock:
        with patch("distutils.core.setup") as distutils_mock:
            setuptools_mock.side_effect = _fake_setup
            distutils_mock.side_effect = _fake_setup

            # Should never happen, but ensure spec.loader is set.
            loader = spec.loader
            if loader is None or not isinstance(loader, Loader):
                raise RuntimeError("Invalid spec loader")

            # This would really fail during the use of the plugin
            # but let's be cautious and add the proper guards.
            try:
                loader.exec_module(setuppy)
            except SystemExit:
                raise _errors.SetupPyFileParseError(path=relpath)
            except ImportError as e:
                raise _errors.SetupPyImportError(path=relpath, error=str(e)) from e

    version = params.get("version")
    description = params.get("description")

    return ExtractedMetadata(version=version, description=description)
