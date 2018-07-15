# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import importlib
import logging
import os
import pkgutil

from snapcraft import extractors
from snapcraft.internal.errors import (
    InvalidExtractorValueError,
    MissingMetadataFileError,
    UnhandledMetadataFileTypeError,
)

logger = logging.getLogger(__name__)


def extract_metadata(part_name: str, file_path: str) -> extractors.ExtractedMetadata:
    if not os.path.exists(file_path):
        raise MissingMetadataFileError(part_name, file_path)

    # Iterate through each extractor module, calling the 'extract' function
    # from it. If it raises an 'UnhandledFileError' move onto the next.
    for _, module_name, _ in pkgutil.iter_modules(extractors.__path__):  # type: ignore
        # We only care about non-private modules in here
        if not module_name.startswith("_"):
            module = importlib.import_module(
                "snapcraft.extractors.{}".format(module_name)
            )

            try:
                # mypy is confused since we dynamically loaded the module. It
                # doesn't think it has an 'extract' function. Ignore.
                metadata = module.extract(file_path)  # type: ignore
                if not isinstance(metadata, extractors.ExtractedMetadata):
                    raise InvalidExtractorValueError(file_path, module_name)

                return metadata
            except extractors.UnhandledFileError:
                pass  # Try the next extractor
            except AttributeError:
                logger.warn(
                    "Extractor {!r} doesn't include the 'extract' function. "
                    "Skipping...".format(module_name)
                )

    # If we get here, no extractor was able to handle the file
    raise UnhandledMetadataFileTypeError(file_path)
