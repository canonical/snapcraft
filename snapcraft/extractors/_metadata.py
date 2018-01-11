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

import yaml
from typing import Any, Dict


class ExtractedMetadata(yaml.YAMLObject):
    """Collection of metadata extracted from a part."""

    yaml_tag = u'!ExtractedMetadata'

    def __init__(self, *, summary='', description='') -> None:
        """Create a new ExtractedMetadata instance.

        :param str summary: Extracted summary
        :param str description: Extracted description
        """

        self._data = {}  # type: Dict[str, str]

        if summary:
            self._data['summary'] = summary
        if description:
            self._data['description'] = description

    def update(self, other: 'ExtractedMetadata') -> None:
        """Update this metadata with other metadata.

        Note that the other metadata will take precedence, and may overwrite
        data contained here.

        :param ExtractedMetadata other: Metadata from which to update
        """
        self._data.update(other.to_dict())

    def get_summary(self) -> str:
        """Return extracted summary.

        :returns: Extracted summary
        :rtype: str
        """
        return self._data.get('summary')

    def get_description(self) -> str:
        """Return extracted description.

        :returns: Extracted description
        :rtype: str
        """
        return self._data.get('description')

    def to_dict(self) -> Dict[str, str]:
        """Return all extracted metadata.

        :returns: All extracted metadata in dict form.
        :rtype: dict
        """
        return self._data.copy()

    def __eq__(self, other: Any) -> bool:
        if type(other) is type(self):
            return self._data == other._data

        return False
