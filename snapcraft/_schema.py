#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import os

import jsonschema
import yaml

from snapcraft.internal import common


class SnapcraftSchemaError(Exception):

    @property
    def message(self):
        return self._message

    def __init__(self, message):
        self._message = message


class Validator:

    def __init__(self, snapcraft_yaml=None):
        """Create a validation instance for snapcraft_yaml."""
        self._snapcraft = snapcraft_yaml if snapcraft_yaml else {}
        self._load_schema()

    @property
    def schema(self):
        """Return all schema properties."""

        return self._schema['properties'].copy()

    @property
    def part_schema(self):
        """Return part-specific schema properties."""

        sub = self.schema['parts']['patternProperties']
        properties = sub['^(?!plugins$)[a-z0-9][a-z0-9+-\/]*$']['properties']
        return properties

    def _load_schema(self):
        schema_file = os.path.abspath(os.path.join(
            common.get_schemadir(), 'snapcraft.yaml'))
        try:
            with open(schema_file) as fp:
                self._schema = yaml.load(fp)
        except FileNotFoundError:
            raise SnapcraftSchemaError(
                'snapcraft validation file is missing from installation path')

    def validate(self):
        format_check = jsonschema.FormatChecker()
        try:
            jsonschema.validate(
                self._snapcraft, self._schema, format_checker=format_check)
        except jsonschema.ValidationError as e:
            messages = [e.message]
            path = []
            while e.absolute_path:
                element = e.absolute_path.popleft()
                # assume numbers are indices and use 'xxx[123]' notation.
                if isinstance(element, int):
                    path[-1] = '{}[{}]'.format(path[-1], element)
                else:
                    path.append(str(element))
            if path:
                messages.insert(0, "The '{}' property does not match the "
                                   "required schema:".format('/'.join(path)))
            if e.cause:
                messages.append('({})'.format(e.cause))

            raise SnapcraftSchemaError(' '.join(messages))
