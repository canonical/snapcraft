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

import magic
import testtools


class HasBinaryFileHeader:
    def __init__(self, expected_magic):
        """Determines the magic of a file, or in other words matches a file
        against known patterns to determine attributes like its MIME type
        or architecture.
        """
        self._expected_magic = expected_magic
        self._ms = magic.open(magic.NONE)
        self._ms.load()

    def __str__(self):
        return '{}()'.format(self.__name__)

    def parse_file_header(self, header):
        return header

    def match(self, file_path):
        header = self._ms.file(file_path)
        # Catch exceptions on splitting the string to provide context
        # This includes "cannot open `...' (No such file or directory)"
        try:
            parsed_header = self.parse_file_header(header)
        except IndexError:
            raise ValueError('Failed to parse file header {!r}'.format(header))
        if self._expected_magic not in parsed_header:
            return testtools.matchers.Mismatch(
                'Expected {!r} to be in {!r}'.format(
                    self._expected_magic, parsed_header))


class HasLinkage(HasBinaryFileHeader):
    """Match if the file has static or dynamic linkage"""

    def parse_file_header(self, header):
        return super().parse_file_header(header).split(',')[3]


class IsDynamicallyLinked(HasLinkage):
    """Match if the file has dynamic linkage"""

    def __init__(self):
        super().__init__('dynamically linked')


class IsStaticallyLinked(HasLinkage):
    """Match if the file has static linkage"""

    def __init__(self):
        super().__init__('statically linked')


class HasArchitecture(HasBinaryFileHeader):
    """Match if the file was built for the expected architecture"""

    def parse_file_header(self, header):
        return super().parse_file_header(header).split(',')[1]
