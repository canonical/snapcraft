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
import tempfile

from snapcraft import (
    libraries,
    tests,
)


class TestLdLibraryPathParser(tests.TestCase):

    def _write_conf_file(self, contents):
        tmp = tempfile.NamedTemporaryFile(delete=False, mode='w')
        self.addCleanup(os.remove, tmp.name)

        tmp.write(contents)
        tmp.close()

        return tmp.name

    def test_extract_ld_library_paths(self):
        file_path = self._write_conf_file("""# This is a comment
/foo/bar
/colon:/separated,/comma\t/tab /space # This is another comment
/baz""")

        self.assertEqual(['/foo/bar', '/colon', '/separated', '/comma',
                          '/tab', '/space', '/baz'],
                         libraries._extract_ld_library_paths(file_path))
