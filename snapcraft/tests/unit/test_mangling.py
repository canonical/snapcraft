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

import os
import textwrap

from testtools.matchers import FileContains

from snapcraft.internal import mangling
from snapcraft.tests import unit


def _create_file(filename, contents):
    os.makedirs('test-dir', exist_ok=True)
    file_path = os.path.join('test-dir', filename)
    with open(file_path, 'w') as f:
        f.write(contents)

    return file_path


class ManglingPythonShebangTestCase(unit.TestCase):

    def test_python(self):
        file_path = _create_file('file', textwrap.dedent("""\
            #! /usr/bin/python2.7

            # Larger file
        """))
        mangling.rewrite_python_shebangs(os.path.dirname(file_path))
        self.assertThat(file_path, FileContains(textwrap.dedent("""\
            #!/usr/bin/env python2.7

            # Larger file
        """)))

    def test_python3(self):
        file_path = _create_file('file', '#!/usr/bin/python3')
        mangling.rewrite_python_shebangs(os.path.dirname(file_path))
        self.assertThat(file_path, FileContains('#!/usr/bin/env python3'))

    def test_python_args(self):
        file_path1 = _create_file('file1', '#!/usr/bin/python')
        file_path2 = _create_file('file2', textwrap.dedent("""\
            #! /usr/bin/python -E

            # Larger file
        """))
        mangling.rewrite_python_shebangs(os.path.dirname(file_path1))
        self.assertThat(file_path1, FileContains('#!/usr/bin/env python'))
        self.assertThat(file_path2, FileContains(
            textwrap.dedent("""\
                #!/bin/sh
                ''''exec python -E -- "$0" "$@" # '''

                # Larger file
            """)))

    def test_python3_args(self):
        file_path1 = _create_file('file1', '#!/usr/bin/python3')
        file_path2 = _create_file('file2', '#!/usr/bin/python3 -E')
        mangling.rewrite_python_shebangs(os.path.dirname(file_path1))
        self.assertThat(file_path1, FileContains('#!/usr/bin/env python3'))
        self.assertThat(file_path2, FileContains(
            textwrap.dedent("""\
                #!/bin/sh
                ''''exec python3 -E -- "$0" "$@" # '''""")))

    def test_python_mixed_args(self):
        file_path1 = _create_file('file1', '#!/usr/bin/python')
        # Ensure extra spaces are chopped off
        file_path2 = _create_file('file2', '#!/usr/bin/python3       -Es')
        mangling.rewrite_python_shebangs(os.path.dirname(file_path1))
        self.assertThat(file_path1, FileContains('#!/usr/bin/env python'))
        self.assertThat(file_path2, FileContains(
            textwrap.dedent("""\
                #!/bin/sh
                ''''exec python3 -Es -- "$0" "$@" # '''""")))

    def test_following_docstring_no_rewrite(self):
        file_path = _create_file('file', textwrap.dedent("""\
            #!/usr/bin/env python3.5

            '''
            This is a test
            =======================
        """))
        mangling.rewrite_python_shebangs(os.path.dirname(file_path))
        self.assertThat(file_path, FileContains(textwrap.dedent("""\
            #!/usr/bin/env python3.5

            '''
            This is a test
            =======================
        """)))
