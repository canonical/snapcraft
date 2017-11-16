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

import re

from snapcraft import file_utils


def rewrite_python_shebangs(root_dir):
    """Recursively change #!/usr/bin/pythonX shebangs to #!/usr/bin/env pythonX

    :param str root_dir: Directory that will be crawled for shebangs.
    """

    file_pattern = re.compile(r'')
    argless_shebang_pattern = re.compile(
        r'\A#!.*(python\S*)$', re.MULTILINE)
    shebang_pattern_with_args = re.compile(
        r'\A#!.*(python\S*)[ \t\f\v]+(\S+)$', re.MULTILINE)

    file_utils.replace_in_file(
        root_dir, file_pattern, argless_shebang_pattern, r'#!/usr/bin/env \1')

    # The above rewrite will barf if the shebang includes any args to python.
    # For example, if the shebang was `#!/usr/bin/python3 -Es`, just replacing
    # that with `#!/usr/bin/env python3 -Es` isn't going to work as `env`
    # doesn't support arguments like that.
    #
    # The solution is to replace the shebang with one pointing to /bin/sh, and
    # then exec the original shebang with included arguments. This requires
    # some quoting hacks to ensure the file can be interpreted by both sh as
    # well as python, but it's better than shipping our own `env`.
    file_utils.replace_in_file(
        root_dir, file_pattern, shebang_pattern_with_args,
        r"""#!/bin/sh\n''''exec \1 \2 -- "$0" "$@" # '''""")
