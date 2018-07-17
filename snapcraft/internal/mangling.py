# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import re
import subprocess
from typing import FrozenSet

from snapcraft import file_utils
from snapcraft.internal import elf


logger = logging.getLogger(__name__)


def rewrite_python_shebangs(root_dir):
    """Recursively change #!/usr/bin/pythonX shebangs to #!/usr/bin/env pythonX

    :param str root_dir: Directory that will be crawled for shebangs.
    """

    file_pattern = re.compile(r"")
    argless_shebang_pattern = re.compile(r"\A#!.*(python\S*)$", re.MULTILINE)
    shebang_pattern_with_args = re.compile(
        r"\A#!.*(python\S*)[ \t\f\v]+(\S+)$", re.MULTILINE
    )

    file_utils.replace_in_file(
        root_dir, file_pattern, argless_shebang_pattern, r"#!/usr/bin/env \1"
    )

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
        root_dir,
        file_pattern,
        shebang_pattern_with_args,
        r"""#!/bin/sh\n''''exec \1 \2 -- "$0" "$@" # '''""",
    )


def clear_execstack(*, elf_files: FrozenSet[elf.ElfFile]) -> None:
    """Clears the execstack for the relevant elf_files

    param elf.ElfFile elf_files: the full list of elf files to analyze
                                 and clear the execstack if present.
    """
    execstack_path = file_utils.get_tool_path("execstack")
    elf_files_with_execstack = [e for e in elf_files if e.execstack_set]

    if elf_files_with_execstack:
        formatted_items = ["- {}".format(e.path) for e in elf_files_with_execstack]
        logger.warning(
            "The execstacks are going to be cleared for the following "
            "files:\n{}\n"
            "To disable this behavior set "
            "`build-attributes: [keep-execstack]` "
            "for the part.".format("\n".join(formatted_items))
        )

    for elf_file in elf_files_with_execstack:
        try:
            subprocess.check_call([execstack_path, "--clear-execstack", elf_file.path])
        except subprocess.CalledProcessError:
            logger.warning("Failed to clear execstack for {!r}".format(elf_file.path))
