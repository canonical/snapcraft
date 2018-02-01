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
import os
import re
from typing import FrozenSet, List

from snapcraft import file_utils
from snapcraft.internal import elf, repo


logger = logging.getLogger(__name__)


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


def _get_dynamic_linker(library_list: List[str]) -> str:
    """Return the dynamic linker from library_list."""
    regex = re.compile(r'(?P<dynamic_linker>ld-[\d.]+.so)$')

    for library in library_list:
        m = regex.search(os.path.basename(library))
        if m:
            return library

    raise RuntimeError(
        'The format for the linker should be of the form '
        '<root>/ld-<X>.<Y>.so. There are no matches for the '
        'current libc6 package')


def handle_glibc_mismatch(*, elf_files: FrozenSet[elf.ElfFile],
                          root_path: str, core_base_path: str,
                          snap_base_path: str,
                          preferred_patchelf_path=None) -> None:
    """Copy over libc6 libraries from the host and patch necessary elf files.

    If no newer glibc version is detected in elf_files, this function returns.
    The dynamic linker and related libraries to libc6 are expected to be found
    in root_path.

    :param snapcraft.internal.elf.ElfFile elf_files:
        set of candidate elf files to patch if a newer libc6 is required.
    :param str root_path: the root path of a snap tree.
    :param str core_base_path: the path to the base snap.
    :param str snap_base_path: absolute path to the snap once installed to
                               setup proper rpaths.
    :param str preferred_patchelf_path: patch the necessary elf_files with
                                        this patchelf.
    """
    formatted_list = list()  # type: List[str]
    patch_elf_files = list()  # type: List[elf.ElfFile]
    for elf_file in elf_files:
        required_glibc = elf_file.get_required_glibc()
        if not required_glibc:
            continue

        # We need to verify now that the GLIBC version would be compatible
        # with that of the base.
        # TODO the linker version depends on the chosen base, but that
        # base may not be installed so we cannot depend on
        # get_core_dynamic_linker to resolve the final path for which
        # we resort to our only working base 16, ld-2.23.so.
        if not elf_file.is_linker_compatible(linker='ld-2.23.so'):
            formatted_list.append('- {} -> GLIBC {}'.format(
                elf_file.path, required_glibc))
            patch_elf_files.append(elf_file)

    if not patch_elf_files:
        return

    logger.warning('The primed files will not work with the current '
                   'base given the GLIBC mismatch of the primed '
                   'files and the linker version (2.23) used in the '
                   'base. These are the GLIBC versions required by '
                   'the primed files that do not match and will be '
                   'patched:\n{}\n'.format('\n'.join(formatted_list)))
    # We assume the current system will satisfy the GLIBC requirement,
    # get the current libc6 libraries (which includes the linker)
    libc6_libraries_list = repo.Repo.get_package_libraries('libc6')

    # For security reasons, we do not want to automatically pull in
    # libraries but expect them to be consciously brought in by stage-packages
    # instead.
    libc6_libraries_paths = [os.path.join(root_path, l[1:])
                             for l in libc6_libraries_list]

    dynamic_linker = _get_dynamic_linker(libc6_libraries_paths)

    # Get the path to the "would be" dynamic linker when this snap is
    # installed. Strip the root_path from the retrieved dynamic_linker
    # variables + the leading `/` so that os.path.join can perform the
    # proper join with snap_base_path.
    dynamic_linker_path = os.path.join(
        snap_base_path, dynamic_linker[len(root_path)+1:])

    elf_patcher = elf.Patcher(dynamic_linker=dynamic_linker_path,
                              root_path=root_path,
                              preferred_patchelf_path=preferred_patchelf_path)
    for elf_file in patch_elf_files:
        # Search for dependencies again now that the new libc6 is
        # migrated.
        elf_file.load_dependencies(root_path=root_path,
                                   core_base_path=core_base_path)
        elf_patcher.patch(elf_file=elf_file)
