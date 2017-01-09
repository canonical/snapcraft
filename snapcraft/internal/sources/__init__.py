# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

"""Common 'source' options.

Unless the part plugin overrides this behaviour, a part can use these
'source' keys in its definition. They tell snapcraft where to pull source
code for that part, and how to unpack it if necessary.

  - source: url-or-path

    A URL or path to some source tree to build. It can be local
    ('./src/foo') or remote ('https://foo.org/...'), and can refer to a
    directory tree or a tarball or a revision control repository
    ('git:...').

  - source-type: git, bzr, hg, svn, tar, deb, rpm, or zip

    In some cases the source string is not enough to identify the version
    control system or compression algorithm. The source-type key can tell
    snapcraft exactly how to treat that content.

  - source-depth: <integer>

    By default clones or branches with full history, specifying a depth
    will truncate the history to the specified number of commits.

  - source-branch: <branch-name>

    Snapcraft will checkout a specific branch from the source tree. This
    only works on multi-branch repositories from git and hg (mercurial).

  - source-commit: <commit>

    Snapcraft will checkout the specific commit from the source tree revision
    control system.

  - source-tag: <tag>

    Snapcraft will checkout the specific tag from the source tree revision
    control system.

  - source-subdir: path

    Snapcraft will checkout the repository or unpack the archive referred to
    by the 'source' keyword into parts/<part-name>/src/ but it will only
    copy the specified subdirectory into parts/<part-name>/build/

Note that plugins might well define their own semantics for the 'source'
keywords, because they handle specific build systems, and many languages
have their own built-in packaging systems (think CPAN, PyPI, NPM). In those
cases you want to refer to the help text for the specific plugin.

  snapcraft help <plugin>

"""

import logging
import os
import os.path
import re

from snapcraft.internal import common
from ._bazaar import Bazaar          # noqa
from ._deb import Deb                # noqa
from ._git import Git                # noqa
from ._local import Local            # noqa
from ._mercurial import Mercurial    # noqa
from ._rpm import Rpm                # noqa
from ._script import Script          # noqa
from ._subversion import Subversion  # noqa
from ._tar import Tar                # noqa
from ._zip import Zip                # noqa

logging.getLogger('urllib3').setLevel(logging.CRITICAL)


__SOURCE_DEFAULTS = {
    'source': '.',
    'source-commit': None,
    'source-depth': None,
    'source-tag': None,
    'source-type': None,
    'source-branch': None,
    'source-subdir': None,
}


def get_source_defaults():
    return __SOURCE_DEFAULTS.copy()


def get(sourcedir, builddir, options):
    """Populate sourcedir and builddir from parameters defined in options.

    :param str sourcedir: The source directory to use.
    :param str builddir: The build directory to use.
    :param options: source options.
    """
    source_type = getattr(options, 'source_type', None)
    source_attributes = dict(
        source_depth=getattr(options, 'source_depth', None),
        source_tag=getattr(options, 'source_tag', None),
        source_commit=getattr(options, 'source_commit', None),
        source_branch=getattr(options, 'source_branch', None),
    )

    handler_class = get_source_handler(options.source, source_type=source_type)
    handler = handler_class(options.source, sourcedir, **source_attributes)
    handler.pull()


_source_handler = {
    'bzr': Bazaar,
    'deb': Deb,
    'rpm': Rpm,
    'git': Git,
    'hg': Mercurial,
    'mercurial': Mercurial,
    'subversion': Subversion,
    'svn': Subversion,
    'tar': Tar,
    'zip': Zip,
}


def get_source_handler(source, *, source_type=''):
    if not source_type:
        source_type = _get_source_type_from_uri(source)

    return _source_handler.get(source_type, Local)


_tar_type_regex = re.compile(r'.*\.((tar(\.(xz|gz|bz2))?)|tgz)$')


def _get_source_type_from_uri(source, ignore_errors=False):
    source_type = ''
    if source.startswith('bzr:') or source.startswith('lp:'):
        source_type = 'bzr'
    elif source.startswith('git:') or source.startswith('git@') or \
            source.endswith('.git'):
        source_type = 'git'
    elif source.startswith('svn:'):
        source_type = 'subversion'
    elif _tar_type_regex.match(source):
        source_type = 'tar'
    elif source.endswith('.zip'):
        source_type = 'zip'
    elif source.endswith('deb'):
        source_type = 'deb'
    elif source.endswith('rpm'):
        source_type = 'rpm'
    elif common.isurl(source) and not ignore_errors:
        raise ValueError('no handler to manage source ({})'.format(source))
    elif not os.path.isdir(source) and not ignore_errors:
        raise ValueError('local source ({}) is not a directory'.format(source))

    return source_type
