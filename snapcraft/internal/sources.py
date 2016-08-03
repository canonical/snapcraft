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

  - source-type: git, bzr, hg, svn, tar, or zip

    In some cases the source string is not enough to identify the version
    control system or compression algorithm. The source-type key can tell
    snapcraft exactly how to treat that content.

  - source-checksum: checksum-of-file

    Snapcraft will use either a file, URL, or raw checksum specified here to
    verify the integrity of the source. The source-type needs to be either tar
    or zip.

  - source-branch: <branch-name>

    Snapcraft will checkout a specific branch from the source tree. This
    only works on multi-branch repositories from git and hg (mercurial).

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
import stat
import os.path
import requests
import shutil
import tarfile
import re
import subprocess
import tempfile
import zipfile
import hashlib
import urllib
import glob

from snapcraft.internal import common
from snapcraft.internal.indicators import download_requests_stream

logging.getLogger('urllib').setLevel(logging.CRITICAL)


class IncompatibleOptionsError(Exception):

    def __init__(self, message):
        self.message = message


class ChecksumDoesNotMatch(Exception):

    def __init__(self, message):
        self.message = message


class Base:

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        self.source = source
        self.source_checksum = source_checksum
        self.source_dir = source_dir
        self.source_tag = source_tag
        self.source_branch = source_branch


class FileBase(Base):

    def pull(self):
        if common.isurl(self.source):
            self.download()
        else:
            shutil.copy2(self.source, self.source_dir)

        self.provision(self.source_dir)

    def download(self):
        request = requests.get(self.source, stream=True, allow_redirects=True)
        request.raise_for_status()

        self.file = os.path.join(
            self.source_dir, os.path.basename(self.source))
        download_requests_stream(request, self.file)


class Script(FileBase):

    def __init__(self, source, source_dir, source_tag=None,
                 source_branch=None):
        super().__init__(source, source_dir, source_tag, source_branch)

    def download(self):
        super().download()
        st = os.stat(self.file)
        os.chmod(self.file, st.st_mode | stat.S_IEXEC)


class Bazaar(Base):

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        super().__init__(
            source, source_dir, source_checksum, source_tag, source_branch)
        if source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a bzr source')
        elif source_checksum:
            raise IncompatibleOptionsError(
                'can\'t specify a source-checksum for a bzr source')

    def pull(self):
        tag_opts = []
        if self.source_tag:
            tag_opts = ['-r', 'tag:' + self.source_tag]
        if os.path.exists(os.path.join(self.source_dir, '.bzr')):
            cmd = ['bzr', 'pull'] + tag_opts + \
                  [self.source, '-d', self.source_dir]
        else:
            os.rmdir(self.source_dir)
            cmd = ['bzr', 'branch'] + tag_opts + \
                  [self.source, self.source_dir]

        subprocess.check_call(cmd)


class Git(Base):

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        super().__init__(
            source, source_dir, source_checksum, source_tag, source_branch)
        if source_tag and source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for '
                'a git source')
        elif source_checksum:
            raise IncompatibleOptionsError(
                'can\'t specify source-checksum for a git source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.git')):
            refspec = 'HEAD'
            if self.source_branch:
                refspec = 'refs/heads/' + self.source_branch
            elif self.source_tag:
                refspec = 'refs/tags/' + self.source_tag

            # Pull changes to this repository and any submodules.
            subprocess.check_call(['git', '-C', self.source_dir, 'pull',
                                   '--recurse-submodules=yes', self.source,
                                   refspec])

            # Merge any updates for the submodules (if any).
            subprocess.check_call(['git', '-C', self.source_dir, 'submodule',
                                   'update'])
        else:
            branch_opts = []
            if self.source_tag or self.source_branch:
                branch_opts = ['--branch',
                               self.source_tag or self.source_branch]
            subprocess.check_call(['git', 'clone', '--depth', '1',
                                  '--recursive'] + branch_opts +
                                  [self.source, self.source_dir])


class Mercurial(Base):

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        super().__init__(
            source, source_dir, source_checksum, source_tag, source_branch)
        if source_tag and source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for a '
                'mercurial source')
        elif source_checksum:
            raise IncompatibleOptionsError(
                'can\'t specify source-checksum for a mercurial source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.hg')):
            ref = []
            if self.source_tag:
                ref = ['-r', self.source_tag]
            elif self.source_branch:
                ref = ['-b', self.source_branch]
            cmd = ['hg', 'pull'] + ref + [self.source, ]
        else:
            ref = []
            if self.source_tag or self.source_branch:
                ref = ['-u', self.source_tag or self.source_branch]
            cmd = ['hg', 'clone'] + ref + [self.source, self.source_dir]

        subprocess.check_call(cmd)


class Subversion(Base):

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        super().__init__(
            source, source_dir, source_checksum, source_tag, source_branch)
        if source_tag:
            if source_branch:
                raise IncompatibleOptionsError(
                    "Can't specify source-tag OR source-branch for a "
                    "Subversion source")
            else:
                raise IncompatibleOptionsError(
                    "Can't specify source-tag for a Subversion source")
        elif source_branch:
            raise IncompatibleOptionsError(
                "Can't specify source-branch for a Subversion source")
        elif source_checksum:
            raise IncompatibleOptionsError(
                "Can't specify source-checksum for a Subversion source")

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.svn')):
            subprocess.check_call(
                ['svn', 'update'], cwd=self.source_dir)
        else:
            if os.path.isdir(self.source):
                subprocess.check_call(
                    ['svn', 'checkout',
                     'file://{}'.format(os.path.abspath(self.source)),
                     self.source_dir])
            else:
                subprocess.check_call(
                    ['svn', 'checkout', self.source, self.source_dir])


class Tar(FileBase):

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        super().__init__(
            source, source_dir, source_checksum, source_tag, source_branch)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a tar source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a tar source')

    def provision(self, dst, clean_target=True, keep_tarball=False):
        # TODO add unit tests.
        tarball = os.path.join(self.source_dir, os.path.basename(self.source))

        if self.source_checksum:
            verify_checksum(self.source_checksum, tarball)

        if clean_target:
            tmp_tarball = tempfile.NamedTemporaryFile().name
            shutil.move(tarball, tmp_tarball)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_tarball, tarball)

        self._extract(tarball, dst)

        if not keep_tarball:
            os.remove(tarball)

    def _extract(self, tarball, dst):
        with tarfile.open(tarball) as tar:
            def filter_members(tar):
                """Filters members and member names:
                    - strips common prefix
                    - bans dangerous names"""
                members = tar.getmembers()
                common = os.path.commonprefix([m.name for m in members])

                # commonprefix() works a character at a time and will
                # consider "d/ab" and "d/abc" to have common prefix "d/ab";
                # check all members either start with common dir
                for m in members:
                    if not (m.name.startswith(common + '/') or
                            m.isdir() and m.name == common):
                        # commonprefix() didn't return a dir name; go up one
                        # level
                        common = os.path.dirname(common)
                        break

                for m in members:
                    if m.name == common:
                        continue
                    if m.name.startswith(common + '/'):
                        m.name = m.name[len(common + '/'):]
                    # strip leading '/', './' or '../' as many times as needed
                    m.name = re.sub(r'^(\.{0,2}/)*', r'', m.name)
                    # We mask all files to be writable to be able to easily
                    # extract on top.
                    m.mode = m.mode | 0o200
                    yield m

            tar.extractall(members=filter_members(tar), path=dst)


class Zip(FileBase):

    def __init__(self, source, source_dir, source_checksum=None,
                 source_tag=None, source_branch=None):
        super().__init__(source, source_dir, source_checksum,
                         source_tag, source_branch)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a zip source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a zip source')

    def provision(self, dst, clean_target=True, keep_zip=False):
        zip = os.path.join(self.source_dir, os.path.basename(self.source))

        if self.source_checksum:
            verify_checksum(self.source_checksum, zip)

        if clean_target:
            tmp_zip = tempfile.NamedTemporaryFile().name
            shutil.move(zip, tmp_zip)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_zip, zip)

        zipfile.ZipFile(zip).extractall(path=dst)

        if not keep_zip:
            os.remove(zip)


class Local(Base):

    def pull(self):
        if os.path.islink(self.source_dir) or os.path.isfile(self.source_dir):
            os.remove(self.source_dir)
        elif os.path.isdir(self.source_dir):
            shutil.rmtree(self.source_dir)

        source_abspath = os.path.abspath(self.source)

        def ignore(directory, files):
            if directory is source_abspath:
                snaps = glob.glob(os.path.join(directory, '*.snap'))
                if snaps:
                    snaps = [os.path.basename(s) for s in snaps]
                    return common.SNAPCRAFT_FILES + snaps
                else:
                    return common.SNAPCRAFT_FILES
            else:
                return []

        shutil.copytree(source_abspath, self.source_dir,
                        copy_function=common.link_or_copy, ignore=ignore)


def get(sourcedir, builddir, options):
    """Populate sourcedir and builddir from parameters defined in options.

    :param str sourcedir: The source directory to use.
    :param str builddir: The build directory to use.
    :param options: source options.
    """
    source_type = getattr(options, 'source_type', None)
    source_checksum = getattr(options, 'source_checksum', None)
    source_tag = getattr(options, 'source_tag', None)
    source_branch = getattr(options, 'source_branch', None)

    handler_class = _get_source_handler(source_type, options.source)
    handler = handler_class(options.source, sourcedir, source_checksum,
                            source_tag, source_branch)
    handler.pull()


def get_required_packages(options):
    """Return a list with required packages to handle the source.

    :param source: the url for the source.
    :param options: plugin options.
    """
    source = getattr(options, 'source', None)
    if not source:
        return []

    source_type = getattr(options, 'source_type', None)
    if not source_type:
        source_type = _get_source_type_from_uri(source, ignore_errors=True)

    packages = []
    if source_type == 'bzr':
        packages.append('bzr')
    elif source_type == 'git':
        packages.append('git')
    elif source_type == 'tar':
        packages.append('tar')
    elif source_type == 'hg' or source_type == 'mercurial':
        packages.append('mercurial')
    elif source_type == 'subversion' or source_type == 'svn':
        packages.append('subversion')

    return packages


_source_handler = {
    'bzr': Bazaar,
    'git': Git,
    'hg': Mercurial,
    'mercurial': Mercurial,
    'svn': Subversion,
    'subversion': Subversion,
    'tar': Tar,
    'zip': Zip,
}


def _get_source_handler(source_type, source):
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
    elif common.isurl(source) and not ignore_errors:
        raise ValueError('no handler to manage source')
    elif not os.path.isdir(source) and not ignore_errors:
        raise ValueError('local source is not a directory')

    return source_type


def verify_checksum(source_checksum, checkfile):
    if source_checksum.startswith('http'):
        response = urllib.request.urlopen(source_checksum)
        data = response.read()
        source_checksum = data.decode('utf-8')
        if (' ' in source_checksum):
            source_checksum = source_checksum.split(' ', 1)[0]
        else:
            print('No file name detected in the checksum file, perhaps an '
                  'invalid checksum file?')
    if os.path.isfile(source_checksum):
        filename = source_checksum
        try:
            filework = open(filename, 'r')
            source_checksum = filework.read()
            if (' ' in source_checksum):
                source_checksum = source_checksum.split(' ', 1)[0]
            else:
                print('No file name detected in the checksum file, perhaps an '
                      'invalid checksum file?')
        finally:
            filework.close()

    _HASH_FUNCTIONS = {
        32: hashlib.md5(),
        40: hashlib.sha1(),
        56: hashlib.sha224(),
        64: hashlib.sha256(),
        96: hashlib.sha384(),
        128: hashlib.sha512()
    }

    try:
        checksum = _HASH_FUNCTIONS[len(source_checksum)]
    except KeyError:
        raise IncompatibleOptionsError('Invalid checksum format')

    with open(checkfile, 'rb') as f:
        for chunk in iter(lambda: f.read(4096), b''):
            checksum.update(chunk)

    checksum = checksum.hexdigest()

    if checksum != source_checksum:
        raise ChecksumDoesNotMatch(
            "the checksum ( {0} ) doesn't match the file ( {1} )".format(
                source_checksum, checksum))
