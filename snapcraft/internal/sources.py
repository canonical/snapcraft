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

import copy
import glob
import logging
import os
import os.path
import stat
import re
import requests
import shutil
import subprocess
import tempfile
import tarfile
import zipfile

import apt_inst
import libarchive

from snapcraft.internal import common
from snapcraft import file_utils
from snapcraft.internal.errors import MissingCommandError
from snapcraft.internal.indicators import download_requests_stream


logging.getLogger('urllib3').setLevel(logging.CRITICAL)


class IncompatibleOptionsError(Exception):

    def __init__(self, message):
        self.message = message


def _check_for_command(command):
    if not shutil.which(command):
        raise MissingCommandError([command])


class Base:

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None,
                 command=None):
        self.source = source
        self.source_dir = source_dir
        self.source_tag = source_tag
        self.source_commit = source_commit
        self.source_branch = source_branch
        self.source_depth = source_depth

        self.command = command

        if self.command:
            _check_for_command(self.command)


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

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)

    def download(self):
        super().download()
        st = os.stat(self.file)
        os.chmod(self.file, st.st_mode | stat.S_IEXEC)


class Bazaar(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'bzr')
        if source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a bzr source')
        if source_depth:
            raise IncompatibleOptionsError(
                'can\'t specify source-depth for a bzr source')
        if source_tag and source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-commit for '
                'a bzr source')

    def pull(self):
        tag_opts = []
        if self.source_tag:
            tag_opts = ['-r', 'tag:' + self.source_tag]
        if self.source_commit:
            tag_opts = ['-r', self.source_commit]
        if os.path.exists(os.path.join(self.source_dir, '.bzr')):
            cmd = [self.command, 'pull'] + tag_opts + \
                  [self.source, '-d', self.source_dir]
        else:
            os.rmdir(self.source_dir)
            cmd = [self.command, 'branch'] + tag_opts + \
                  [self.source, self.source_dir]

        subprocess.check_call(cmd)


class Git(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'git')
        if source_tag and source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for '
                'a git source')
        if source_tag and source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-commit for '
                'a git source')
        if source_branch and source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify both source-branch and source-commit for '
                'a git source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.git')):
            refspec = 'HEAD'
            if self.source_branch:
                refspec = 'refs/heads/' + self.source_branch
            elif self.source_tag:
                refspec = 'refs/tags/' + self.source_tag
            elif self.source_commit:
                refspec = self.source_commit

            # Pull changes to this repository and any submodules.
            subprocess.check_call([self.command, '-C', self.source_dir,
                                   'pull', '--recurse-submodules=yes',
                                   self.source, refspec])

            # Merge any updates for the submodules (if any).
            subprocess.check_call([self.command, '-C', self.source_dir,
                                   'submodule', 'update'])
        else:
            command = [self.command, 'clone', '--recursive']
            if self.source_tag or self.source_branch:
                command.extend([
                    '--branch', self.source_tag or self.source_branch])
            if self.source_depth:
                command.extend(['--depth', str(self.source_depth)])
            subprocess.check_call(command + [self.source, self.source_dir])

            if self.source_commit:
                subprocess.check_call([self.command, '-C', self.source_dir,
                                       'checkout', self.source_commit])


class Mercurial(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'hg')
        if source_tag and source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-branch for a '
                'mercurial source')
        if source_tag and source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify both source-tag and source-commit for a '
                'mercurial source')
        if source_branch and source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify both source-branch and source-commit for a '
                'mercurial source')
        if source_depth:
            raise IncompatibleOptionsError(
                'can\'t specify source-depth for a mercurial source')

    def pull(self):
        if os.path.exists(os.path.join(self.source_dir, '.hg')):
            ref = []
            if self.source_tag:
                ref = ['-r', self.source_tag]
            elif self.source_commit:
                ref = ['-r', self.source_commit]
            elif self.source_branch:
                ref = ['-b', self.source_branch]
            cmd = [self.command, 'pull'] + ref + [self.source, ]
        else:
            ref = []
            if self.source_tag or self.source_branch or self.source_commit:
                ref = ['-u', self.source_tag or self.source_branch or
                       self.source_commit]
            cmd = [self.command, 'clone'] + ref + [self.source,
                                                   self.source_dir]

        subprocess.check_call(cmd)


class Subversion(Base):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth, 'svn')
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
        if source_depth:
            raise IncompatibleOptionsError(
                'can\'t specify source-depth for a Subversion source')

    def pull(self):
        opts = []

        if self.source_commit:
            opts = ["-r", self.source_commit]

        if os.path.exists(os.path.join(self.source_dir, '.svn')):
            subprocess.check_call(
                [self.command, 'update'] + opts, cwd=self.source_dir)
        else:
            if os.path.isdir(self.source):
                subprocess.check_call(
                    [self.command, 'checkout',
                     'file://{}'.format(os.path.abspath(self.source)),
                     self.source_dir] + opts)
            else:
                subprocess.check_call(
                    [self.command, 'checkout', self.source, self.source_dir] +
                    opts)


class Tar(FileBase):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a tar source')
        elif source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify a source-commit for a tar source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a tar source')
        if source_depth:
            raise IncompatibleOptionsError(
                'can\'t specify a source-depth for a tar source')

    def provision(self, dst, clean_target=True, keep_tarball=False):
        # TODO add unit tests.
        tarball = os.path.join(self.source_dir, os.path.basename(self.source))

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
                    self._strip_prefix(common, m)
                    # We mask all files to be writable to be able to easily
                    # extract on top.
                    m.mode = m.mode | 0o200
                    yield m

            tar.extractall(members=filter_members(tar), path=dst)

    def _strip_prefix(self, common, member):
        if member.name.startswith(common + '/'):
            member.name = member.name[len(common + '/'):]
        # strip leading '/', './' or '../' as many times as needed
        member.name = re.sub(r'^(\.{0,2}/)*', r'', member.name)
        # do the same for linkname if this is a hardlink
        if member.islnk() and not member.issym():
            if member.linkname.startswith(common + '/'):
                member.linkname = member.linkname[len(common + '/'):]
            member.linkname = re.sub(r'^(\.{0,2}/)*', r'', member.linkname)


class Zip(FileBase):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a zip source')
        elif source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify a source-commit for a zip source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a zip source')
        if source_depth:
            raise IncompatibleOptionsError(
                'can\'t specify a source-depth for a zip source')

    def provision(self, dst, clean_target=True, keep_zip=False):
        zip = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_zip = tempfile.NamedTemporaryFile().name
            shutil.move(zip, tmp_zip)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_zip, zip)

        zipfile.ZipFile(zip).extractall(path=dst)

        if not keep_zip:
            os.remove(zip)


class Deb(FileBase):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a deb source')
        elif source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify a source-commit for a deb source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a deb source')

    def provision(self, dst, clean_target=True, keep_deb=False):
        deb_file = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_deb = tempfile.NamedTemporaryFile().name
            shutil.move(deb_file, tmp_deb)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_deb, deb_file)

        deb = apt_inst.DebFile(deb_file)
        deb.data.extractall(dst)

        if not keep_deb:
            os.remove(deb_file)


class Rpm(FileBase):

    def __init__(self, source, source_dir, source_tag=None, source_commit=None,
                 source_branch=None, source_depth=None):
        super().__init__(source, source_dir, source_tag, source_commit,
                         source_branch, source_depth)
        if source_tag:
            raise IncompatibleOptionsError(
                'can\'t specify a source-tag for a rpm source')
        elif source_commit:
            raise IncompatibleOptionsError(
                'can\'t specify a source-commit for a rpm source')
        elif source_branch:
            raise IncompatibleOptionsError(
                'can\'t specify a source-branch for a rpm source')

    def provision(self, dst, clean_target=True, keep_rpm=False):
        rpm_file = os.path.join(self.source_dir, os.path.basename(self.source))

        if clean_target:
            tmp_rpm = tempfile.NamedTemporaryFile().name
            shutil.move(rpm_file, tmp_rpm)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_rpm, rpm_file)

        # Ensure dst does not have trailing slash
        dst = dst.rstrip('/')
        # Open the RPM file and extract it to destination
        with libarchive.file_reader(rpm_file) as rpm:
            for rpm_file_entry in rpm:
                # Binary RPM archive data has paths starting with ./ to support
                # relocation if enabled in the building of RPMs
                rpm_file_entrypath = rpm_file_entry.pathname.lstrip('./')
                rpm_file_entrypath = rpm_file_entrypath.lstrip('/')
                rpm_file_entry.pathname = os.path.join(dst, rpm_file_entrypath)
                # XXX: libarchive frees the entry at the end of loop iterations
                # See https://github.com/Changaco/python-libarchive-c/issues/43
                libarchive.extract.extract_entries([rpm_file_entry])

        if not keep_rpm:
            os.remove(rpm_file)


class Local(Base):

    def pull(self):
        if os.path.islink(self.source_dir) or os.path.isfile(self.source_dir):
            os.remove(self.source_dir)
        elif os.path.isdir(self.source_dir):
            shutil.rmtree(self.source_dir)

        source_abspath = os.path.abspath(self.source)

        def ignore(directory, files):
            if directory is source_abspath:
                ignored = copy.copy(common.SNAPCRAFT_FILES)
                relative_cwd = os.path.basename(os.getcwd())
                if os.path.join(directory, relative_cwd) == os.getcwd():
                    # Source is a parent of the working directory.
                    # Do not recursively copy it into itself.
                    ignored.append(relative_cwd)
                snaps = glob.glob(os.path.join(directory, '*.snap'))
                if snaps:
                    snaps = [os.path.basename(s) for s in snaps]
                    ignored += snaps
                return ignored
            else:
                return []

        shutil.copytree(source_abspath, self.source_dir,
                        copy_function=file_utils.link_or_copy, ignore=ignore)


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

    handler_class = _get_source_handler(source_type, options.source)
    handler = handler_class(options.source, sourcedir, **source_attributes)
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
    elif source.endswith('deb'):
        source_type = 'deb'
    elif source.endswith('rpm'):
        source_type = 'rpm'
    elif common.isurl(source) and not ignore_errors:
        raise ValueError('no handler to manage source')
    elif not os.path.isdir(source) and not ignore_errors:
        raise ValueError('local source is not a directory')

    return source_type
