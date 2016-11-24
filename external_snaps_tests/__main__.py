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

"""Snapcraft external snaps tests.

This will clone the external repository, search for snapcraft.yaml files and
snap the packages.

Usage:
  external_snaps_tests REPO_URL [--cleanbuild] [--keep-dir]

Options:
  --cleanbuild  Build the snaps in a clean LXC container.
  --keep-dir    Do not remove the temporary directory where the repository was
                cloned and snapped.

"""

import os
import shutil
import subprocess
import sys
import tempfile

import docopt


def main():
    arguments = docopt.docopt(__doc__)
    repo = arguments['REPO_URL']
    cleanbuild = arguments['--cleanbuild']
    keep_dir = arguments['--keep-dir']
    if _is_git(repo):
        if shutil.which('git'):
            path = _git_clone(repo)
            _build_snaps(path, cleanbuild, keep_dir)
        else:
            sys.exit('Please install git.')
    else:
        sys.exit('Unsupported repository.')


def _is_git(repo):
    return (repo.startswith('https://github.com/') or
            repo.startswith('git://') or
            repo.startswith('https://git.launchpad.net/'))


def _git_clone(url):
    temp_dir = tempfile.mkdtemp(prefix='snapcraft-')
    command = ['git', 'clone', url, temp_dir]
    print(' '.join(command))
    subprocess.check_call(command)
    return temp_dir


def _build_snaps(path, cleanbuild=False, keep_dir=False):
    try:
        for dirpath, _, filenames in os.walk(path):
            if 'snapcraft.yaml' in filenames or '.snapcraft.yaml' in filenames:
                _build_snap(dirpath, cleanbuild, keep_dir)
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)
    finally:
        if keep_dir:
            print(
                'You can inspect the built project repository in {}'.format(
                    path))
        else:
            shutil.rmtree(path)


def _build_snap(path, cleanbuild=False, keep_dir=False):
    snapcraft = os.path.abspath(os.path.join('bin', 'snapcraft'))
    print('Updating the parts cache...')
    subprocess.check_call([snapcraft, 'update'])
    print('Snapping {}'.format(path))
    command = [snapcraft, '-d']
    if cleanbuild:
        command.append('cleanbuild')
    print(' '.join(command))
    subprocess.check_call(command, cwd=path)


if __name__ == '__main__':
    main()
