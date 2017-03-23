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

import contextlib
import os
import subprocess


@contextlib.contextmanager
def return_to_cwd():
    cwd = os.getcwd()
    try:
        yield
    finally:
        os.chdir(cwd)


def call(cmd):
    subprocess.check_call(cmd, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)


def call_with_output(cmd):
    return subprocess.check_output(cmd).decode('utf-8').strip()


def create_bzr_repo(name):
    with return_to_cwd():
        os.makedirs(name)
        os.chdir(name)
        call(['bzr', 'init'])
        call(['bzr', 'whoami', 'Test User <test.user@example.com>'])
        with open('testing', 'w') as fp:
            fp.write('testing')

        call(['bzr', 'add', 'testing'])
        call(['bzr', 'commit', '-m', 'testing'])
        call(['bzr', 'tag', 'feature-tag'])
        revno = call_with_output(['bzr', 'revno'])

        return revno
