# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

# Data/methods shared between plugins and snapcraft

import os
import subprocess
import tempfile

env = []


def assembleEnv():
    return '\n'.join(['export ' + e for e in env])


def run(cmd, **kwargs):
    # FIXME: This is gross to keep writing this, even when env is the same
    if isinstance(cmd, list):
        cmd = ' '.join(cmd)
    with tempfile.NamedTemporaryFile(mode='w+') as f:
        f.write(assembleEnv())
        f.write('\n')
        f.write('exec ' + cmd)
        f.flush()
        return subprocess.call(['/bin/sh', f.name], **kwargs) == 0


def log(msg, file=None):
    print('\033[01m' + msg + '\033[0m', file=None)

commandOrder = ["pull", "build", "stage", "snap"]
stagedir = os.path.join(os.getcwd(), "stage")
snapdir = os.path.join(os.getcwd(), "snap")
