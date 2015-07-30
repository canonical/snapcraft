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
import sys
import tempfile


COMMAND_ORDER = ["pull", "build", "stage", "snap"]
_DEFAULT_PLUGINDIR = '/usr/share/snapcraft/plugins'
_plugindir = _DEFAULT_PLUGINDIR

env = []


def assemble_env():
    return '\n'.join(['export ' + e for e in env])


def run(cmd, **kwargs):
    assert isinstance(cmd, list), "run command must be a list"
    # FIXME: This is gross to keep writing this, even when env is the same
    with tempfile.NamedTemporaryFile(mode='w+') as f:
        f.write(assemble_env())
        f.write('\n')
        f.write('exec $*')
        f.flush()
        return subprocess.call(['/bin/sh', f.name] + cmd, **kwargs) == 0


def fatal(msg):
    sys.exit(1)


def get_stagedir():
    return os.path.join(os.getcwd(), 'stage')


def get_snapdir():
    return os.path.join(os.getcwd(), 'snap')


def set_plugindir(plugindir):
    global _plugindir
    _plugindir = plugindir


def get_plugindir():
    return _plugindir

def get_arch():
    return run(['dpkg-architecture', '-qDEB_HOST_ARCH_CPU'])

def get_arch_triplet():
    return run(['dpkg-architecture', '-qDEB_HOST_MULTIARCH'])
