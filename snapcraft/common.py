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
import urllib


COMMAND_ORDER = ["pull", "build", "stage", "snap"]
_DEFAULT_PLUGINDIR = '/usr/share/snapcraft/plugins'
_plugindir = _DEFAULT_PLUGINDIR
_DEFAULT_SCHEMADIR = '/usr/share/snapcraft/schema'
_schemadir = _DEFAULT_SCHEMADIR
_arch = None
_arch_triplet = None

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
        subprocess.check_call(['/bin/sh', f.name] + cmd, **kwargs)


def run_output(cmd, **kwargs):
    assert isinstance(cmd, list), "run command must be a list"
    # FIXME: This is gross to keep writing this, even when env is the same
    with tempfile.NamedTemporaryFile(mode='w+') as f:
        f.write(assemble_env())
        f.write('\n')
        f.write('exec $*')
        f.flush()
        return subprocess.check_output(['/bin/sh', f.name] + cmd,
                                       **kwargs).decode('utf8').strip()


def fatal():
    sys.exit(1)


def get_arch():
    global _arch
    if _arch is None:
        _arch = subprocess.check_output(
            ['dpkg-architecture', '-qDEB_BUILD_ARCH']).decode('utf8').strip()
    return _arch


def get_arch_triplet():
    global _arch_triplet
    if _arch_triplet is None:
        _arch_triplet = subprocess.check_output(
            ['dpkg-architecture', '-qDEB_BUILD_MULTIARCH']
        ).decode('utf8').strip()
    return _arch_triplet


def get_partsdir():
    return os.path.join(os.getcwd(), 'parts')


def get_stagedir():
    return os.path.join(os.getcwd(), 'stage')


def get_snapdir():
    return os.path.join(os.getcwd(), 'snap')


def set_plugindir(plugindir):
    global _plugindir
    _plugindir = plugindir


def get_plugindir():
    return _plugindir


def set_schemadir(schemadir):
    global _schemadir
    _schemadir = schemadir


def get_schemadir():
    return _schemadir


def isurl(url):
    return urllib.parse.urlparse(url).scheme != ""
