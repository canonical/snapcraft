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

"""The kbuild plugin is used for building kbuild based projects as snapcraft
parts.

This plugin is based on the snapcraft.BasePlugin and supports the properties
provided by that plus the following kbuild specific options with semantics as
explained above:

    - kdefconfig:
      (list of kdefconfigs)
      defconfig target to use as the base configuration. default: "defconfig"

    - kconfigfile:
      (filepath)
      path to file to use as base configuration. If provided this option wins
      over kdefconfig. default: None

    - kconfigs
      (list of strings)
      explicit list of configs to force; this will override the configs that
      were set as base through kdefconfig and kconfigfile and dependent configs
      will be fixed using the defaults encoded in the kbuild config
      definitions.  If you don't want default for one or more implicit configs
      coming out of these, just add them to this list as well.

The plugin applies your selected defconfig first by running

    make defconfig

and then uses the kconfigs flag to augment the resulting config by prepending
the configured kconfigs values to the .config and running

    "yes" "" | make oldconfig

to create an updated .config file.

If kconfigfile is provided this plugin will use the provided config file
wholesale as the starting point instead of make $kdefconfig. In case user
configures both a kdefconfig as well as kconfigfile, kconfigfile approach will
be used.
"""

import logging
import os
import shutil
import subprocess

from snapcraft import BasePlugin


logger = logging.getLogger(__name__)


class KBuildPlugin(BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['kdefconfig'] = {
            'type': 'array',
            'default': ['defconfig'],
        }

        schema['properties']['kconfigfile'] = {
            'type': 'string',
            'default': None,
        }

        schema['properties']['kconfigs'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['kdefconfig', 'kconfigfile', 'kconfigs']

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('make')

        self.make_targets = []
        self.make_install_targets = ['install']
        self.make_cmd = [
            'make', '-j{}'.format(self.parallel_build_count)]
        if logger.isEnabledFor(logging.DEBUG):
            self.make_cmd.append('V=1')

    def do_base_config(self, config_path):
        # if kconfigfile is provided use that
        # otherwise use defconfig to seed the base config
        if self.options.kconfigfile is None:
            # we need to run this with -j1, unit tests are a good defense here.
            make_cmd = self.make_cmd.copy()
            make_cmd[1] = '-j1'
            self.run(make_cmd + self.options.kdefconfig)
        else:
            shutil.copy(self.options.kconfigfile, config_path)

    def do_patch_config(self, config_path):
        # prepend the generated file with provided kconfigs
        #  - concat kconfigs to buffer
        #  - read current .config and append
        #  - write out to disk
        if not self.options.kconfigs:
            return

        config = '\n'.join(self.options.kconfigs)

        # note that prepending and appending the overrides seems
        # only way to convince all kbuild versions to pick up the
        # configs during oldconfig in .config
        with open(config_path, 'r') as f:
            config = (
                '{config_override}\n\n{config}\n{config_override}\n'.format(
                    config_override=config, config=f.read()))

        with open(config_path, 'w') as f:
            f.write(config)

    def do_remake_config(self):
        # update config to include kconfig amendments using oldconfig
        cmd = 'yes "" | {} oldconfig'.format(' '.join(self.make_cmd))
        subprocess.check_call(cmd, shell=True, cwd=self.builddir)

    def do_build(self):
        # build the software
        self.run(self.make_cmd + self.make_targets)

    def do_install(self):
        # install to installdir
        self.run(self.make_cmd +
                 ['CONFIG_PREFIX={}'.format(self.installdir)] +
                 self.make_install_targets)

    def build(self):
        super().build()

        config_path = os.path.join(self.builddir, '.config')

        self.do_base_config(config_path)
        self.do_patch_config(config_path)
        self.do_remake_config()
        self.do_build()
        self.do_install()
