# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 James Beedy <jamesbeedy@gmail.com>
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

"""The ruby plugin is useful for ruby based parts.
The plugin uses gem to install dependencies from a `Gemfile`.
This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.
Additionally, this plugin uses the following plugin-specific keywords:
    - gems:
      (list)
      A list of gems to install.
    - ruby-version:
      (string)
      The version of ruby you want this snap to run.
"""
import os
import re
import platform

from snapcraft import BasePlugin
from snapcraft.sources import Tar


class RubyPlugin(BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['ruby-version'] = {
            'type': 'string',
            'default': '2.3.1'
        }
        schema['properties']['gems'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': []
        }
        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['ruby-version', 'gems']

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._ruby_version = self.options.ruby_version
        self._ruby_part_dir = os.path.join(self.partdir, 'ruby')
        self._ruby_download_url = \
            'https://cache.ruby-lang.org/pub/ruby/ruby-{}.tar.gz'.format(
                self._ruby_version)
        self._ruby_tar = Tar(self._ruby_download_url, self._ruby_part_dir)
        self._gems = self.options.gems or []
        self._install_bundler = False

        self.build_packages.extend(['gcc', 'g++', 'make', 'zlib1g-dev',
                                    'libssl-dev', 'libreadline-dev'])

        version_map = [('2.%d.[0-9]' % i, '2.%d.0' % i) for i in range(5)]

        for version_regex, version_dir in version_map:
            if re.compile(version_regex).match(self._ruby_version):
                self._ruby_version_dir = version_dir
                break

    def pull(self):
        super().pull()
        os.makedirs(self._ruby_part_dir, exist_ok=True)
        self._ruby_tar.download()
        self._ruby_install(builddir=self._ruby_part_dir)
        self._gem_install()
        if self._install_bundler:
            self._bundle_install()

    def env(self, root):
        env = super().env(root)
        env.append('PATH={}:{}'.format(
            os.path.join(root, 'bin'), os.environ['PATH']))
        env.append('RUBYPATH={}'.format(os.path.join(root, 'bin')))
        rubydir = os.path.join(root, 'lib', 'ruby')
        rubylib = os.path.join(rubydir, self._ruby_version_dir)
        env.append('RUBYLIB={}:{}'.format(
            rubylib, os.path.join(
                rubylib, '{}-linux'.format(platform.machine()))))
        env.append('GEM_HOME={}'.format(
            os.path.join(rubydir, 'gems', self._ruby_version_dir)))
        env.append('GEM_PATH={}'.format(
            os.path.join(rubydir, 'gems', self._ruby_version_dir)))
        return env

    def _ruby_install(self, builddir):
        self._ruby_tar.provision(
            builddir, clean_target=False, keep_tarball=True)
        self.run(['./configure', '--disable-install-rdoc', '--prefix=/'],
                 cwd=builddir)
        self.run(['make', '-j{}'.format(self.parallel_build_count)],
                 cwd=builddir)
        self.run(['make', 'install', 'DESTDIR={}'.format(self.installdir)],
                 cwd=builddir)

    def _gem_install(self):
        if os.path.exists('Gemfile'):
            self._install_bundler = True
            self._gems = self._gems + ['bundler']
        if self._gems:
            gem_install_cmd = [os.path.join(self.installdir, 'bin', 'ruby'),
                               os.path.join(self.installdir, 'bin', 'gem'),
                               'install']
            self.run(gem_install_cmd + self._gems)

    def _bundle_install(self):
        bundle_install_cmd = [os.path.join(self.installdir, 'bin', 'ruby'),
                              os.path.join(self.installdir, 'bin', 'bundle'),
                              'install']
        self.run(bundle_install_cmd)
