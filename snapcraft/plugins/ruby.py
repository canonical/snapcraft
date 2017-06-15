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
from shutil import rmtree
from os import makedirs, environ
from os.path import exists, join, abspath

from snapcraft import BasePlugin
from snapcraft.sources import Tar


class RubyPlugin(BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['ruby-version'] = {
            'type': 'string',
            'default': "2.3.1"
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
        return ['ruby-version']

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._ruby_version = self.options.ruby_version
        self._rubylib = join(self.installdir, 'lib', 'ruby')
        self._ruby_dir = join(self.partdir, 'ruby')
        self._gem_path = join(self._rubylib, 'gems', self._ruby_version, 'gems')
        self._gem_home = self._gem_path
        self._ruby_download_url = \
            ('https://cache.ruby-lang.org/pub/ruby/ruby-%s.tar.gz' % \
             self.options.ruby_version)
        self._ruby_tar = Tar(self._ruby_download_url, self._ruby_dir)
        self._gems = self.options.gems
        self._install_bundler = False

        pkgs = ['gcc', 'g++', 'make', 'zlib1g-dev',
                'libssl-dev', 'libreadline-dev']
        for pkg in pkgs:
            self.build_packages.append(pkg)

    def pull(self):
        super().pull()
        makedirs(self._ruby_dir, exist_ok=True)
        self._ruby_tar.download()
        
    def build(self):
        super().build()
        self._ruby_install(builddir=self._ruby_dir)
        self._gem_install()
        if self._install_bundler:
            self._bundle_install()

    def _env(self):
        """Ruby env vars."""
        env = environ.copy()
        env['RUBYLIB'] = self._rubylib
        env['GEM_PATH'] = self._gem_path
        env['GEM_HOME'] = self._gem_home
        return env

    def _ruby_install(self, builddir):
        self._ruby_tar.provision(
            builddir, clean_target=False, keep_tarball=True)
        self.run(['./configure', '--disable-install-rdoc', '--prefix=/'], cwd=builddir)
        self.run(['make', '-j{}'.format(self.parallel_build_count)], cwd=builddir)
        self.run(['make', 'install', 'DESTDIR={}'.format(self.installdir)], cwd=builddir)

    def _gem_install(self):
        if exists(join(self.builddir, 'Gemfile')):
            self._install_bundler = True
            self._gems = self._gems + ['bundler']
        for gem in self._gems:
            self.run(['gem', 'install'] + [gem], env=self._env())

    def _bundle_install(self):
        self.run(['bundle', 'install'], env=self._env())
