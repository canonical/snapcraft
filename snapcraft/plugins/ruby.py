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
    - use-bundler
      (boolean)
      Use bundler to install gems from a Gemfile (defaults 'false').
    - ruby-version:
      (string)
      The version of ruby you want this snap to run.
"""

import glob
import logging
import os
import re

from snapcraft import BasePlugin, file_utils
from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.sources import Tar


logger = logging.getLogger(__name__)


class RubyPlugin(BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["use-bundler"] = {"type": "boolean", "default": False}
        schema["properties"]["ruby-version"] = {"type": "string", "default": "2.4.2"}
        schema["properties"]["gems"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["ruby-version", "gems", "use-bundler"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        # Beta Warning
        # Remove this comment and warning once ruby plugin is stable.
        logger.warn(
            "The ruby plugin is currently in beta, "
            "its API may break. Use at your own risk"
        )

        self._ruby_version = options.ruby_version
        self._ruby_part_dir = os.path.join(self.partdir, "ruby")
        self._ruby_download_url = "https://cache.ruby-lang.org/pub/ruby/ruby-{}.tar.gz".format(
            self._ruby_version
        )
        self._ruby_tar = Tar(self._ruby_download_url, self._ruby_part_dir)
        self._gems = options.gems or []

        self.build_packages.extend(
            ["gcc", "g++", "make", "zlib1g-dev", "libssl-dev", "libreadline-dev"]
        )

    def snap_fileset(self):
        fileset = super().snap_fileset()
        fileset.append("-include/")
        fileset.append("-share/")
        return fileset

    def pull(self):
        super().pull()
        os.makedirs(self._ruby_part_dir, exist_ok=True)

        logger.info("Fetching ruby {}...".format(self._ruby_version))
        self._ruby_tar.download()

        logger.info("Building/installing ruby...")
        self._ruby_install(builddir=self._ruby_part_dir)

        self._gem_install()
        if self.options.use_bundler:
            self._bundle_install()

    def env(self, root):
        env = super().env(root)

        for key, value in self._env_dict(root).items():
            env.append('{}="{}"'.format(key, value))

        return env

    def _env_dict(self, root):
        env = dict()
        rubydir = os.path.join(root, "lib", "ruby")

        # Patch versions of ruby continue to use the minor version's RUBYLIB,
        # GEM_HOME, and GEM_PATH. Fortunately there should just be one, so we
        # can detect it by globbing instead of trying to determine what the
        # minor version is programmatically
        versions = glob.glob(os.path.join(rubydir, "gems", "*"))

        # Before Ruby has been pulled/installed, no versions will be found.
        # If that's the case, we won't define any Ruby-specific variables yet
        if len(versions) == 1:
            ruby_version = os.path.basename(versions[0])

            rubylib = os.path.join(rubydir, ruby_version)

            # Ruby uses some pretty convoluted rules for determining its
            # arch-specific RUBYLIB. Rather than try and duplicate that logic
            # here, let's just look for a file that we know is in there:
            # rbconfig.rb. There should only be one.
            paths = glob.glob(os.path.join(rubylib, "*", "rbconfig.rb"))
            if len(paths) != 1:
                raise SnapcraftEnvironmentError(
                    "Expected a single rbconfig.rb, but found {}".format(len(paths))
                )

            env["RUBYLIB"] = "{}:{}".format(rubylib, os.path.dirname(paths[0]))
            env["GEM_HOME"] = os.path.join(rubydir, "gems", ruby_version)
            env["GEM_PATH"] = os.path.join(rubydir, "gems", ruby_version)
        elif len(versions) > 1:
            raise SnapcraftEnvironmentError(
                "Expected a single Ruby version, but found {}".format(len(versions))
            )

        return env

    def _run(self, command, **kwargs):
        """Regenerate the build environment, then run requested command.

        Without this function, the build environment would not be regenerated
        and thus the newly installed Ruby would not be discovered.
        """

        env = os.environ.copy()
        env.update(self._env_dict(self.installdir))
        self.run(command, env=env, **kwargs)

    def _ruby_install(self, builddir):
        self._ruby_tar.provision(builddir, clean_target=False, keep_tarball=True)
        self._run(["./configure", "--disable-install-rdoc", "--prefix=/"], cwd=builddir)
        self._run(["make", "-j{}".format(self.parallel_build_count)], cwd=builddir)
        self._run(
            ["make", "install", "DESTDIR={}".format(self.installdir)], cwd=builddir
        )
        # Fix all shebangs to use the in-snap ruby
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"^#!.*ruby"),
            r"#!/usr/bin/env ruby",
        )

    def _gem_install(self):
        if self.options.use_bundler:
            self._gems = self._gems + ["bundler"]
        if self._gems:
            logger.info("Installing gems...")
            gem_install_cmd = [
                os.path.join(self.installdir, "bin", "ruby"),
                os.path.join(self.installdir, "bin", "gem"),
                "install",
                "--env-shebang",
            ]
            self._run(gem_install_cmd + self._gems)

    def _bundle_install(self):
        bundle_install_cmd = [
            os.path.join(self.installdir, "bin", "ruby"),
            os.path.join(self.installdir, "bin", "bundle"),
            "install",
        ]
        self._run(bundle_install_cmd)
