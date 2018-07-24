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

"""The ament plugin is useful for building ROS2 parts.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - version:
      (string)
      The ROS2 version required by this system. This relates to the ros2 tags.
      Defaults to 'release-beta3'.
"""

import logging
import os
import re
import textwrap

import snapcraft
from . import _ros

logger = logging.getLogger(__name__)


class AmentPlugin(snapcraft.BasePlugin):
    @property
    def PLUGIN_STAGE_SOURCES(self):
        """Specify the sources used for stage-packages.

        Note the ROS archive in particular. Build-packages do not come from
        here, but stage-packages do.
        """

        return """
deb http://packages.ros.org/ros/ubuntu/ {0} main
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0} main universe
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0}-updates main universe
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0}-security main universe
deb http://${{security}}.ubuntu.com/${{suffix}} {0}-security main universe
""".format(
            "xenial"
        )

    @classmethod
    def schema(cls):
        """Specify the jsonschema of the supported options.

        Snapcraft will use this schema to validate the YAML when this plugin is
        used.
        """

        schema = super().schema()

        schema["properties"]["version"] = {"type": "string", "default": "release-beta3"}

        return schema

    @classmethod
    def get_pull_properties(cls):
        """Inform Snapcraft of the properties associated with pulling.

        If these change in the YAML, Snapcraft will consider the pull step
        dirty.
        """
        return ["version"]

    def __init__(self, name, options, project):
        """Initialize a new AmentPlugin.

        :param str name: The name of this part
        :param object options: Objectified version of the properties for this
                               part
        :param project: Instance of ProjectOptions for project-wide settings
        :type project: snapcraft.ProjectOptions
        """
        super().__init__(name, options, project)

        # FIXME: Remove this warning once the plugin (and indeed ROS2) is
        # considered stable
        logger.warn(
            "The ament plugin is currently in beta, its API may "
            "break. Use at your own risk"
        )

        self._bootstrap_dir = os.path.join(self.partdir, "bootstrap")

        self._bootstrapper = _ros.ros2.Bootstrapper(
            version=self.options.version,
            bootstrap_path=self._bootstrap_dir,
            ubuntu_sources=self.PLUGIN_STAGE_SOURCES,
            project=self.project,
        )

        self.stage_packages.extend(self._bootstrapper.get_stage_packages())
        self.build_packages.extend(self._bootstrapper.get_build_packages())

    def pull(self):
        """Pull the provided source, as well as the ROS2 underlay."""

        super().pull()

        # Pull ros2 underlay
        self._bootstrapper.pull()

    def clean_pull(self):
        """Clean the fetched source and ROS2 underlay."""

        super().clean_pull()

        self._bootstrapper.clean()

    def build(self):
        """Bootstrap the ROS2 underlay, then use it to build the source.

        This step can take a while.
        """

        super().build()

        self._prepare_build()

        self.run(
            [
                "ament",
                "build",
                self.sourcedir,
                "--build-space",
                self.builddir,
                "--install-space",
                self.installdir,
                "--cmake-args",
                "-DCMAKE_BUILD_TYPE=Release",
            ],
            env=self._build_env(),
        )

        self._finish_build()

    def _prepare_build(self):
        # Bootstrap ros2, and migrate it into our installdir
        ros2_install_dir = self._bootstrapper.build()
        snapcraft.file_utils.link_or_copy_tree(ros2_install_dir, self.installdir)

        # Also rewrite the prefixes to point to the part installdir rather
        # than the bootstrap area. Otherwise Ament embeds absolute paths which
        # makes the resulting code non-relocatable.
        snapcraft.file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"\${AMENT_CURRENT_PREFIX:=.*}"),
            "${{AMENT_CURRENT_PREFIX:={}}}".format(self.installdir),
        )

    def _finish_build(self):
        # Set the AMENT_CURRENT_PREFIX throughout to a sensible default,
        # removing part-specific directories that will clash with other parts.
        snapcraft.file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"\${AMENT_CURRENT_PREFIX:=.*}"),
            "${AMENT_CURRENT_PREFIX:=$SNAP}",
        )

    def env(self, root):
        """Runtime environment for ROS2 apps."""

        env = []

        env.append('PYTHONUSERBASE="{}"'.format(root))
        env.append(
            'PYTHONPATH="{}:{}${{PYTHONPATH:+:$PYTHONPATH}}"'.format(
                os.path.join(root, "usr", "lib", "python3", "dist-packages"),
                os.path.join(root, "lib", "python3", "dist-packages"),
            )
        )

        # Each of these lines is prepended with an `export` when the
        # environment is actually generated. In order to inject real shell code
        # we have to hack it in by appending it on the end of an item already
        # in the environment. FIXME: There should be a better way to do this.
        env[-1] = env[-1] + "\n\n" + self._source_setup_sh(root)

        return env

    def _build_env(self):
        env = os.environ.copy()
        env["PYTHONPATH"] = os.path.join(
            os.path.sep, "usr", "lib", "python3", "dist-packages"
        )
        return env

    def _source_setup_sh(self, root):
        return textwrap.dedent(
            """
            if [ -f {ros_setup} ]; then
                . {ros_setup}
            fi
        """
        ).format(ros_setup=os.path.join(root, "setup.sh"))
