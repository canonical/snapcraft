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
      Defaults to 'release-beta2'.
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
        return """
deb http://packages.ros.org/ros/ubuntu/ {0} main
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0} main universe
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0}-updates main universe
deb http://${{prefix}}.ubuntu.com/${{suffix}}/ {0}-security main universe
deb http://${{security}}.ubuntu.com/${{suffix}} {0}-security main universe
""".format('xenial')

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['version'] = {
            'type': 'string',
            'default': 'release-beta3'
        }

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ['version']

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return []

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        # FIXME: Remove this warning once the plugin (and indeed ROS2) is
        # considered stable
        logger.warn('The ament plugin is currently in beta, its API may '
                    'break. Use at your own risk')

        self._bootstrap_dir = os.path.join(self.partdir, 'bootstrap')

        self._bootstrapper = _ros.ros2.Bootstrapper(
            version=self.options.version,
            bootstrap_path=self._bootstrap_dir,
            ubuntu_sources=self.PLUGIN_STAGE_SOURCES,
            project=self.project)

        self.stage_packages.extend(self._bootstrapper.get_stage_packages())
        self.build_packages.extend(self._bootstrapper.get_build_packages())

    def pull(self):
        super().pull()

        # Pull ros2 underlay
        self._bootstrapper.pull()

    def clean_pull(self):
        super().clean_pull()

        self._bootstrapper.clean()

    def build(self):
        super().build()

        self._prepare_build()

        self.run([
            'ament', 'build', self.sourcedir, '--build-space', self.builddir,
            '--install-space', self.installdir, '--cmake-args',
            '-DCMAKE_BUILD_TYPE=Release'])

        self._finish_build()

    def _prepare_build(self):
        # Bootstrap ros2, and migrate it into our installdir
        ros2_install_dir = self._bootstrapper.build()
        snapcraft.file_utils.link_or_copy_tree(
            ros2_install_dir, self.installdir)

        # Also rewrite the prefixes to point to the part installdir rather
        # than the bootstrap area. Otherwise Ament embeds absolute paths.
        snapcraft.file_utils.replace_in_file(
            self.installdir, re.compile(r''),
            re.compile(r'\${AMENT_CURRENT_PREFIX:=.*}'),
            '${{AMENT_CURRENT_PREFIX:={}}}'.format(self.installdir))

    def _finish_build(self):
        # Set the AMENT_CURRENT_PREFIX throughout to a sensible default,
        # removing part-specific directories that will clash with other parts.
        snapcraft.file_utils.replace_in_file(
            self.installdir, re.compile(r''),
            re.compile(r'\${AMENT_CURRENT_PREFIX:=.*}'),
            '${AMENT_CURRENT_PREFIX:=$SNAP}')

    def env(self, root):
        """Runtime environment for ROS binaries and services."""

        env = []

        # paths = common.get_library_paths(root, self.project.arch_triplet)
        # ld_library_path = formatting_utils.combine_paths(
        #     paths, prepend='', separator=':')
        #
        # env = [
        #     # This environment variable tells ROS nodes where to find ROS
        #     # master. It does not affect ROS master, however-- this is just the
        #     # default URI.
        #     'ROS_MASTER_URI=http://localhost:11311',
        #
        #     # Various ROS tools (e.g. rospack, roscore) keep a cache or a log,
        #     # and use $ROS_HOME to determine where to put them.
        #     'ROS_HOME=${SNAP_USER_DATA:-/tmp}/ros',
        #
        #     # FIXME: LP: #1576411 breaks ROS snaps on the desktop, so we'll
        #     # temporarily work around that bug by forcing the locale to
        #     # C.UTF-8.
        #     'LC_ALL=C.UTF-8',
        #
        #     # The Snapcraft Core will ensure that we get a good LD_LIBRARY_PATH
        #     # overall, but it defines it after this function runs. Some ROS
        #     # tools will cause binaries to be run when we source the setup.sh,
        #     # below, so we need to have a sensible LD_LIBRARY_PATH before then.
        #     'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{}'.format(ld_library_path),
        # ]
        #
        # # There's a chicken and egg problem here, everything run get's an
        # # env built, even package installation, so the first runs for these
        # # will likely fail.
        # try:
        #     # The ROS packaging system tools (e.g. rospkg, etc.) don't go
        #     # into the ROS install path (/opt/ros/$distro), so we need the
        #     # PYTHONPATH to include the dist-packages in /usr/lib as well.
        #     #
        #     # Note: Empty segments in PYTHONPATH are interpreted as `.`, thus
        #     # adding the current working directory to the PYTHONPATH. That is
        #     # not desired in this situation, so take proper precautions when
        #     # expanding PYTHONPATH: only add it if it's not empty.
        #     env.append('PYTHONPATH={}${{PYTHONPATH:+:$PYTHONPATH}}'.format(
        #         common.get_python2_path(root)))
        # except errors.SnapcraftEnvironmentError as e:
        #     logger.debug(e)

        # The setup.sh we source below requires the in-snap python. Here we
        # make sure it's in the PATH before it's run.
        # env.append('PATH=$PATH:{}/usr/bin'.format(root))

        env.append('PYTHONUSERBASE={}'.format(root))
        env.append('PYTHONPATH={}:{}:{}'.format(
            os.path.join(
                os.path.sep, 'usr', 'lib', 'python3', 'dist-packages'),
            os.path.join(root, 'usr', 'lib', 'python3', 'dist-packages'),
            os.path.join(root, 'lib', 'python3', 'dist-packages')))
        script = self._source_setup_sh(root)

        # Each of these lines is prepended with an `export` when the
        # environment is actually generated. In order to inject real shell code
        # we have to hack it in by appending it on the end of an item already
        # in the environment. FIXME: There should be a better way to do this.
        env[-1] = env[-1] + '\n\n' + script

        return env

    def _source_setup_sh(self, root):
        return textwrap.dedent('''
            if [ -f {ros_setup} ]; then
                . {ros_setup}
            fi
        ''').format(
            ros_setup=os.path.join(root, 'setup.sh'))

        # We need to source ROS's setup.sh at this point. However, it accepts
        # arguments (thus will parse $@), and we really don't want it to, since
        # $@ in this context will be meant for the app being launched
        # (LP: #1660852). So we'll backup all args, source the setup.sh, then
        # restore all args for the wrapper's `exec` line.
        # return textwrap.dedent('''
        #     # Shell quote arbitrary string by replacing every occurrence of '
        #     # with '\\'', then put ' at the beginning and end of the string.
        #     # Prepare yourself, fun regex ahead.
        #     quote()
        #     {{
        #         for i; do
        #             printf %s\\\\n "$i" | sed "s/\'/\'\\\\\\\\\'\'/g;1s/^/\'/;\$s/\$/\' \\\\\\\\/"
        #         done
        #         echo " "
        #     }}
        #
        #     BACKUP_ARGS=$(quote "$@")
        #     set --
        #     {}
        #     eval "set -- $BACKUP_ARGS"
        # ''').format(source_script)  # noqa
