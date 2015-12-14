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

import logging

import snapcraft
import snapcraft.yaml

from snapcraft import (
    common,
    meta,
    pluginhandler,
    repo,
)


logger = logging.getLogger(__name__)


def execute(step, part_names=None):
    """Exectute until step in the lifecycle.

    Lifecycle execution will happen for each step iterating over all
    the available parts, if part_names is specified, only those parts
    will run.

    If one of the parts to execute has an after keyword, execution is
    forced until the stage step for such part. If part_names was provided
    and after is not in this set, an exception will be raised.

    :param str step: A valid step in the lifecycle: pull, build, strip or snap.
    :raises RuntimeError: If a prerequesite of the part needs to be staged
                          and such part is not in the list of parts to iterate
                          over.
    :returns: A dict with the snap name, version and architectures.
    """
    config = snapcraft.yaml.load_config()
    repo.install_build_packages(config.build_tools)

    _Executor(config).run(step, part_names)

    return {'name': config.data['name'],
            'version': config.data['version'],
            'arch': config.data['architectures']}


class _Executor:

    def __init__(self, config):
        self.config = config

    def run(self, step, part_names=None, recursed=False):
        if part_names:
            self.config.validate_parts(part_names)
            parts = {p for p in self.config.all_parts if p.name in part_names}
        else:
            parts = self.config.all_parts
            part_names = self.config.part_names

        dirty = {p.name for p in parts if p.should_step_run('stage')}
        step_index = common.COMMAND_ORDER.index(step) + 1

        for step in common.COMMAND_ORDER[0:step_index]:
            if step == 'stage':
                pluginhandler.check_for_collisions(self.config.all_parts)
            for part in parts:
                self._run_step(step, part, part_names, dirty, recursed)

        self._create_meta(step, part_names)

    def _run_step(self, step, part, part_names, dirty, recursed):
        common.reset_env()
        prereqs = self.config.part_prereqs(part.name)
        if recursed:
            prereqs = prereqs & dirty
        if prereqs and not prereqs.issubset(part_names):
            raise RuntimeError(
                'Requested {!r} of {!r} but there are unsatisfied '
                'prerequisites: {!r}'.format(
                    step, part.name, ' '.join(prereqs)))
        elif prereqs:
            # prerequisites need to build all the way to the staging
            # step to be able to share the common assets that make them
            # a dependency.
            logger.info(
                '{!r} has prerequisites that need to be staged: '
                '{}'.format(part.name, ' '.join(prereqs)))
            self.run('stage', prereqs, recursed=True)

        common.env = self.config.build_env_for_part(part)
        getattr(part, step)()

    def _create_meta(self, step, part_names):
        if step == 'strip' and part_names == self.config.part_names:
            common.env = self.config.snap_env()
            meta.create(self.config.data)
