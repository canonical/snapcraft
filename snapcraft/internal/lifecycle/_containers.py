# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import os
import tarfile

from snapcraft.internal import errors, lxd, project_loader


logger = logging.getLogger(__name__)


def _create_tar_filter(tar_filename):
    def _tar_filter(tarinfo):
        fn = tarinfo.name
        if fn.startswith('./parts/') and not fn.startswith('./parts/plugins'):
            return None
        elif fn in ('./stage', './prime', tar_filename):
            return None
        elif fn.endswith('.snap'):
            return None
        elif fn.endswith('_source.tar.bz2'):
            return None
        return tarinfo
    return _tar_filter


def containerbuild(step, project_options, container_config,
                   output=None, args=[]):
    config = project_loader.load_config(project_options)
    if container_config.remote:
        logger.info('Using LXD remote {!r} from SNAPCRAFT_CONTAINER_BUILDS'
                    .format(container_config.remote))
    else:
        logger.info('Using default LXD remote because '
                    'SNAPCRAFT_CONTAINER_BUILDS is set to 1')
    lxd.Project(output=output, source=os.path.curdir,
                project_options=project_options,
                remote=container_config.remote,
                metadata=config.get_metadata()).execute(step, args)


def cleanbuild(project_options, remote=''):
    if remote and not lxd._remote_is_valid(remote):
        raise errors.InvalidContainerRemoteError(remote)

    config = project_loader.load_config(project_options)
    tar_filename = '{}_{}_source.tar.bz2'.format(
        config.data['name'], config.data['version'])

    with tarfile.open(tar_filename, 'w:bz2') as t:
        t.add(os.path.curdir, filter=_create_tar_filter(tar_filename))
    lxd.Cleanbuilder(source=tar_filename,
                     project_options=project_options,
                     metadata=config.get_metadata(), remote=remote).execute()
