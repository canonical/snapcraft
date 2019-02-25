# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
import shutil
import time
import urllib

from lazr import restfulclient
from launchpadlib.launchpad import Launchpad
from typing import Any, Dict, List, Tuple
from urllib.parse import urlparse
from xdg import BaseDirectory
from snapcraft import yaml_utils
from .errors import (
    NoLaunchpadUsernameError,
    RemoteBuilderNotReadyError,
    RemoteBuildNotFoundError,
)

import snapcraft

_LP_POLL_INTERVAL = 30
_LP_SUCCESS_STATUS = "Successfully built"
_LP_FAIL_STATUS = "Failed to build"

logger = logging.getLogger(__name__)


class LaunchpadClient:
    def __init__(self, project, build_id: str) -> None:
        self._id = build_id
        self._core_channel = "stable"
        self._snapcraft_channel = "edge"
        self._name = project.info.name
        self._version = project.info.version
        self._waiting = []
        self._data_dir = os.path.join(
            BaseDirectory.save_data_path("snapcraft"), "launchpad"
        )

        os.makedirs(self._data_dir, mode=0o700, exist_ok=True)
        self._cache_dir = os.path.join(self._data_dir, "cache")
        self._credentials = os.path.join(self._data_dir, "credentials")

    def login(self, user: str) -> None:
        if user:
            self._save_info(user=user)
        else:
            info = self._load_info()
            user = info["user"] if "user" in info else None

        if not user:
            raise NoLaunchpadUsernameError

        self.user = user
        self._lp = Launchpad.login_with(
            "snapcraft {}".format(snapcraft.__version__),
            "production",
            self._cache_dir,
            credentials_file=self._credentials,
            version="devel",
        )

    def get_snap(self):
        try:
            return self._lp.snaps.getByName(name=self._id, owner="/~" + self.user)
        except restfulclient.errors.NotFound:
            return None

    def create_snap(self, repository: str, archs: List[str]) -> None:
        logger.debug("Create snap for {}".format(self._id))
        url = repository.replace("git+ssh://", "https://")  # FIXME
        snap = {
            "name": self._id,
            "owner": "/~" + self.user,
            "git_repository_url": url,
            "git_path": "HEAD",
            "auto_build": False,
            "auto_build_archive": "/ubuntu/+archive/primary",
            "auto_build_pocket": "Updates",
        }

        # This shouldn't be necessary, but apparently LP needs this
        if archs:
            snap["processors"] = ["/+processors/" + arch for arch in archs]

        self._lp.snaps.new(**snap)

    def delete_snap(self, snap) -> None:
        snap.lp_delete()

    def start_build(self) -> Tuple[int, str]:
        owner = self._lp.people[self.user]
        dist = self._lp.distributions["ubuntu"]
        snap = self._lp.snaps.getByName(name=self._id, owner=owner)
        request = snap.requestBuilds(
            archive=dist.main_archive,
            channels={"core": self._core_channel, "snapcraft": self._snapcraft_channel},
            pocket="Updates",
        )
        logger.debug("Request URL: {}".format(request))

        request_url = urlparse(request.self_link)
        build_number = os.path.split(request_url.path)[-1]

        ready = False
        for i in range(0, 5):
            builds = self._lp.load(request.builds_collection_link)
            logger.debug(
                "Builds collection entries: {} ({})".format(len(builds.entries), i)
            )
            if builds.entries:
                ready = True
                break
            time.sleep(5)
        if not ready:
            raise RemoteBuilderNotReadyError()

        self._waiting = [build["arch_tag"] for build in builds.entries]
        self._builds_collection_link = request.builds_collection_link

        return build_number

    def recover_build(self, req_number: int) -> None:
        url = "https://api.launchpad.net/devel/~{}/+snap/{}/+build-request/{}".format(
            self.user, self._id, req_number
        )

        try:
            request = self._lp.load(url)
        except restfulclient.errors.NotFound:
            raise RemoteBuildNotFoundError(name=self._name, req_number=req_number)

        logger.debug("Request URL: {}".format(request))
        builds = self._lp.load(request.builds_collection_link)
        self._waiting = [build["arch_tag"] for build in builds.entries]
        self._builds_collection_link = request.builds_collection_link

    def monitor_build(self) -> None:
        logger.debug("Monitoring builds: {}".format(" ".join(self._waiting)))
        while len(self._waiting):
            time.sleep(_LP_POLL_INTERVAL)
            builds = self._lp.load(self._builds_collection_link)

            for build in builds.entries:
                arch = build["arch_tag"]
                web_link = build["web_link"]
                build_state = build["buildstate"]
                logger.debug("{} state: {}".format(arch, build_state))
                if arch in self._waiting:
                    if build_state == _LP_SUCCESS_STATUS:
                        self._process_build(arch, web_link)
                    elif build_state == _LP_FAIL_STATUS:
                        self._process_fail(arch, web_link)

    def show_build_status(self) -> None:
        builds = self._lp.load(self._builds_collection_link)
        for build in builds.entries:
            arch = build["arch_tag"]
            build_state = build["buildstate"]
            logger.info("{}: {}".format(arch, build_state))

    def _process_build(self, arch: str, web_link: str) -> None:
        snap_name = "{}_{}_{}.snap".format(self._name, self._version, arch)
        try:
            self._download_file("{}/+files/{}".format(web_link, snap_name), snap_name)
            logger.info("Snapped {}".format(snap_name))
        except urllib.error.HTTPError as e:
            logger.error("Snap download error ({}): {}".format(arch, e))
        finally:
            self._waiting.remove(arch)

    def _process_fail(self, arch: str, web_link: str) -> None:
        log_name = "buildlog_snap_ubuntu_xenial_{}_{}_BUILDING.txt.gz".format(
            arch, self._id
        )
        fail_log = "buildlog_{}.txt.gz".format(arch)
        try:
            self._download_file("{}/+files/{}".format(web_link, log_name), fail_log)
            logger.error(
                "Build failed for arch {}. Log file is {!r}.".format(arch, fail_log)
            )
        except urllib.error.HTTPError as e:
            logger.error("Log file download error ({}): {}".format(arch, e))
        finally:
            self._waiting.remove(arch)

    def _download_file(self, url: str, name: str) -> None:
        logger.debug("Download snap from {!r}".format(url))
        with urllib.request.urlopen(url) as response, open(name, "wb") as snapfile:
            shutil.copyfileobj(response, snapfile)

    def _load_info(self) -> Dict[str, Any]:
        filepath = os.path.join(self._data_dir, "config.yaml")
        if not os.path.exists(filepath):
            return dict()
        with open(filepath) as info_file:
            return yaml_utils.load(info_file)

    def _save_info(self, **data: Dict[str, Any]) -> None:
        filepath = os.path.join(self._data_dir, "config.yaml")
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(filepath, "w") as info_file:
            yaml_utils.dump(data, stream=info_file)
