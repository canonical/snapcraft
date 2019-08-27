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

import datetime
import logging
import os
import shutil
import time
import urllib.request
import urllib.error
import urllib.parse

from lazr import restfulclient
from launchpadlib.launchpad import Launchpad
from typing import Any, Dict, List, Tuple, Optional
from xdg import BaseDirectory
from . import errors

import snapcraft
from snapcraft.config import Config
from snapcraft.internal.sources._git import Git
from snapcraft.project import Project

_LP_POLL_INTERVAL = 30
_LP_SUCCESS_STATUS = "Successfully built"
_LP_FAIL_STATUS = "Failed to build"

logger = logging.getLogger(__name__)


class LaunchpadClient:
    """Launchpad remote builder operations."""

    def __init__(self, *, project: Project, build_id: str, user: Optional[str]) -> None:
        if not Git.check_command_installed():
            raise errors.GitNotFoundProviderError(provider="Launchpad")

        self._id = build_id
        self._core_channel = "stable"
        self._snapcraft_channel = "edge"
        self._name = project.info.name
        self._waiting = []  # type: List[str]
        self._data_dir = os.path.join(
            BaseDirectory.save_data_path("snapcraft"), "launchpad"
        )
        self._cache_dir = os.path.join(
            BaseDirectory.save_cache_path("snapcraft"), "launchpad"
        )

        os.makedirs(self._data_dir, mode=0o700, exist_ok=True)
        self._credentials = os.path.join(self._data_dir, "credentials")
        self._load_snapcraft_config()

        # Save user to snapcraft config, if specified.
        if user is not None:
            self._user = user
            self._update_snapcraft_config()

        if self._user is None:
            raise errors.NoLaunchpadUsernameError

    def _load_snapcraft_config(self):
        self._config = Config()
        self._config.load()
        escaped_user = self._config.get("username", section_name="Launchpad")
        if escaped_user:
            self._user = urllib.parse.unquote(escaped_user)
        else:
            self._user = None

    def _update_snapcraft_config(self):
        self._config.load()
        escaped_user = urllib.parse.quote(self._user).replace("%", "%%")
        self._config.set("username", escaped_user, section_name="Launchpad")
        self._config.save()

    def login(self) -> None:
        self._lp = Launchpad.login_with(
            "snapcraft remote-build {}".format(snapcraft.__version__),
            "production",
            self._cache_dir,
            credentials_file=self._credentials,
            version="devel",
        )

    def create_snap(self, repository: str, archs: List[str]) -> None:
        """Create a snap recipe."""
        logger.debug("Create snap for {}".format(self._id))
        # TODO: remove this after launchpad infrastructure is ready (LP #1827679)
        url = repository.replace("git+ssh://", "https://")
        snap = {
            "name": self._id,
            "owner": "/~" + self._user,
            "git_repository_url": url,
            "git_path": "master",
            "auto_build": False,
            "auto_build_archive": "/ubuntu/+archive/primary",
            "auto_build_pocket": "Updates",
        }

        if archs:
            snap["processors"] = ["/+processors/" + arch for arch in archs]

        self._lp.snaps.new(**snap)

    def delete_snap(self) -> None:
        """Remove a snap recipe and all associated files."""
        try:
            snap = self._lp.snaps.getByName(name=self._id, owner="/~" + self._user)
            snap.lp_delete()
        except restfulclient.errors.NotFound:
            pass

    def start_build(self, timeout: int = 5, attempts: int = 5) -> int:
        """Initiate a new snap build."""
        owner = self._lp.people[self._user]
        dist = self._lp.distributions["ubuntu"]
        snap = self._lp.snaps.getByName(name=self._id, owner=owner)
        snap_build_request = snap.requestBuilds(
            archive=dist.main_archive,
            channels={"core": self._core_channel, "snapcraft": self._snapcraft_channel},
            pocket="Updates",
        )
        logger.debug("Request URL: {}".format(snap_build_request))

        while attempts > 0 and snap_build_request.status == "Pending":
            logger.debug(
                "Build request: status={} error={}".format(
                    snap_build_request.status, snap_build_request.error_message
                )
            )

            time.sleep(timeout)

            # Refresh status.
            snap_build_request.lp_refresh()
            attempts -= 1

        # Build request failed.
        if snap_build_request.status == "Failed":
            self.delete_snap()
            raise errors.RemoteBuilderError(
                builder_error=snap_build_request.error_message
            )

        # Timed out.
        if snap_build_request.status == "Pending":
            self.delete_snap()
            raise errors.RemoteBuilderNotReadyError()

        # Shouldn't end up here.
        if snap_build_request.status != "Completed":
            self.delete_snap()
            raise errors.RemoteBuilderError(
                builder_error="Unknown builder error - reported status: {}".format(
                    snap_build_request.status
                )
            )

        # Shouldn't end up here either.
        if not snap_build_request.builds.entries:
            self.delete_snap()
            raise errors.RemoteBuilderError(
                builder_error="Unknown builder error - no build entries found."
            )

        self._waiting = [
            build["arch_tag"] for build in snap_build_request.builds.entries
        ]
        self._builds_collection_link = snap_build_request.builds_collection_link
        build_number = snap_build_request.self_link.rsplit("/", 1)[-1]
        return build_number

    def recover_build(self, req_number: int) -> None:
        """Prepare internal state to monitor an existing build."""
        url = "https://api.launchpad.net/devel/~{}/+snap/{}/+build-request/{}".format(
            self._user, self._id, req_number
        )

        try:
            request = self._lp.load(url)
        except restfulclient.errors.NotFound:
            raise errors.RemoteBuildNotFoundError(
                name=self._name, req_number=req_number
            )

        logger.debug("Request URL: {}".format(request))
        builds = self._lp.load(request.builds_collection_link)
        self._waiting = [build["arch_tag"] for build in builds.entries]
        self._builds_collection_link = request.builds_collection_link

    def monitor_build(self, interval: int = _LP_POLL_INTERVAL) -> None:
        """Check build progress, and download artifacts when ready."""
        logger.debug("Monitoring builds: {}".format(" ".join(self._waiting)))
        while len(self._waiting):
            time.sleep(interval)
            builds = self._lp.load(self._builds_collection_link)
            if not builds.entries:
                break

            for build in builds.entries:
                arch = build["arch_tag"]
                build_state = build["buildstate"]
                logger.debug("{} state: {}".format(arch, build_state))
                if arch in self._waiting:
                    if build_state == _LP_SUCCESS_STATUS:
                        self._process_success(build)
                    elif build_state == _LP_FAIL_STATUS:
                        self._process_fail(build)

    def get_build_status(self) -> List[Tuple[str, str]]:
        status = []  # type: List[Tuple[str, str]]
        builds = self._lp.load(self._builds_collection_link)
        for build in builds.entries:
            arch = build["arch_tag"]
            build_state = build["buildstate"]
            status.append((arch, build_state))

        return status

    def _process_success(self, build: Dict[str, Any]) -> None:
        arch = build["arch_tag"]
        snap_build = self._lp.load(build["self_link"])
        urls = snap_build.getFileUrls()
        logger.debug("Success urls: {}".format(urls))
        if not urls:
            logger.error("Snap file not available for arch {}.".format(arch))
            self._waiting.remove(arch)
            return

        self._download_log(build)

        for url in urls:
            snap_name = url.rsplit("/", 1)[-1]
            # Sanity check
            if not snap_name.endswith("_{}.snap".format(arch)):
                continue
            try:
                self._download_file(url, snap_name)
                logger.info("Snapped {}".format(snap_name))
            except urllib.error.HTTPError as e:
                logger.error("Snap download error: {}: {}".format(e, snap_name))

        self._waiting.remove(arch)

    def _process_fail(self, build: Dict[str, Any]) -> None:
        try:
            arch = build["arch_tag"]
            log_name = self._download_log(build)
            logger.error(
                "Build failed for arch {}. Log file is {!r}.".format(arch, log_name)
            )
        except urllib.error.HTTPError as e:
            logger.error("Log file download error ({}): {}".format(arch, e))
        finally:
            self._waiting.remove(arch)

    def _get_logfile_name(self, build: Dict[str, Any]) -> str:
        arch = build["arch_tag"]
        log_name = name = "{}_{}.txt.gz".format(self._name, arch)
        number = 1
        while os.path.isfile(log_name):
            log_name = "{}.{}".format(name, number)
            number += 1
        return log_name

    def _download_log(self, build: Dict[str, Any]) -> str:
        url = build["build_log_url"]
        logger.debug("Build log url: {}".format(url))
        log_name = self._get_logfile_name(build)
        self._download_file(url, log_name)
        return log_name

    def _download_file(self, url: str, name: str) -> None:
        logger.debug("Download snap from {!r}".format(url))
        with urllib.request.urlopen(url) as response, open(name, "wb") as snapfile:
            shutil.copyfileobj(response, snapfile)  # type: ignore

    def _gitify_repository(self, repo_dir: str) -> Git:
        """Git-ify source repository tree.

        :return: Git handler instance to git repository.
        """
        git_handler = Git(repo_dir, repo_dir, silent=True)

        # Init repo.
        git_handler.init()

        # Add all files found in repo.
        for f in os.listdir(repo_dir):
            if f != ".git":
                git_handler.add(f)

        # Commit files.
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        git_handler.commit(
            "snapcraft commit\n\nversion: {}\ntimestamp: {}\n".format(
                snapcraft.__version__, timestamp
            )
        )

        return git_handler

    def push_source_tree(self, repo_dir: str) -> str:
        """Push source tree to launchpad, returning URL."""
        git_handler = self._gitify_repository(repo_dir)

        url = "git+ssh://{user}@git.launchpad.net/~{user}/+git/{id}/".format(
            user=self._user, id=self._id
        )

        logger.info("Sending data to remote builder... ({})".format(url))
        git_handler.push(url, "HEAD:master", force=True)
        return url
